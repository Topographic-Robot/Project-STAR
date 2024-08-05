#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

/*
 * QMC5883L Connections:
 * VCC -> 3.3V
 * GND -> GND
 * SCL -> GPIO 22
 * SDA -> GPIO 21
 */

/* QMC5883L I2C address */
#define QMC5883L_SENSOR_ADDR 0x0D

#define I2C_MASTER_SCL_IO         22 /* GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO         21 /* GPIO number for I2C master data */
#define I2C_MASTER_NUM            I2C_NUM_1 /* I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ        50000 /* Lower I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0     /* I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0     /* I2C master doesn't need buffer */

#define QMC5883L_XOUT_L  0x00 /* Register to read X-axis low byte */
#define QMC5883L_CONTROL 0x09 /* Register to control QMC5883L */
#define QMC5883L_RST_REG 0x0B /* Register to reset QMC5883L */
#define QMC5883L_STATUS  0x06 /* Register to read the status */

static const char *TAG = "QMC5883L";

static esp_err_t i2c_master_init(void) {
  i2c_config_t conf = {
      .mode             = I2C_MODE_MASTER,
      .sda_io_num       = I2C_MASTER_SDA_IO,
      .sda_pullup_en    = GPIO_PULLUP_ENABLE,
      .scl_io_num       = I2C_MASTER_SCL_IO,
      .scl_pullup_en    = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
    return err;
  }
  err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE,
                           I2C_MASTER_TX_BUF_DISABLE, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
  }
  return err;
}

static esp_err_t qmc5883l_write_byte(uint8_t reg_addr, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, QMC5883L_SENSOR_ADDR << 1 | I2C_MASTER_WRITE,
                        true);
  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);
  esp_err_t err =
      i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(err));
  }
  return err;
}

static esp_err_t qmc5883l_read(uint8_t reg_addr, uint8_t *data, size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, QMC5883L_SENSOR_ADDR << 1 | I2C_MASTER_WRITE,
                        true);
  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, QMC5883L_SENSOR_ADDR << 1 | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t err =
      i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
  }
  return err;
}

static void qmc5883l_init(void) {
  esp_err_t err;
  err = qmc5883l_write_byte(QMC5883L_RST_REG, 0x01); /* Reset QMC5883L */
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "QMC5883L reset failed");
  }
  vTaskDelay(100 / portTICK_PERIOD_MS); /* Delay after reset */
  err = qmc5883l_write_byte(QMC5883L_CONTROL,
                            0x1D); /* Set QMC5883L to continuous mode, 200Hz, 2G
                                      range, 512 oversampling */
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "QMC5883L control setup failed");
  }
}

static const char *get_direction(float heading) {
  if (heading >= 337.5 || heading < 22.5)
    return "N";
  else if (heading >= 22.5 && heading < 67.5)
    return "NE";
  else if (heading >= 67.5 && heading < 112.5)
    return "E";
  else if (heading >= 112.5 && heading < 157.5)
    return "SE";
  else if (heading >= 157.5 && heading < 202.5)
    return "S";
  else if (heading >= 202.5 && heading < 247.5)
    return "SW";
  else if (heading >= 247.5 && heading < 292.5)
    return "W";
  else if (heading >= 292.5 && heading < 337.5)
    return "NW";
  return "Unknown";
}

static void qmc5883l_read_data(void) {
  uint8_t data[6];
  /* mag_z isnt used */
  int16_t mag_x, mag_y, mag_z;

  esp_err_t err = qmc5883l_read(QMC5883L_XOUT_L, data, 6);
  if (err == ESP_OK) {
    /* XXX: Explain this */
    mag_x = (int16_t)(data[1] << 8 | data[0]);
    mag_y = (int16_t)(data[3] << 8 | data[2]);
    mag_z = (int16_t)(data[5] << 8 | data[4]);

    float heading = atan2((float)mag_y, (float)mag_x) * 180 / M_PI;
    if (heading < 0) {
      heading += 360;
    }

    const char *direction = get_direction(heading);
    ESP_LOGI(TAG, "%0.2f deg %s", heading, direction);
  } else {
    ESP_LOGE(TAG, "Failed to read data from QMC5883L");
  }
}

void app_main(void) {
  ESP_ERROR_CHECK(i2c_master_init());
  qmc5883l_init();

  while (1) {
    qmc5883l_read_data();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
