#include "driver/i2c.h"
#include "esp_log.h"

/*
 * BH1750 Connections:
 * VCC -> 3.3V
 * GND -> GND
 * SCL -> GPIO 22
 * SDA -> GPIO 21
 * ADDR -> GND (for I2C address 0x23)
 */

/* BH1750 I2C address */
#define BH1750_SENSOR_ADDR 0x23

#define I2C_MASTER_SCL_IO         22 /* GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO         21 /* GPIO number for I2C master data */
#define I2C_MASTER_NUM            I2C_NUM_1 /* I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ        50000 /* Lower I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0     /* I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0     /* I2C master doesn't need buffer */

#define BH1750_POWER_DOWN 0x00 /* Command to set Power Down */
#define BH1750_POWER_ON   0x01 /* Command to set Power On */
#define BH1750_RESET      0x07 /* Command to reset data register value */
#define BH1750_CONT_H_RES_MODE                                                 \
  0x10 /* Start measurement at 1 lux resolution. Measurement time is typically \
          120ms */

static const char *TAG = "BH1750";

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

static esp_err_t bh1750_write_byte(uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | I2C_MASTER_WRITE, true);
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

static esp_err_t bh1750_read(uint8_t *data, size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | I2C_MASTER_READ, true);
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

static void bh1750_init(void) {
  esp_err_t err;
  err = bh1750_write_byte(BH1750_POWER_ON); /* Power on BH1750 */
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "BH1750 power on failed");
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);   /* Delay after power on */
  err = bh1750_write_byte(BH1750_RESET); /* Reset BH1750 */
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "BH1750 reset failed");
  }
  vTaskDelay(10 / portTICK_PERIOD_MS); /* Delay after reset */
}

static float bh1750_read_light_intensity(void) {
  uint8_t   data[2];
  esp_err_t err = bh1750_read(data, 2);
  if (err == ESP_OK) {
    uint16_t raw_light_intensity = (data[0] << 8) | data[1];
    return raw_light_intensity / 1.2; /* Convert raw value to lux */
  } else {
    ESP_LOGE(TAG, "Failed to read data from BH1750");
    return -1.0;
  }
}

void app_main(void) {
  ESP_ERROR_CHECK(i2c_master_init());
  bh1750_init();
  bh1750_write_byte(BH1750_CONT_H_RES_MODE);

  while (1) {
    float light_intensity = bh1750_read_light_intensity();
    if (light_intensity >= 0) {
      ESP_LOGI(TAG, "Light Intensity: %0.2f lux", light_intensity);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
