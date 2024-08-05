#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define I2C_MASTER_SCL_IO         22 /* GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO         21 /* GPIO number for I2C master data */
#define I2C_MASTER_NUM            I2C_NUM_0 /* I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ        100000    /* I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0         /* I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0         /* I2C master doesn't need buffer */
#define HMC5883L_ADDR             0x1E /* Slave address for HMC5883L sensor */

#define WRITE_BIT     I2C_MASTER_WRITE /* I2C master write */
#define READ_BIT      I2C_MASTER_READ  /* I2C master read */
#define ACK_CHECK_EN  0x1 /* I2C master will check ack from slave */
#define ACK_CHECK_DIS 0x0 /* I2C master will not check ack from slave */
#define ACK_VAL       0x0 /* I2C ack value */
#define NACK_VAL      0x1 /* I2C nack value */

static const char *TAG = "HMC5883L";

static esp_err_t i2c_master_init(void) {
  i2c_config_t conf = {
      .mode             = I2C_MODE_MASTER,
      .sda_io_num       = I2C_MASTER_SDA_IO,
      .sda_pullup_en    = GPIO_PULLUP_ENABLE,
      .scl_io_num       = I2C_MASTER_SCL_IO,
      .scl_pullup_en    = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  i2c_param_config(I2C_MASTER_NUM, &conf);
  return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t hmc5883l_write_byte(uint8_t reg_addr, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  /* XXX: Comments are needed */
  i2c_master_write_byte(cmd, (HMC5883L_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

static esp_err_t hmc5883l_read_bytes(uint8_t reg_addr, uint8_t *data,
                                     size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  /* XXX: Comments are needed */
  i2c_master_write_byte(cmd, (HMC5883L_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
  i2c_master_start(cmd);
  /* XXX: Comments are needed */
  i2c_master_write_byte(cmd, (HMC5883L_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
  if (len > 1) {
    i2c_master_read(cmd, data, len - 1, ACK_VAL);
  }
  i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

static void hmc5883l_init(void) {
  /* Initialize the HMC5883L */
  esp_err_t ret;
  ret = hmc5883l_write_byte(0x00,
                            0x70); /* Configuration Register A: 8-average, */
                                   /* 15 Hz default, normal measurement */
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write to configuration register A");
  }
  vTaskDelay(100 / portTICK_PERIOD_MS); /* Short delay */

  ret = hmc5883l_write_byte(0x01, 0xA0); /* Configuration Register B: Gain=5 */
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write to configuration register B");
  }
  vTaskDelay(100 / portTICK_PERIOD_MS); /* Short delay */

  ret = hmc5883l_write_byte(
      0x02, 0x00); /* Mode Register: Continuous measurement mode */
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write to mode register");
  }
  vTaskDelay(100 / portTICK_PERIOD_MS); /* Short delay */
}

static void hmc5883l_read_magnetometer(int16_t *mag_x, int16_t *mag_y,
                                       int16_t *mag_z) {
  uint8_t   data[6]; /* XXX: Move 6 into some macro? */
  esp_err_t ret =
      hmc5883l_read_bytes(0x03, data, 6); /* XXX: Duplicating 6 again here */
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read magnetometer data");
  } else {
    /* XXX: Some explaination of what this does would be nice */
    *mag_x = (int16_t)((data[0] << 8) | data[1]);
    *mag_y = (int16_t)((data[4] << 8) | data[5]);
    *mag_z = (int16_t)((data[2] << 8) | data[3]);
  }
}

void app_main(void) {
  ESP_ERROR_CHECK(i2c_master_init());
  hmc5883l_init();

  while (1) {
    int16_t mag_x, mag_y, mag_z;
    hmc5883l_read_magnetometer(&mag_x, &mag_y, &mag_z);
    ESP_LOGI(TAG, "Magnetometer X: %d, Y: %d, Z: %d", mag_x, mag_y, mag_z);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
