#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define I2C_MASTER_SCL_IO         22 /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO         21 /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM            I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ        100000    /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS     1000

#define PCA9685_ADDR 0x40 /*!< Slave address for PCA9685 */

#define PCA9685_MODE1    0x00
#define PCA9685_MODE2    0x01
#define PCA9685_PRESCALE 0xFE

static const char *TAG = "pca9685_servo";

// Initialize I2C
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
    ESP_LOGE(TAG, "I2C param config failed");
    return err;
  }
  return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Write to PCA9685 register
static esp_err_t pca9685_write(uint8_t reg_addr, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  return err;
}

// Set PWM frequency
static esp_err_t pca9685_set_pwm_freq(float freq) {
  uint8_t prescale_val = (uint8_t)(25000000 / (4096 * freq) - 1);
  ESP_ERROR_CHECK(pca9685_write(PCA9685_MODE1, 0x10)); // Enter sleep mode
  ESP_ERROR_CHECK(pca9685_write(PCA9685_PRESCALE, prescale_val));
  ESP_ERROR_CHECK(pca9685_write(PCA9685_MODE1, 0x80)); // Restart
  ESP_ERROR_CHECK(pca9685_write(PCA9685_MODE2, 0x04)); // Output logic
  return ESP_OK;
}

// Set PWM value for a specific channel
static esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
  ESP_ERROR_CHECK(pca9685_write(0x06 + 4 * channel, on & 0xFF));
  ESP_ERROR_CHECK(pca9685_write(0x07 + 4 * channel, on >> 8));
  ESP_ERROR_CHECK(pca9685_write(0x08 + 4 * channel, off & 0xFF));
  ESP_ERROR_CHECK(pca9685_write(0x09 + 4 * channel, off >> 8));
  return ESP_OK;
}

void app_main(void) {
  ESP_ERROR_CHECK(i2c_master_init());
  ESP_ERROR_CHECK(pca9685_set_pwm_freq(50)); // Set frequency to 50Hz for servos

  while (1) {
    // Move servo to left (1ms pulse width)
    ESP_ERROR_CHECK(pca9685_set_pwm(0, 0, 204)); // 1ms pulse (204/4096 * 20ms)
    vTaskDelay(pdMS_TO_TICKS(1000));             // Wait for 1 second

    // Move servo to right (2ms pulse width)
    ESP_ERROR_CHECK(pca9685_set_pwm(0, 0, 409)); // 2ms pulse (409/4096 * 20ms)
    vTaskDelay(pdMS_TO_TICKS(1000));             // Wait for 1 second
  }
}
