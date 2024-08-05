#include "bh1750.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "BH1750";

/* private functions *********************************************************
 */
static esp_err_t _i2c_init(uint8_t scl_io, uint8_t sda_io, uint32_t freq_hz,
                           uint8_t i2c_bus) {
  i2c_config_t conf = {
      .mode             = I2C_MODE_MASTER,
      .sda_io_num       = sda_io,
      .sda_pullup_en    = GPIO_PULLUP_ENABLE,
      .scl_io_num       = scl_io,
      .scl_pullup_en    = GPIO_PULLUP_ENABLE,
      .master.clk_speed = freq_hz,
  };

  esp_err_t err = i2c_param_config(i2c_bus, &conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
    return err;
  }
  /* I2C master doesnt need buffer */
  return i2c_driver_install(i2c_bus, conf.mode, 0, 0, 0);
}

static esp_err_t _bh1750_write_byte(uint8_t data, uint8_t i2c_bus) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);

  esp_err_t err = i2c_master_cmd_begin(i2c_bus, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(err));
  }
  return err;
}

static esp_err_t _bh1750_read(uint8_t *data, size_t len, uint8_t i2c_bus) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(i2c_bus, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
  }
  return err;
}

/* public functions **********************************************************
 */

esp_err_t bh1750_init(uint8_t scl_io, uint8_t sda_io, uint32_t freq_hz,
                      uint8_t i2c_bus) {
  esp_err_t err = _i2c_init(scl_io, sda_io, freq_hz, i2c_bus);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    return err;
  }

  /* power on device */
  err = _bh1750_write_byte(BH1750_POWER_ON, i2c_bus);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "BH1750 power on failed");
    return err;
  }
  vTaskDelay(10 / portTICK_PERIOD_MS); /* Delay after power on */

  /* reset the device */
  err = _bh1750_write_byte(BH1750_RESET, i2c_bus);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "BH1750 reset failed");
    return err;
  }
  vTaskDelay(10 / portTICK_PERIOD_MS); /* Delay after reset */

  /* set the resolution mode */
  err = _bh1750_write_byte(BH1750_CONT_H_RES_MODE, i2c_bus);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "BH1750 setting resolution mode failed");
    return err;
  }
  vTaskDelay(10 / portTICK_PERIOD_MS); /* Delay after H_RES_MODE */

  return err;
}

float bh1750_read_lux(uint8_t i2c_bus) {
  uint8_t   data[2];
  esp_err_t err = _bh1750_read(data, 2, i2c_bus);
  if (err == ESP_OK) {
    uint16_t raw_light_intensity = (data[0] << 8) | data[1];
    return raw_light_intensity / 1.2; /* Convert raw value to lux */
  } else {
    ESP_LOGE(TAG, "Failed to read data from BH1750");
    return -1.0;
  }
}
