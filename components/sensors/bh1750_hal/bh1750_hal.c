/* components/sensors/bh1750_hal/bh1750_hal.c */

#include "bh1750_hal.h"
#include "common/i2c.h"
#include "esp_log.h"

/* Constants ******************************************************************/

const uint8_t  bh1750_i2c_address            = 0x23;
const uint8_t  bh1750_i2c_bus                = I2C_NUM_0;
const char    *bh1750_tag                    = "BH1750";
const uint8_t  bh1750_scl_io                 = GPIO_NUM_22;
const uint8_t  bh1750_sda_io                 = GPIO_NUM_21;
const uint32_t bh1750_i2c_freq_hz            = 100000;
const uint32_t bh1750_polling_rate_ticks     = pdMS_TO_TICKS(5 * 1000);
const uint8_t  bh1750_max_retries            = 4;
const uint32_t bh1750_initial_retry_interval = pdMS_TO_TICKS(15);
const uint32_t bh1750_max_backoff_interval   = pdMS_TO_TICKS(8 * 60);

/* Public Functions ***********************************************************/

esp_err_t bh1750_init(void *sensor_data)
{
  bh1750_data_t *bh1750_data = (bh1750_data_t *)sensor_data;
  ESP_LOGI(bh1750_tag, "Starting Configuration");

  bh1750_data->i2c_address        = bh1750_i2c_address;
  bh1750_data->i2c_bus            = bh1750_i2c_bus;
  bh1750_data->lux                = -1.0;
  bh1750_data->state              = k_bh1750_uninitialized;
  bh1750_data->retry_count        = 0;
  bh1750_data->retry_interval     = bh1750_initial_retry_interval;
  bh1750_data->last_attempt_ticks = 0;

  /* Initialize the I2C bus */
  esp_err_t ret = priv_i2c_init(bh1750_scl_io, bh1750_sda_io, bh1750_i2c_freq_hz, 
                                bh1750_i2c_bus, bh1750_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(bh1750_tag, "I2C driver install failed: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Power on and reset the sensor */
  ret = priv_i2c_write_byte(k_bh1750_power_on_cmd, bh1750_i2c_bus, 
                            bh1750_i2c_address, bh1750_tag);
  if (ret != ESP_OK) {
    bh1750_data->state = k_bh1750_power_on_error;
    return ret;
  }

  ret = priv_i2c_write_byte(k_bh1750_reset_cmd, bh1750_i2c_bus, 
                            bh1750_i2c_address, bh1750_tag);
  if (ret != ESP_OK) {
    bh1750_data->state = k_bh1750_reset_error;
    return ret;
  }

  bh1750_data->state = k_bh1750_ready;
  ESP_LOGI(bh1750_tag, "Sensor Configuration Complete");
  return ESP_OK;
}

void bh1750_read(bh1750_data_t *sensor_data) 
{
  uint8_t   data[2];
  esp_err_t ret = priv_i2c_read_bytes(data, 2, bh1750_i2c_bus, 
                                      bh1750_i2c_address, bh1750_tag);
  if (ret != ESP_OK) {
    sensor_data->lux   = -1.0;
    sensor_data->state = k_bh1750_error;
    ESP_LOGE(bh1750_tag, "Failed to read data from BH1750");
    return;
  }

  uint16_t raw_light_intensity = (data[0] << 8) | data[1];
  sensor_data->lux             = raw_light_intensity / 1.2;
  ESP_LOGI(bh1750_tag, "Measured light intensity: %f lux", sensor_data->lux);
}

void bh1750_reset_on_error(bh1750_data_t *sensor_data) 
{
  if (sensor_data->state & k_bh1750_error) {
    TickType_t now_ticks = xTaskGetTickCount();
    if (now_ticks - sensor_data->last_attempt_ticks >= sensor_data->retry_interval) {
      sensor_data->last_attempt_ticks = now_ticks;

      if (bh1750_init(sensor_data) == ESP_OK) {
        sensor_data->state          = k_bh1750_ready;
        sensor_data->retry_count    = 0;
        sensor_data->retry_interval = bh1750_initial_retry_interval;
      } else {
        sensor_data->retry_count += 1;

        if (sensor_data->retry_count >= bh1750_max_retries) {
          sensor_data->retry_count    = 0;
          sensor_data->retry_interval = (sensor_data->retry_interval * 2 <= bh1750_max_backoff_interval) ?
                                        sensor_data->retry_interval * 2 : 
                                        bh1750_max_backoff_interval;
        }
      }
    }
  }
}

void bh1750_tasks(void *sensor_data)
{
  bh1750_data_t *bh1750_data = (bh1750_data_t *)sensor_data;
  while (1) {
    bh1750_read(bh1750_data);
    bh1750_reset_on_error(bh1750_data);
    vTaskDelay(bh1750_polling_rate_ticks);
  }
}
