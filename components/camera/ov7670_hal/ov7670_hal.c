/* components/camera/ov7670_hal/ov7670_hal.c */

/* TODO: Test this */

#include "ov7670_hal.h"
#include "common/i2c.h"
#include "esp_log.h"
#include "freertos/task.h"

/* Constants ******************************************************************/

const char    *ov7670_tag                = "OV7670";
const uint8_t  ov7670_scl_io             = GPIO_NUM_22;
const uint8_t  ov7670_sda_io             = GPIO_NUM_21;
const uint8_t  ov7670_i2c_address        = 0x42;
const uint32_t ov7670_polling_rate_ticks = pdMS_TO_TICKS(5000);

/* Private Functions **********************************************************/

/**
 * @brief Applies default configuration settings to the OV7670 module.
 *
 * @return
 * - `ESP_OK` on success.
 * - Error codes from `esp_err_t` on failure.
 */
static esp_err_t ov7670_configure_defaults(void)
{
  esp_err_t ret;

  /* Reset all registers */
  ret = priv_i2c_write_reg_byte(0x12, 0x80, I2C_NUM_0, ov7670_i2c_address, ov7670_tag);
  if (ret != ESP_OK) {
    return ret;
  }

  vTaskDelay(pdMS_TO_TICKS(100)); /* Allow reset to complete */

  /* Example: Set QVGA resolution and RGB output */
  ret = priv_i2c_write_reg_byte(0x12, 0x14, I2C_NUM_0, ov7670_i2c_address, ov7670_tag);
  if (ret != ESP_OK) {
    return ret;
  }

  ESP_LOGI(ov7670_tag, "Default configuration applied");
  return ESP_OK;
}

/* Public Functions ***********************************************************/

esp_err_t ov7670_init(ov7670_data_t *sensor_data)
{
  ESP_LOGI(ov7670_tag, "Initializing OV7670 Camera");

  /* Initialize I2C interface */
  esp_err_t ret = priv_i2c_init(ov7670_scl_io, ov7670_sda_io, 100000, I2C_NUM_0, ov7670_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(ov7670_tag, "I2C initialization failed");
    sensor_data->state = k_ov7670_config_error;
    return ret;
  }

  /* Apply default settings */
  ret = ov7670_configure_defaults();
  if (ret != ESP_OK) {
    ESP_LOGE(ov7670_tag, "Default configuration failed");
    sensor_data->state = k_ov7670_config_error;
    return ret;
  }

  sensor_data->state = k_ov7670_ready;
  ESP_LOGI(ov7670_tag, "OV7670 initialization complete");
  return ESP_OK;
}

void ov7670_reset_on_error(ov7670_data_t *sensor_data)
{
  if (sensor_data->state == k_ov7670_config_error) {
    TickType_t current_ticks = xTaskGetTickCount();

    if (current_ticks - sensor_data->last_attempt_ticks >= sensor_data->retry_interval) {
      ESP_LOGW(ov7670_tag, "Attempting to reset OV7670");

      if (ov7670_init(sensor_data) == ESP_OK) {
        sensor_data->retries = 0;
        sensor_data->retry_interval = pdMS_TO_TICKS(15000);
      } else {
        sensor_data->retries++;
        sensor_data->retry_interval = pdMS_TO_TICKS(sensor_data->retry_interval * 2);
      }

      sensor_data->last_attempt_ticks = current_ticks;
    }
  }
}

void ov7670_tasks(void *sensor_data)
{
  ov7670_data_t *camera_data = (ov7670_data_t *)sensor_data;

  while (1) {
    if (camera_data->state == k_ov7670_config_error) {
      ov7670_reset_on_error(camera_data);
    }

    vTaskDelay(ov7670_polling_rate_ticks);
  }
}

