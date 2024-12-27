/* components/camera/ov7670_hal/ov7670_hal.c */

/* TODO: Test this, 
 * "The OV7670 requires an external clock (typically 12–24 MHz) on the XCLK pin. 
 * Without this, the module will not power up correctly." */

#include "ov7670_hal.h"
#include "common/i2c.h"
#include "esp_log.h"
#include "freertos/task.h"

/* Constants ******************************************************************/

const char    *ov7670_tag                = "OV7670";
const uint8_t  ov7670_scl_io             = GPIO_NUM_22;
const uint8_t  ov7670_sda_io             = GPIO_NUM_21;
const uint8_t  ov7670_i2c_address        = 0x42;
const uint8_t  ov7670_i2c_bus            = I2C_NUM_0;
const uint32_t ov7670_i2c_freq_hz        = 100000;
const uint32_t ov7670_polling_rate_ticks = pdMS_TO_TICKS(5000);

/* Private Functions (Static) *************************************************/

/**
 * @brief Applies the specified configuration settings to the OV7670 module.
 *
 * @param[in] config The configuration structure specifying resolution, format, and clock settings.
 * @return
 * - `ESP_OK` on success.
 * - Error codes from `esp_err_t` on failure.
 */
static esp_err_t priv_ov7670_apply_config(const ov7670_config_t *config)
{
  esp_err_t ret;

  /* Set resolution and format (COM7 register) */
  uint8_t com7_value = config->resolution | config->output_format;
  ret                = priv_i2c_write_reg_byte(k_ov7670_reg_com7, com7_value, 
                                               ov7670_i2c_bus, ov7670_i2c_address, 
                                               ov7670_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(ov7670_tag, "Failed to set resolution and format");
    return ret;
  }

  /* Set clock divider (CLKRC register) */
  ret = priv_i2c_write_reg_byte(k_ov7670_reg_clkrc, config->clock_divider, 
                                ov7670_i2c_bus, ov7670_i2c_address, ov7670_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(ov7670_tag, "Failed to set clock divider");
    return ret;
  }

  ESP_LOGI(ov7670_tag, "Configuration applied: Resolution=0x%02X, "
                       "Format=0x%02X, Clock Divider=0x%02X",
      config->resolution, config->output_format, config->clock_divider);

  return ESP_OK;
}

/**
 * @brief Applies default configuration settings to the OV7670 module.
 *
 * @return
 * - `ESP_OK` on success.
 * - Error codes from `esp_err_t` on failure.
 */
static esp_err_t priv_ov7670_configure_defaults(void)
{
  ov7670_config_t default_config = {
    .resolution    = k_ov7670_res_qvga,
    .output_format = k_ov7670_output_rgb,
    .clock_divider = k_ov7670_clk_div_2,
  };

  return priv_ov7670_apply_config(&default_config);
}

/* Public Functions ***********************************************************/

esp_err_t ov7670_init(ov7670_data_t *camera_data)
{
  ESP_LOGI(ov7670_tag, "Initializing OV7670 Camera");

  /* Initialize I2C interface */
  esp_err_t ret = priv_i2c_init(ov7670_scl_io, ov7670_sda_io, ov7670_i2c_freq_hz, 
                                ov7670_i2c_bus, ov7670_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(ov7670_tag, "I2C initialization failed");
    camera_data->state = k_ov7670_config_error;
    return ret;
  }

  /* Apply default settings */
  ret = priv_ov7670_configure_defaults();
  if (ret != ESP_OK) {
    ESP_LOGE(ov7670_tag, "Default configuration failed");
    camera_data->state = k_ov7670_config_error;
    return ret;
  }

  camera_data->state = k_ov7670_ready;
  ESP_LOGI(ov7670_tag, "OV7670 initialization complete");
  return ESP_OK;
}

esp_err_t ov7670_configure(ov7670_data_t *camera_data)
{
  ESP_LOGI(ov7670_tag, "Applying new configuration to OV7670 Camera");

  /* Apply the configuration stored in camera_data */
  esp_err_t ret = priv_ov7670_apply_config(&camera_data->config);
  if (ret != ESP_OK) {
    ESP_LOGE(ov7670_tag, "Configuration failed");
    camera_data->state = k_ov7670_config_error;
    return ret;
  }

  ESP_LOGI(ov7670_tag, "Configuration successfully applied");
  return ESP_OK;
}

void ov7670_reset_on_error(ov7670_data_t *camera_data)
{
  if (camera_data->state == k_ov7670_config_error) {
    TickType_t current_ticks = xTaskGetTickCount();

    if (current_ticks - camera_data->last_attempt_ticks >= camera_data->retry_interval) {
      ESP_LOGW(ov7670_tag, "Attempting to reset OV7670");

      if (ov7670_init(camera_data) == ESP_OK) {
        camera_data->retries        = 0;
        camera_data->retry_interval = pdMS_TO_TICKS(15000);
      } else {
        camera_data->retries++;
        camera_data->retry_interval = pdMS_TO_TICKS(camera_data->retry_interval * 2);
      }

      camera_data->last_attempt_ticks = current_ticks;
    }
  }
}

void ov7670_tasks(void *camera_data_)
{
  ov7670_data_t *camera_data = (ov7670_data_t *)camera_data_;

  while (1) {
    if (camera_data->state == k_ov7670_config_error) {
      ov7670_reset_on_error(camera_data);
    }

    vTaskDelay(ov7670_polling_rate_ticks);
  }
}

