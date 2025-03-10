/* components/camera/ov7670_hal/ov7670_hal.c */

/* 
 * "The OV7670 requires an external clock (12–24 MHz) on the XCLK pin.
 *  Without this, the module will not power up correctly."
 */

#include "ov7670_hal.h"
#include <inttypes.h>
#include "common/i2c.h"
#include "freertos/task.h"
#include "log_handler.h"
#include "error_handler.h"
#include "common/common_setup.h"
#include "common/common_cleanup.h"

#ifdef USE_OV7670_XCLK_GPIO_27
#include "driver/ledc.h"
#endif

/* Contants *******************************************************************/

/* 
 * If we define USE_OV7670_XCLK_GPIO_27, we will configure LEDC on GPIO_NUM_27
 * to generate the XCLK. Otherwise, do nothing (external clock assumed).
 */
#ifdef USE_OV7670_XCLK_GPIO_27
static const uint32_t   ov7670_xclk_freq_hz = 24000000; /* 24 MHz */
static const gpio_num_t ov7670_xclk_gpio    = GPIO_NUM_27;
#endif

const char* const ov7670_tag                = "OV7670";
const uint8_t     ov7670_i2c_address        = (0x42 >> 1);         /* 7-bit address */
const i2c_port_t  ov7670_i2c_bus            = I2C_NUM_0;
const uint32_t    ov7670_i2c_freq_hz        = 100000;              /* 100 kHz */
const uint32_t    ov7670_polling_rate_ticks = pdMS_TO_TICKS(5000);
const uint8_t     ov7670_scl_io             = GPIO_NUM_22;
const uint8_t     ov7670_sda_io             = GPIO_NUM_21;
const uint8_t     ov7670_max_retries        = 5;
const uint32_t    ov7670_retry_delay_ms     = 1000;

/* Private Variables **********************************************************/

static error_handler_t s_ov7670_error_handler;

/* Private (Static) Functions *************************************************/

#ifdef USE_OV7670_XCLK_GPIO_27
/**
 * @brief Configure LEDC to generate XCLK on GPIO 27.
 * @param freq_hz The desired clock frequency (e.g., 24MHz).
 * @return ESP_OK on success, or error code on failure.
 */
static esp_err_t priv_configure_xclk_on_gpio_27(uint32_t freq_hz)
{
  log_info(ov7670_tag, 
           "XCLK Setup", 
           "Setting up GPIO 27 at %" PRIu32 " Hz", 
           freq_hz);

  /* Calculate LEDC timer parameters */
  uint32_t duty = 1 << 15; /* 50% duty cycle (half of 2^16) */
  
  /* Configure LEDC timer */
  ledc_timer_config_t ledc_timer = {
    .duty_resolution = LEDC_TIMER_15_BIT,
    .freq_hz         = freq_hz,
    .speed_mode      = LEDC_HIGH_SPEED_MODE,
    .timer_num       = LEDC_TIMER_0,
    .clk_cfg         = LEDC_AUTO_CLK,
  };
  
  esp_err_t ret = ledc_timer_config(&ledc_timer);
  if (ret != ESP_OK) {
    ERROR_HARDWARE(&s_ov7670_error_handler, ret, ERROR_SEVERITY_HIGH, 
                  "Failed to configure LEDC timer for XCLK");
    return ret;
  }
  
  /* Configure LEDC channel */
  ledc_channel_config_t ledc_channel = {
    .channel    = LEDC_CHANNEL_0,
    .duty       = duty,
    .gpio_num   = ov7670_xclk_gpio,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .hpoint     = 0,
    .timer_sel  = LEDC_TIMER_0,
  };
  
  ret = ledc_channel_config(&ledc_channel);
  if (ret != ESP_OK) {
    ERROR_HARDWARE(&s_ov7670_error_handler, ret, ERROR_SEVERITY_HIGH, 
                  "Failed to configure LEDC channel for XCLK");
    return ret;
  }
  
  return ESP_OK;
}
#endif

/**
 * @brief Apply configuration settings to the OV7670 camera.
 * @param config Pointer to the configuration structure.
 * @return ESP_OK on success, or error code on failure.
 */
static esp_err_t priv_ov7670_apply_config(const ov7670_config_t* const config)
{
  if (!config) {
    ERROR_HARDWARE(&s_ov7670_error_handler, ESP_ERR_INVALID_ARG, ERROR_SEVERITY_MEDIUM, 
                  "NULL configuration pointer");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* TODO: Implement configuration register writes based on the config struct */
  /* This is a placeholder for actual register configuration */
  
  log_info(ov7670_tag, 
           "Config Applied", 
           "Applied configuration: res=%u, fmt=%u, clk_div=%u",
           config->resolution, 
           config->output_format, 
           config->clock_divider);
  
  return ESP_OK;
}

/**
 * @brief Configure the OV7670 with default settings.
 * @return ESP_OK on success, or error code on failure.
 */
static esp_err_t priv_ov7670_configure_defaults(void)
{
  /* TODO: Implement default register configuration */
  /* This is a placeholder for actual register configuration */
  
  log_info(ov7670_tag, "Default Config", "Applied default configuration");
  return ESP_OK;
}

/**
 * @brief Reset function for error handler.
 */
static esp_err_t priv_ov7670_reset(void* context)
{
  ov7670_data_t* const camera_data = (ov7670_data_t*)context;
  if (!camera_data) {
    log_error(ov7670_tag, "Reset Error", "Camera data pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  log_info(ov7670_tag, "Reset", "Attempting to reset OV7670 camera");
  
  /* Cleanup existing resources */
  ov7670_cleanup(camera_data);
  
  /* Reinitialize */
  return ov7670_init(camera_data);
}

/* Public Functions ***********************************************************/

esp_err_t ov7670_init(ov7670_data_t* const camera_data)
{
  if (!camera_data) {
    log_error(ov7670_tag, 
              "Invalid Parameter", 
              "Camera data pointer is NULL, cannot proceed with initialization");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(ov7670_tag, 
           "Init Started", 
           "Beginning OV7670 camera initialization sequence");
           
  /* Initialize error handler */
  error_handler_init(&s_ov7670_error_handler, 
                    ov7670_tag,
                    ov7670_max_retries, 
                    ov7670_retry_delay_ms,
                    ov7670_retry_delay_ms * 10, 
                    priv_ov7670_reset,
                    camera_data, 
                    ov7670_retry_delay_ms,
                    ov7670_retry_delay_ms * 20);
  
  /* Register component with system error handler */
  component_info_t ov7670_component = {
    .component_id = "ov7670",
    .handler = &s_ov7670_error_handler,
    .parent_id = NULL,
    .priority = 5
  };
  
  esp_err_t ret = error_handler_register_component(&ov7670_component);
  if (ret != ESP_OK) {
    log_error(ov7670_tag, "Error Handler", "Failed to register component with error handler");
    return ESP_FAIL;
  }

  /* 1. Initialize I2C interface */
  ret = priv_i2c_init(ov7670_scl_io, 
                      ov7670_sda_io,
                      ov7670_i2c_freq_hz, 
                      ov7670_i2c_bus,
                      ov7670_tag);
  if (ret != ESP_OK) {
    ERROR_HARDWARE(&s_ov7670_error_handler, ret, ERROR_SEVERITY_HIGH, 
                  "Failed to initialize I2C interface");
    log_error(ov7670_tag, 
              "I2C Error", 
              "Failed to initialize I2C interface: %s", 
              esp_err_to_name(ret));
    camera_data->state = k_ov7670_config_error;
    return ret;
  }

#ifdef USE_OV7670_XCLK_GPIO_27
  /* 2. Configure the ESP32 to generate the XCLK on GPIO_NUM_27 */
  ret = priv_configure_xclk_on_gpio_27(ov7670_xclk_freq_hz);
  if (ret != ESP_OK) {
    ERROR_HARDWARE(&s_ov7670_error_handler, ret, ERROR_SEVERITY_HIGH, 
                  "Failed to configure external clock on GPIO 27");
    log_error(ov7670_tag, 
              "XCLK Error", 
              "Failed to configure external clock on GPIO 27");
    camera_data->state = k_ov7670_config_error;
    return ret;
  }
  log_info(ov7670_tag, 
           "XCLK Ready", 
           "External clock configured on GPIO 27 at %" PRIu32 " Hz", 
           ov7670_xclk_freq_hz);
#else
  /* If not defined, do nothing: we assume an external clock is provided */
  log_info(ov7670_tag, 
           "Clock Mode", 
           "Using external clock source for camera timing");
#endif

  /* 3. Apply default settings */
  ret = priv_ov7670_configure_defaults();
  if (ret != ESP_OK) {
    ERROR_HARDWARE(&s_ov7670_error_handler, ret, ERROR_SEVERITY_HIGH, 
                  "Failed to apply default camera settings");
    log_error(ov7670_tag, 
              "Config Error", 
              "Failed to apply default camera settings");
    camera_data->state = k_ov7670_config_error;
    return ret;
  }

  camera_data->state = k_ov7670_ready;
  log_info(ov7670_tag, 
           "Init Complete", 
           "Camera initialized successfully (state=%u)", 
           camera_data->state);
           
  /* Record successful status in error handler */
  error_handler_record_status(&s_ov7670_error_handler, ESP_OK);
  
  return ESP_OK;
}

esp_err_t ov7670_configure(ov7670_data_t* const camera_data)
{
  if (!camera_data) {
    ERROR_HARDWARE(&s_ov7670_error_handler, ESP_ERR_INVALID_ARG, ERROR_SEVERITY_LOW, 
                  "NULL camera data pointer");
    return ESP_ERR_INVALID_ARG;
  }
  
  if (camera_data->state != k_ov7670_ready) {
    ERROR_HARDWARE(&s_ov7670_error_handler, ESP_ERR_INVALID_STATE, ERROR_SEVERITY_MEDIUM, 
                  "Camera not in ready state");
    return ESP_ERR_INVALID_STATE;
  }
  
  esp_err_t ret = priv_ov7670_apply_config(&camera_data->config);
  if (ret != ESP_OK) {
    ERROR_HARDWARE(&s_ov7670_error_handler, ret, ERROR_SEVERITY_MEDIUM, 
                  "Failed to apply camera configuration");
    camera_data->state = k_ov7670_config_error;
    return ret;
  }
  
  /* Record successful status in error handler */
  error_handler_record_status(&s_ov7670_error_handler, ESP_OK);
  
  return ESP_OK;
}

void ov7670_reset_on_error(ov7670_data_t* const camera_data)
{
  if (!camera_data) {
    log_error(ov7670_tag, "Reset Error", "Camera data pointer is NULL");
    return;
  }
  
  if (camera_data->state != k_ov7670_config_error) {
    log_warn(ov7670_tag, 
             "Reset Warning", 
             "Camera not in error state, reset not needed");
    return;
  }
  
  /* Check if we should retry based on time elapsed */
  TickType_t current_ticks = xTaskGetTickCount();
  TickType_t elapsed_ticks = current_ticks - camera_data->last_attempt_ticks;
  
  if (elapsed_ticks < camera_data->retry_interval) {
    /* Not enough time has passed since the last attempt */
    return;
  }
  
  /* Update last attempt time */
  camera_data->last_attempt_ticks = current_ticks;
  
  /* Check if we've exceeded the retry count */
  if (camera_data->retries >= ov7670_max_retries) {
    log_error(ov7670_tag, 
              "Reset Failed", 
              "Maximum retry count (%u) exceeded", 
              ov7670_max_retries);
    return;
  }
  
  log_info(ov7670_tag, 
           "Reset Attempt", 
           "Attempting to reset camera (retry %u of %u)",
           camera_data->retries + 1, 
           ov7670_max_retries);
  
  /* Increment retry counter */
  camera_data->retries++;
  
  /* Attempt to reinitialize */
  esp_err_t ret = ov7670_init(camera_data);
  if (ret == ESP_OK) {
    log_info(ov7670_tag, "Reset Success", "Camera reset successful");
    camera_data->retries = 0;
  } else {
    ERROR_HARDWARE(&s_ov7670_error_handler, ret, ERROR_SEVERITY_HIGH, 
                  "Camera reset failed");
    log_error(ov7670_tag, "Reset Failed", "Camera reset failed: %s", esp_err_to_name(ret));
  }
}

void ov7670_tasks(void* const camera_data_)
{
  ov7670_data_t* const camera_data = (ov7670_data_t*)camera_data_;
  
  if (!camera_data) {
    log_error(ov7670_tag, "Task Error", "Camera data pointer is NULL");
    return;
  }
  
  /* If camera is in error state, attempt to reset it */
  if (camera_data->state == k_ov7670_config_error) {
    ov7670_reset_on_error(camera_data);
  }
  
  /* TODO: Add other periodic tasks like checking camera status */
}

esp_err_t ov7670_cleanup(void* const camera_data)
{
  ov7670_data_t* const ov7670_data = (ov7670_data_t*)camera_data;
  if (!ov7670_data) {
    log_error(ov7670_tag, 
              "Cleanup Error", 
              "Camera data pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(ov7670_tag, "Cleanup Start", "Beginning OV7670 camera cleanup");
  esp_err_t ret = ESP_OK;

#ifdef USE_OV7670_XCLK_GPIO_27
  /* Stop LEDC output for XCLK */
  esp_err_t temp_ret = ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
  if (temp_ret != ESP_OK) {
    ERROR_HARDWARE(&s_ov7670_error_handler, temp_ret, ERROR_SEVERITY_LOW, 
                  "Failed to stop LEDC output");
    log_warn(ov7670_tag, 
             "XCLK Warning", 
             "Failed to stop LEDC output: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }

  /* Reset XCLK GPIO using common_cleanup */
  uint64_t xclk_pin_mask = (1ULL << ov7670_xclk_gpio);
  temp_ret = common_cleanup_gpio(xclk_pin_mask, ov7670_tag);
  if (temp_ret != ESP_OK) {
    ERROR_HARDWARE(&s_ov7670_error_handler, temp_ret, ERROR_SEVERITY_LOW, 
                  "Failed to reset XCLK pin configuration");
    log_warn(ov7670_tag, 
             "GPIO Warning", 
             "Failed to reset XCLK pin configuration: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }
#endif

  /* Reset I2C pins using common_cleanup */
  uint64_t i2c_pin_mask = (1ULL << ov7670_scl_io) | (1ULL << ov7670_sda_io);
  esp_err_t temp_ret = common_cleanup_gpio(i2c_pin_mask, ov7670_tag);
  if (temp_ret != ESP_OK) {
    ERROR_HARDWARE(&s_ov7670_error_handler, temp_ret, ERROR_SEVERITY_LOW, 
                  "Failed to reset I2C pin configuration");
    log_warn(ov7670_tag, 
             "GPIO Warning", 
             "Failed to reset I2C pin configuration: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }

  /* Clean up I2C resources */
  temp_ret = priv_i2c_cleanup(ov7670_i2c_bus, ov7670_tag);
  if (temp_ret != ESP_OK) {
    ERROR_HARDWARE(&s_ov7670_error_handler, temp_ret, ERROR_SEVERITY_LOW, 
                  "Failed to clean up I2C resources");
    log_warn(ov7670_tag, 
             "I2C Warning", 
             "Failed to clean up I2C resources: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }

  /* Reset camera data structure */
  if (ov7670_data != NULL) {
    ov7670_data->state              = k_ov7670_uninitialized;
    ov7670_data->retries            = 0;
    ov7670_data->retry_interval     = pdMS_TO_TICKS(15000);
    ov7670_data->last_attempt_ticks = 0;
  }
  
  /* Clean up the error handler */
  error_handler_cleanup(&s_ov7670_error_handler);
  error_handler_unregister_component("ov7670");

  if (ret == ESP_OK) {
    log_info(ov7670_tag, 
             "Cleanup Complete", 
             "OV7670 camera resources released successfully");
  } else {
    log_warn(ov7670_tag, 
             "Cleanup Warning", 
             "OV7670 cleanup completed with some warnings");
  }

  return ret;
}

