/* components/sensors/mq135_hal/mq135_hal.c */

#include "mq135_hal.h"
#include <math.h>
#include "file_write_manager.h"
#include "webserver_tasks.h"
#include "cJSON.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "error_handler.h"
#include "log_handler.h"

/* Constants *******************************************************************/

const char* const mq135_tag                    = "MQ135";
const uint8_t     mq135_aout_pin               = GPIO_NUM_34;
const uint8_t     mq135_dout_pin               = GPIO_NUM_35;
const uint32_t    mq135_polling_rate_ticks     = pdMS_TO_TICKS(1000);
const uint32_t    mq135_warmup_time_ms         = 180000; /**< 3-minute warm-up time */
const uint8_t     mq135_max_retries            = 4;
const uint32_t    mq135_initial_retry_interval = pdMS_TO_TICKS(15000);
const uint32_t    mq135_max_backoff_interval   = pdMS_TO_TICKS(480000);

/* Globals (Static) ***********************************************************/

static adc_oneshot_unit_handle_t s_adc1_handle         = { 0 }; /**< ADC handle for the MQ135 sensor. */
static error_handler_t           s_mq135_error_handler = { 0 };

/* Static (Private) Functions **************************************************/

/**
 * @brief Calculates gas concentration in ppm from raw ADC value.
 *
 * Converts the raw ADC reading from the MQ135 sensor to gas concentration in
 * parts per million (ppm) using an approximation formula.
 *
 * @param[in] raw_adc_value Raw ADC reading from the sensor.
 *
 * @return Gas concentration in ppm.
 *
 * @note The formula used is an approximation and may require calibration for
 *       accurate measurements.
 */
static float priv_mq135_calculate_ppm(int raw_adc_value)
{
  float resistance = (4095.0 / raw_adc_value - 1.0) * 10.0;              /* Assuming RL = 10k */
  float ppm        = 116.6020682 * pow(resistance / 10.0, -2.769034857); /* Example curve from datasheet */
  return ppm;
}

/* Public Functions ***********************************************************/

char* mq135_data_to_json(const mq135_data_t* const data)
{
  cJSON* json = cJSON_CreateObject();
  if (!json) {
    log_error(mq135_tag, 
              "JSON Error", 
              "Failed to allocate memory for JSON object");
    return NULL;
  }

  if (!cJSON_AddStringToObject(json, "sensor_type", "gas")) {
    log_error(mq135_tag, 
              "JSON Error", 
              "Failed to add sensor_type field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "gas_concentration", data->gas_concentration)) {
    log_error(mq135_tag, 
              "JSON Error", 
              "Failed to add gas_concentration field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  char *json_string = cJSON_PrintUnformatted(json);
  if (!json_string) {
    log_error(mq135_tag, 
              "JSON Error", 
              "Failed to serialize JSON object to string");
    cJSON_Delete(json);
    return NULL;
  }

  cJSON_Delete(json);
  return json_string;
}

esp_err_t mq135_init(void* const sensor_data)
{
  mq135_data_t* const mq135_data = (mq135_data_t*)sensor_data;
  log_info(mq135_tag, "Init Start", "Beginning MQ135 gas sensor initialization");

  mq135_data->raw_adc_value      = 0;
  mq135_data->gas_concentration  = 0.0;
  mq135_data->state              = k_mq135_warming_up;
  mq135_data->warmup_start_ticks = xTaskGetTickCount();

  /* Initialize error handler */
  mq135_data->error_handler = malloc(sizeof(error_handler_t));
  if (mq135_data->error_handler == NULL) {
    log_error(mq135_tag, "Memory Error", "Failed to allocate memory for error handler");
    return ESP_ERR_NO_MEM;
  }

  error_handler_config_t error_config = {
    .max_retries = mq135_max_retries,
    .initial_retry_interval = mq135_initial_retry_interval,
    .max_backoff_interval = mq135_max_backoff_interval,
    .reset_func = NULL, /* No reset function for now */
    .reset_arg = sensor_data
  };
  esp_err_t ret = error_handler_init(mq135_data->error_handler,
                                    mq135_tag,
                                    mq135_max_retries,
                                    mq135_initial_retry_interval,
                                    mq135_max_backoff_interval,
                                    NULL, /* No reset function for now */
                                    sensor_data,
                                    mq135_initial_retry_interval,
                                    mq135_max_backoff_interval);
  if (ret != ESP_OK) {
    log_error(mq135_tag, "Error Handler Init Failed", 
              "Failed to initialize error handler: %s", esp_err_to_name(ret));
    free(mq135_data->error_handler);
    mq135_data->error_handler = NULL;
    return ret;
  }

  /* Setup GPIO pins using common setup */
  ret = common_setup_gpio((1ULL << mq135_aout_pin) | (1ULL << mq135_dout_pin),
                         GPIO_MODE_INPUT,
                         GPIO_PULLUP_DISABLE,
                         GPIO_PULLDOWN_DISABLE,
                         GPIO_INTR_DISABLE,
                         mq135_tag);
  if (ret != ESP_OK) {
    log_error(mq135_tag, "GPIO Setup Failed", 
              "Failed to configure GPIO pins: %s", esp_err_to_name(ret));
    error_handler_cleanup(mq135_data->error_handler);
    free(mq135_data->error_handler);
    mq135_data->error_handler = NULL;
    return ret;
  }

  /* Setup ADC */
  adc_oneshot_unit_init_cfg_t adc1_init_cfg = { .unit_id = ADC_UNIT_1 };
  ret = adc_oneshot_new_unit(&adc1_init_cfg, &s_adc1_handle);
  if (ret != ESP_OK) {
    log_error(mq135_tag, 
              "ADC Error", 
              "Failed to initialize ADC unit for gas sensor");
    common_cleanup_gpio((1ULL << mq135_aout_pin) | (1ULL << mq135_dout_pin), mq135_tag);
    error_handler_cleanup(mq135_data->error_handler);
    free(mq135_data->error_handler);
    mq135_data->error_handler = NULL;
    return ret;
  }

  adc_oneshot_chan_cfg_t chan_cfg = {
    .atten    = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ret = adc_oneshot_config_channel(s_adc1_handle, ADC_CHANNEL_6, &chan_cfg);
  if (ret != ESP_OK) {
    log_error(mq135_tag, 
              "ADC Error", 
              "Failed to configure ADC channel for gas sensor");
    adc_oneshot_del_unit(s_adc1_handle);
    common_cleanup_gpio((1ULL << mq135_aout_pin) | (1ULL << mq135_dout_pin), mq135_tag);
    error_handler_cleanup(mq135_data->error_handler);
    free(mq135_data->error_handler);
    mq135_data->error_handler = NULL;
    return ret;
  }

  log_info(mq135_tag, 
           "Init Complete", 
           "MQ135 gas sensor initialized successfully");
  return ESP_OK;
}

esp_err_t mq135_read(mq135_data_t* const sensor_data)
{
  mq135_data_t* const mq135_data = (mq135_data_t*)sensor_data;

  TickType_t now_ticks = xTaskGetTickCount();
  if (now_ticks - mq135_data->warmup_start_ticks < pdMS_TO_TICKS(mq135_warmup_time_ms)) {
    mq135_data->state = k_mq135_warming_up;
    log_warn(mq135_tag, "Warmup Status", "Gas sensor still in warm-up period");
    return ESP_FAIL;
  }

  int       raw_adc;
  esp_err_t ret = adc_oneshot_read(s_adc1_handle, ADC_CHANNEL_6, &raw_adc);
  if (ret != ESP_OK) {
    mq135_data->state = k_mq135_read_error;
    log_error(mq135_tag, 
              "Read Error", 
              "Failed to read ADC value from gas sensor");
    return ESP_FAIL;
  }

  mq135_data->raw_adc_value     = raw_adc;
  mq135_data->gas_concentration = priv_mq135_calculate_ppm(raw_adc);

  log_info(mq135_tag, "Data Update", 
           "New reading - Raw ADC: %u, Gas Concentration: %.2f ppm", 
           raw_adc, mq135_data->gas_concentration);

  mq135_data->state = k_mq135_ready;
  return ESP_OK;
}

void mq135_reset_on_error(mq135_data_t* const sensor_data)
{
  if (sensor_data->error_handler) {
    esp_err_t ret = error_handler_reset(sensor_data->error_handler);
    if (ret == ESP_OK) {
      log_info(mq135_tag, "Reset Success", "Successfully reset MQ135 gas sensor");
      sensor_data->state = k_mq135_ready;
    } else {
      log_error(mq135_tag, "Reset Failed", 
                "Failed to reset MQ135 gas sensor: %s", esp_err_to_name(ret));
      sensor_data->state = k_mq135_error;
    }
  } else {
    log_error(mq135_tag, "Error Handler Missing", 
              "Cannot reset MQ135 gas sensor: error handler not initialized");
    sensor_data->state = k_mq135_error;
  }
}

void mq135_tasks(void* const sensor_data)
{
  mq135_data_t* const mq135_data = (mq135_data_t*)sensor_data;
  if (!mq135_data) {
    log_error(mq135_tag, "Task Error", "Invalid sensor data pointer provided");
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    esp_err_t ret = mq135_read(mq135_data);
    if (ret == ESP_OK) {
      char* json = mq135_data_to_json(mq135_data);
      if (json) {
        send_sensor_data_to_webserver(json);
        file_write_enqueue("mq135.txt", json);
        free(json);
      } else {
        log_error(mq135_tag, 
                  "JSON Error", 
                  "Failed to convert gas sensor data to JSON format");
      }
    } else {
      /* Record error in error handler */
      if (mq135_data->error_handler) {
        ERROR_HARDWARE(mq135_data->error_handler, ret, k_error_severity_medium, 
                      "Failed to read MQ135 gas sensor data");
      }
      mq135_reset_on_error(mq135_data);
    }

    vTaskDelay(mq135_polling_rate_ticks);
  }
}

esp_err_t mq135_cleanup(void* const sensor_data)
{
  mq135_data_t* const mq135_data = (mq135_data_t*)sensor_data;
  log_info(mq135_tag, "Cleanup Start", "Beginning MQ135 gas sensor cleanup");

  esp_err_t ret = ESP_OK;

  /* Clean up GPIO pins using common cleanup */
  esp_err_t temp_ret = common_cleanup_gpio((1ULL << mq135_aout_pin) | (1ULL << mq135_dout_pin), mq135_tag);
  if (temp_ret != ESP_OK) {
    log_warn(mq135_tag, 
             "GPIO Warning", 
             "Failed to reset GPIO pin configuration: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }

  /* Delete ADC unit */
  if (s_adc1_handle) {
    temp_ret = adc_oneshot_del_unit(s_adc1_handle);
    if (temp_ret != ESP_OK) {
      log_warn(mq135_tag, 
               "ADC Warning", 
               "Failed to delete ADC unit: %s", 
               esp_err_to_name(temp_ret));
      ret = temp_ret;
    }
  }

  /* Clean up error handler */
  if (mq135_data->error_handler) {
    temp_ret = error_handler_cleanup(mq135_data->error_handler);
    if (temp_ret != ESP_OK) {
      log_warn(mq135_tag, 
               "Error Handler Warning", 
               "Failed to clean up error handler: %s", 
               esp_err_to_name(temp_ret));
      ret = temp_ret;
    }
    free(mq135_data->error_handler);
    mq135_data->error_handler = NULL;
  }

  log_info(mq135_tag, "Cleanup Complete", "MQ135 gas sensor cleanup completed");
  return ret;
}

