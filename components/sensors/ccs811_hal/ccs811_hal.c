/* components/sensors/ccs811_hal/ccs811_hal.c */

#include "ccs811_hal.h"
#include "file_write_manager.h"
#include "webserver_tasks.h"
#include "cJSON.h"
#include "common/i2c.h"
#include "error_handler.h"
#include "log_handler.h"

/* Constants ******************************************************************/

const uint8_t    ccs811_i2c_address            = 0x5A;
const i2c_port_t ccs811_i2c_bus                = I2C_NUM_0;
const char      *ccs811_tag                    = "CCS811";
const uint8_t    ccs811_scl_io                 = GPIO_NUM_22;
const uint8_t    ccs811_sda_io                 = GPIO_NUM_21;
const uint8_t    ccs811_wake_io                = GPIO_NUM_33;
const uint8_t    ccs811_rst_io                 = GPIO_NUM_32;
const uint8_t    ccs811_int_io                 = GPIO_NUM_25;
const uint32_t   ccs811_i2c_freq_hz            = 100000;
const uint32_t   ccs811_polling_rate_ticks     = pdMS_TO_TICKS(1 * 1000);
const uint8_t    ccs811_max_retries            = 4;
const uint32_t   ccs811_initial_retry_interval = pdMS_TO_TICKS(15 * 1000);
const uint32_t   ccs811_max_backoff_interval   = pdMS_TO_TICKS(8 * 60 * 1000);

/* Static Functions **********************************************************/

/**
 * @brief Reset function for the CCS811 sensor
 * 
 * This function is called by the error handler when a reset is needed.
 * It performs a full reinitialization of the sensor.
 * 
 * @param context Pointer to the ccs811_data_t structure
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t priv_ccs811_reset(void *context)
{
  ccs811_data_t *ccs811_data = (ccs811_data_t *)context;
  esp_err_t      ret;

  /* Reset the sensor */
  gpio_set_level(ccs811_rst_io, k_ccs811_gpio_low);
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level(ccs811_rst_io, k_ccs811_gpio_high);
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Wake up the sensor */
  gpio_set_level(ccs811_wake_io, k_ccs811_gpio_low);
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Application start */
  ret = priv_i2c_write_byte(k_ccs811_cmd_app_start, 
                            ccs811_i2c_bus, 
                            ccs811_i2c_address, 
                            ccs811_tag);
  if (ret != ESP_OK) {
    ccs811_data->state = k_ccs811_app_start_error;
    log_error(ccs811_tag, 
              "Sensor Start Error", 
              "Failed to start CCS811 application mode");
    return ret;
  }

  ccs811_data->state = k_ccs811_ready;
  return ESP_OK;
}

/* Public Functions ***********************************************************/

char *ccs811_data_to_json(const ccs811_data_t *data)
{
  cJSON *json = cJSON_CreateObject();
  if (!json) {
    log_error(ccs811_tag, 
              "JSON Error", 
              "Failed to allocate memory for JSON object");
    return NULL;
  }

  if (!cJSON_AddStringToObject(json, "sensor_type", "air_quality")) {
    log_error(ccs811_tag, 
              "JSON Error", 
              "Failed to add sensor_type field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "eCO2", data->eco2)) {
    log_error(ccs811_tag, 
              "JSON Error", 
              "Failed to add eCO2 field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "TVOC", data->tvoc)) {
    log_error(ccs811_tag, 
              "JSON Error", 
              "Failed to add TVOC field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  char *json_string = cJSON_PrintUnformatted(json);
  if (!json_string) {
    log_error(ccs811_tag, 
              "JSON Error", 
              "Failed to serialize JSON object to string");
    cJSON_Delete(json);
    return NULL;
  }

  cJSON_Delete(json);
  return json_string;
}

esp_err_t ccs811_init(void *sensor_data)
{
  ccs811_data_t *ccs811_data = (ccs811_data_t *)sensor_data;
  log_info(ccs811_tag, "Init Start", "Beginning CCS811 sensor initialization");

  /* Initialize error handler */
  error_handler_init(&(ccs811_data->error_handler), 
                     ccs811_tag,
                     ccs811_max_retries, 
                     ccs811_initial_retry_interval,
                     ccs811_max_backoff_interval, 
                     priv_ccs811_reset,
                     ccs811_data, 
                     ccs811_initial_retry_interval,
                     ccs811_max_backoff_interval);

  ccs811_data->i2c_address = ccs811_i2c_address;
  ccs811_data->i2c_bus     = ccs811_i2c_bus;
  ccs811_data->eco2        = 0;
  ccs811_data->tvoc        = 0;
  ccs811_data->state       = k_ccs811_uninitialized;

  /* Initialize the I2C bus */
  esp_err_t ret = priv_i2c_init(ccs811_scl_io, 
                                ccs811_sda_io, 
                                ccs811_i2c_freq_hz,
                                ccs811_i2c_bus, 
                                ccs811_tag);
  if (ret != ESP_OK) {
    log_error(ccs811_tag, "I2C Error", "Failed to initialize I2C driver");
    return ret;
  }

  /* Perform initial sensor setup */
  ret = priv_ccs811_reset(ccs811_data);
  if (ret != ESP_OK) {
    log_error(ccs811_tag, "Reset Error", "Failed to perform initial sensor reset");
    return ret;
  }

  log_info(ccs811_tag, "Init Complete", "CCS811 sensor initialized successfully");
  return ESP_OK;
}

esp_err_t ccs811_read(ccs811_data_t *sensor_data)
{
  uint8_t data[k_ccs811_alg_data_len];
  esp_err_t ret = priv_i2c_read_bytes(data, 
                                      k_ccs811_alg_data_len, 
                                      ccs811_i2c_bus, 
                                      ccs811_i2c_address,
                                      ccs811_tag);
  if (ret != ESP_OK) {
    log_error(ccs811_tag, "Read Error", "Failed to read sensor data via I2C");
    sensor_data->eco2  = 0;
    sensor_data->tvoc  = 0;
    sensor_data->state = k_ccs811_read_error;
    return ESP_FAIL;
  }

  sensor_data->eco2 = (data[0] << 8) | data[1];
  sensor_data->tvoc = (data[2] << 8) | data[3];
  log_info(ccs811_tag, "Data Update", 
           "New readings - eCO2: %u ppm, TVOC: %u ppb", 
           sensor_data->eco2, sensor_data->tvoc);

  sensor_data->state = k_ccs811_data_updated;
  return ESP_OK;
}

void ccs811_tasks(void *sensor_data)
{
  ccs811_data_t *ccs811_data = (ccs811_data_t *)sensor_data;
  if (!ccs811_data) {
    log_error(ccs811_tag, "Task Error", "Invalid sensor data pointer provided");
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    esp_err_t ret = ccs811_read(ccs811_data);
    if (ret == ESP_OK) {
      char *json = ccs811_data_to_json(ccs811_data);
      if (json) {
        send_sensor_data_to_webserver(json);
        file_write_enqueue("ccs811.txt", json);
        free(json);
      } else {
        log_error(ccs811_tag, "JSON Error", "Failed to convert sensor data to JSON format");
      }
    } else if (ccs811_data->state & k_ccs811_error) {
      error_handler_record_error(&(ccs811_data->error_handler), ESP_FAIL);
    }
    vTaskDelay(ccs811_polling_rate_ticks);
  }
}

