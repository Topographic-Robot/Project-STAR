/* components/sensors/qmc5883l_hal/qmc5883l_hal.c */

#include "qmc5883l_hal.h"
#include <math.h>
#include "file_write_manager.h"
#include "webserver_tasks.h"
#include "cJSON.h"
#include "common/i2c.h"
#include "error_handler.h"
#include "log_handler.h"

/* Constants ******************************************************************/

const uint8_t    qmc5883l_i2c_address            = 0x0D;
const i2c_port_t qmc5883l_i2c_bus                = I2C_NUM_0;
const char      *qmc5883l_tag                    = "QMC5883L";
const uint8_t    qmc5883l_scl_io                 = GPIO_NUM_22;
const uint8_t    qmc5883l_sda_io                 = GPIO_NUM_21;
const gpio_num_t qmc5883l_drdy_pin               = GPIO_NUM_18;
const uint32_t   qmc5883l_i2c_freq_hz            = 100000;
const uint32_t   qmc5883l_polling_rate_ticks     = pdMS_TO_TICKS(5 * 1000);
const uint8_t    qmc5883l_odr_setting            = k_qmc5883l_odr_100hz;
const uint8_t    qmc5883l_max_retries            = 4;
const uint32_t   qmc5883l_initial_retry_interval = pdMS_TO_TICKS(15);
const uint32_t   qmc5883l_max_backoff_interval   = pdMS_TO_TICKS(8 * 60);
const uint8_t    qmc5883l_mag_data_size          = 6;                     /* Size of magnetometer data in bytes (2 bytes x 3 axes) */

/* Globals (Static) ***********************************************************/

static const qmc5883l_scale_t qmc5883l_scale_configs[] = {
  {k_qmc5883l_range_2g, 200.0 / 32768.0 }, /**< ±2 Gauss range, scaling factor */
  {k_qmc5883l_range_8g, 800.0 / 32768.0 }, /**< ±8 Gauss range, scaling factor */
};

static const uint8_t   qmc5883l_scale_config_idx = 0; /**< Index of chosen values (0 for ±2G, 1 for ±8G) */
static error_handler_t s_qmc5883l_error_handler  = { 0 };

/* Private Functions **********************************************************/

static esp_err_t priv_qmc5883l_configure_drdy_pin(void)
{
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << qmc5883l_drdy_pin),
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE
  };
  return gpio_config(&io_conf);
}

static bool priv_qmc5883l_is_data_ready(void)
{
  return gpio_get_level(qmc5883l_drdy_pin) == 1;
}

/* Public Functions ***********************************************************/

char *qmc5883l_data_to_json(const qmc5883l_data_t *data)
{
  cJSON *json = cJSON_CreateObject();
  if (!json) {
    log_error(qmc5883l_tag, 
              "JSON Creation Failed", 
              "Memory allocation failed while creating JSON object");
    return NULL;
  }

  if (!cJSON_AddStringToObject(json, "sensor_type", "magnetometer")) {
    log_error(qmc5883l_tag, 
              "JSON Field Error", 
              "Failed to add sensor_type field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "mag_x", data->mag_x)) {
    log_error(qmc5883l_tag, 
              "JSON Field Error", 
              "Failed to add mag_x field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "mag_y", data->mag_y)) {
    log_error(qmc5883l_tag, 
              "JSON Field Error", 
              "Failed to add mag_y field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "mag_z", data->mag_z)) {
    log_error(qmc5883l_tag, 
              "JSON Field Error", 
              "Failed to add mag_z field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "heading", data->heading)) {
    log_error(qmc5883l_tag, 
              "JSON Field Error", 
              "Failed to add heading field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  char *json_string = cJSON_PrintUnformatted(json);
  if (!json_string) {
    log_error(qmc5883l_tag, 
              "JSON Serialization Failed", 
              "Unable to convert JSON object to string format");
    cJSON_Delete(json);
    return NULL;
  }

  cJSON_Delete(json);
  return json_string;
}

esp_err_t qmc5883l_init(void *sensor_data)
{
  qmc5883l_data_t *qmc5883l_data = (qmc5883l_data_t *)sensor_data;
  log_info(qmc5883l_tag, 
           "Init Started", 
           "Beginning QMC5883L magnetometer initialization");

  qmc5883l_data->i2c_address        = qmc5883l_i2c_address;
  qmc5883l_data->i2c_bus            = qmc5883l_i2c_bus;
  qmc5883l_data->mag_x              = 0.0;
  qmc5883l_data->mag_y              = 0.0;
  qmc5883l_data->mag_z              = 0.0;
  qmc5883l_data->state              = k_qmc5883l_uninitialized;
  qmc5883l_data->retry_count        = 0;
  qmc5883l_data->retry_interval     = qmc5883l_initial_retry_interval;
  qmc5883l_data->last_attempt_ticks = 0;

  esp_err_t ret = priv_i2c_init(qmc5883l_scl_io, 
                                qmc5883l_sda_io,
                                qmc5883l_i2c_freq_hz, 
                                qmc5883l_i2c_bus,
                                qmc5883l_tag);

  if (ret != ESP_OK) {
    log_error(qmc5883l_tag, 
              "I2C Error", 
              "Failed to install I2C driver: %s", 
              esp_err_to_name(ret));
    qmc5883l_data->state = k_qmc5883l_power_on_error;
    return ret;
  }

  ret = priv_qmc5883l_configure_drdy_pin();
  if (ret != ESP_OK) {
    log_error(qmc5883l_tag, 
              "GPIO Error", 
              "Failed to configure DRDY pin for QMC5883L");
    qmc5883l_data->state = k_qmc5883l_power_on_error;
    return ret;
  }

  uint8_t ctrl1 = k_qmc5883l_mode_continuous | 
                  qmc5883l_odr_setting |
                  qmc5883l_scale_configs[qmc5883l_scale_config_idx].range |
                  k_qmc5883l_osr_512;

  ret = priv_i2c_write_reg_byte(k_qmc5883l_ctrl1_cmd, 
                                ctrl1,
                                qmc5883l_i2c_bus, 
                                qmc5883l_i2c_address,
                                qmc5883l_tag);
  if (ret != ESP_OK) {
    log_error(qmc5883l_tag, 
              "Config Error", 
              "Failed to configure CTRL1 register with measurement settings");
    qmc5883l_data->state = k_qmc5883l_error;
    return ret;
  }

  qmc5883l_data->state = k_qmc5883l_ready;
  log_info(qmc5883l_tag, 
           "Init Complete", 
           "QMC5883L magnetometer initialization completed successfully");
  return ESP_OK;
}

esp_err_t qmc5883l_read(qmc5883l_data_t *sensor_data)
{
  if (sensor_data == NULL) {
    log_error(qmc5883l_tag, 
              "Invalid Parameter", 
              "Sensor data pointer is NULL, cannot proceed with read operation");
    return ESP_FAIL;
  }

  if (!priv_qmc5883l_is_data_ready()) {
    log_warn(qmc5883l_tag, 
             "Data Not Ready", 
             "DRDY pin indicates no new magnetometer data available");
    return ESP_ERR_NOT_FOUND;
  }

  uint8_t   mag_data[qmc5883l_mag_data_size];
  esp_err_t ret = priv_i2c_read_reg_bytes(k_qmc5883l_data_xout_l_cmd, 
                                          mag_data,
                                          qmc5883l_mag_data_size,
                                          sensor_data->i2c_bus,
                                          sensor_data->i2c_address,
                                          qmc5883l_tag);
  if (ret != ESP_OK) {
    log_error(qmc5883l_tag, 
              "Read Error", 
              "Failed to read magnetometer data from QMC5883L");
    sensor_data->state = k_qmc5883l_error;
    return ESP_FAIL;
  }

  sensor_data->state = k_qmc5883l_data_updated;

  /* Convert raw magnetometer data to 16-bit signed integers
   * Data format: Each axis uses 2 bytes in little-endian format
   * mag_data[1:0] = X-axis (high byte : low byte)
   * mag_data[3:2] = Y-axis (high byte : low byte)
   * mag_data[5:4] = Z-axis (high byte : low byte)
   * Left shift high byte by 8 bits and combine with low byte using OR
   */
  int16_t mag_x_raw = (int16_t)((mag_data[1] << 8) | mag_data[0]);
  int16_t mag_y_raw = (int16_t)((mag_data[3] << 8) | mag_data[2]);
  int16_t mag_z_raw = (int16_t)((mag_data[5] << 8) | mag_data[4]);

  float scale_factor = qmc5883l_scale_configs[qmc5883l_scale_config_idx].scale;
  sensor_data->mag_x = mag_x_raw * scale_factor;
  sensor_data->mag_y = mag_y_raw * scale_factor;
  sensor_data->mag_z = mag_z_raw * scale_factor;

  float heading = atan2(sensor_data->mag_y, sensor_data->mag_x) * 180.0 / M_PI;
  if (heading < 0) {
    heading += 360.0;
  }
  sensor_data->heading = heading;

  log_info(qmc5883l_tag, 
           "Data Updated", 
           "Magnetic field: [%f, %f, %f] µT, Heading: %.1f°",
           sensor_data->mag_x, 
           sensor_data->mag_y, 
           sensor_data->mag_z,
           sensor_data->heading);

  sensor_data->state = k_qmc5883l_data_updated;
  return ESP_OK;
}

void qmc5883l_reset_on_error(qmc5883l_data_t *sensor_data)
{
  if (sensor_data->state & k_qmc5883l_error) {
    TickType_t now_ticks = xTaskGetTickCount();
    if (now_ticks - sensor_data->last_attempt_ticks >= sensor_data->retry_interval) {
      sensor_data->last_attempt_ticks = now_ticks;

      if (qmc5883l_init(sensor_data) == ESP_OK) {
        sensor_data->state          = k_qmc5883l_ready;
        sensor_data->retry_count    = 0;
        sensor_data->retry_interval = qmc5883l_initial_retry_interval;
      } else {
        sensor_data->retry_count += 1;

        if (sensor_data->retry_count >= qmc5883l_max_retries) {
          sensor_data->retry_count    = 0;
          sensor_data->retry_interval = (sensor_data->retry_interval * 2 <= qmc5883l_max_backoff_interval) ?
                                        sensor_data->retry_interval * 2 :
                                        qmc5883l_max_backoff_interval;
        }
      }
    }
  }
}

void qmc5883l_tasks(void *sensor_data)
{
  qmc5883l_data_t *qmc5883l_data = (qmc5883l_data_t *)sensor_data;
  while (1) {
    if (qmc5883l_read(qmc5883l_data) == ESP_OK) {
      char *json = qmc5883l_data_to_json(qmc5883l_data);
      send_sensor_data_to_webserver(json);
      file_write_enqueue("qmc5883l.txt", json);
      free(json);
    } else {
      qmc5883l_reset_on_error(qmc5883l_data);
    }
    vTaskDelay(qmc5883l_polling_rate_ticks);
  }
}
