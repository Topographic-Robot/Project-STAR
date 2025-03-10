/* components/sensors/mpu6050_hal/mpu6050_hal.c */

/* TODO: The values retrieved from this sensor seems a bit sus, needs to be configured a bit better */
/* TODO: Implement the interrupt handler and use GPIO_NUM_26 */

#include "mpu6050_hal.h"
#include <math.h>
#include "file_write_manager.h"
#include "webserver_tasks.h"
#include "cJSON.h"
#include "common/i2c.h"
#include "driver/gpio.h"
#include "error_handler.h"
#include "log_handler.h"
#include "common/common_setup.h"
#include "common/common_cleanup.h"

/* Constants ******************************************************************/

const uint8_t     mpu6050_i2c_address        = 0x68;
const i2c_port_t  mpu6050_i2c_bus            = I2C_NUM_0;
const char* const mpu6050_tag                = "MPU6050";
const uint8_t     mpu6050_scl_io             = GPIO_NUM_22;
const uint8_t     mpu6050_sda_io             = GPIO_NUM_21;
const uint32_t    mpu6050_i2c_freq_hz        = 100000;
const uint32_t    mpu6050_polling_rate_ticks = pdMS_TO_TICKS(20);
const uint8_t     mpu6050_sample_rate_div    = 4;
const uint8_t     mpu6050_config_dlpf        = k_mpu6050_config_dlpf_94hz;
const uint8_t     mpu6050_int_io             = GPIO_NUM_26;
const uint8_t     mpu6050_max_retries        = 3;
const uint32_t    mpu6050_initial_retry_interval = pdMS_TO_TICKS(100);
const uint32_t    mpu6050_max_backoff_interval = pdMS_TO_TICKS(5 * 60 * 1000); /* 5 minutes */

/**
 * @brief Static constant array of accelerometer configurations and scaling factors.
 *
 * The MPU6050 accelerometer has several sensitivity options that define the maximum
 * measurable acceleration range. Each configuration has a corresponding sensitivity
 * value, given in LSB/g, which allows conversion from raw sensor output to
 * acceleration in g.
 *
 * Benefits of configuring accelerometer sensitivity:
 * - Higher sensitivity (e.g., ±2g) offers finer resolution for small movements,
 *   ideal for low-speed applications or where precision is required.
 * - Lower sensitivity (e.g., ±16g) provides a wider measurement range, suitable
 *   for detecting high-impact or fast movements.
 *
 * Sensitivity options (in LSB/g) as per the MPU6050 datasheet:
 * - ±2g: 16384 LSB/g
 * - ±4g: 8192 LSB/g
 * - ±8g: 4096 LSB/g
 * - ±16g: 2048 LSB/g
 */
static const mpu6050_accel_config_t mpu6050_accel_configs[] = {
  { k_mpu6050_accel_fs_2g,  16384.0 }, /**< Sensitivity: 16384 LSB/g */
  { k_mpu6050_accel_fs_4g,  8192.0  }, /**< Sensitivity: 8192 LSB/g */
  { k_mpu6050_accel_fs_8g,  4096.0  }, /**< Sensitivity: 4096 LSB/g */
  { k_mpu6050_accel_fs_16g, 2048.0  }, /**< Sensitivity: 2048 LSB/g */
};

/**
 * @brief Static constant array of gyroscope configurations and scaling factors.
 *
 * The MPU6050 gyroscope provides several sensitivity options that define the maximum
 * measurable rotational speed range. Each configuration has an associated sensitivity
 * value in LSB/°/s, enabling conversion from raw sensor output to angular velocity
 * in degrees per second (°/s).
 *
 * Benefits of configuring gyroscope sensitivity:
 * - Higher sensitivity (e.g., ±250°/s) allows finer resolution for slow rotations,
 *   which is ideal for applications requiring precision.
 * - Lower sensitivity (e.g., ±2000°/s) enables a wider measurement range, useful
 *   for detecting fast or high-impact rotations.
 *
 * Sensitivity options (in LSB/°/s) as per the MPU6050 datasheet:
 * - ±250°/s: 131 LSB/°/s
 * - ±500°/s: 65.5 LSB/°/s
 * - ±1000°/s: 32.8 LSB/°/s
 * - ±2000°/s: 16.4 LSB/°/s
 */
static const mpu6050_gyro_config_t mpu6050_gyro_configs[] = {
  { k_mpu6050_gyro_fs_250dps,  131.0 }, /**< Sensitivity: 131 LSB/°/s */
  { k_mpu6050_gyro_fs_500dps,  65.5  }, /**< Sensitivity: 65.5 LSB/°/s */
  { k_mpu6050_gyro_fs_1000dps, 32.8  }, /**< Sensitivity: 32.8 LSB/°/s */
  { k_mpu6050_gyro_fs_2000dps, 16.4  }, /**< Sensitivity: 16.4 LSB/°/s */
};

static const uint8_t   mpu6050_gyro_config_idx  = 1; /**< Using ±500°/s for better precision in normal use */
static const uint8_t   mpu6050_accel_config_idx = 1; /**< Using ±4g for better precision in normal use */
static error_handler_t s_mpu6050_error_handler  = { 0 };

/* Static Functions **********************************************************/

/**
 * @brief Reset function for the MPU6050 sensor
 * 
 * This function is called by the error handler when a reset is needed.
 * It performs a full reinitialization of the sensor.
 * 
 * @param context Pointer to the mpu6050_data_t structure
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t priv_mpu6050_reset(void* const context)
{
  mpu6050_data_t* mpu6050_data = (mpu6050_data_t*)context;
  esp_err_t      ret;

  /* Wake up the MPU6050 sensor */
  ret = priv_i2c_write_reg_byte(k_mpu6050_pwr_mgmt_1_cmd, 
                                k_mpu6050_power_on_cmd,
                                mpu6050_i2c_bus, 
                                mpu6050_i2c_address, 
                                mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "Power On Failed", 
              "Unable to wake up MPU6050 sensor from sleep mode");
    mpu6050_data->state = k_mpu6050_power_on_error;
    return ret;
  }

  /* Delay to allow the sensor to power on */
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Reset the MPU6050 sensor */
  ret = priv_i2c_write_reg_byte(k_mpu6050_pwr_mgmt_1_cmd, 
                                k_mpu6050_reset_cmd,
                                mpu6050_i2c_bus, 
                                mpu6050_i2c_address, 
                                mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "Reset Failed", 
              "Unable to reset MPU6050 sensor to default state");
    mpu6050_data->state = k_mpu6050_reset_error;
    return ret;
  }

  /* Delay to allow the sensor to reset */
  vTaskDelay(pdMS_TO_TICKS(100));

  /* Configure the rest of the sensor as in the init function */
  // ... (rest of reset implementation)

  mpu6050_data->state = k_mpu6050_ready;
  return ESP_OK;
}

/* Static (Private) Functions **************************************************/

/**
 * @brief Interrupt Service Routine (ISR) for handling MPU6050 data ready interrupts.
 *
 * This function is called when the MPU6050 asserts its INT pin, indicating that new data
 * is ready to be read. It gives the `data_ready_sem` semaphore to unblock the task waiting
 * to read the data.
 *
 * @param[in] arg Pointer to the `mpu6050_data_t` structure.
 *
 * @note This ISR should be kept as short as possible to avoid blocking other interrupts. 
 *       Ensure that the semaphore is properly initialized before enabling the interrupt.
 */
static void IRAM_ATTR priv_mpu6050_interrupt_handler(void* const arg)
{
  mpu6050_data_t* const sensor_data                = (mpu6050_data_t* const)arg;
  BaseType_t            higher_priority_task_woken = pdFALSE;

  /* Give the semaphore to signal that data is ready */
  xSemaphoreGiveFromISR(sensor_data->data_ready_sem, &higher_priority_task_woken);

  if (higher_priority_task_woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

/* Public Functions ***********************************************************/

char* mpu6050_data_to_json(const mpu6050_data_t* const data)
{
  cJSON* const json = cJSON_CreateObject();
  if (!json) {
    log_error(mpu6050_tag, 
              "JSON Creation Failed", 
              "Memory allocation failed while creating JSON object");
    return NULL;
  }

  if (!cJSON_AddStringToObject(json, "sensor_type", "accelerometer_gyroscope")) {
    log_error(mpu6050_tag, 
              "JSON Field Error", 
              "Failed to add sensor_type field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "accel_x", data->accel_x)) {
    log_error(mpu6050_tag, 
              "JSON Field Error", 
              "Failed to add accel_x field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "accel_y", data->accel_y)) {
    log_error(mpu6050_tag, 
              "JSON Field Error", 
              "Failed to add accel_y field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "accel_z", data->accel_z)) {
    log_error(mpu6050_tag, 
              "JSON Field Error", 
              "Failed to add accel_z field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "gyro_x", data->gyro_x)) {
    log_error(mpu6050_tag, 
              "JSON Field Error", 
              "Failed to add gyro_x field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "gyro_y", data->gyro_y)) {
    log_error(mpu6050_tag, 
              "JSON Field Error", 
              "Failed to add gyro_y field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  if (!cJSON_AddNumberToObject(json, "gyro_z", data->gyro_z)) {
    log_error(mpu6050_tag, 
              "JSON Field Error", 
              "Failed to add gyro_z field to JSON object");
    cJSON_Delete(json);
    return NULL;
  }

  char* const json_string = cJSON_PrintUnformatted(json);
  if (!json_string) {
    log_error(mpu6050_tag, 
              "JSON Serialization Failed", 
              "Unable to convert JSON object to string format");
    cJSON_Delete(json);
    return NULL;
  }

  cJSON_Delete(json);
  return json_string;
}

esp_err_t mpu6050_init(void* const sensor_data)
{
  mpu6050_data_t* const mpu6050_data = (mpu6050_data_t* const)sensor_data;
  log_info(mpu6050_tag, 
           "Init Started", 
           "Beginning MPU6050 sensor initialization");

  /* Initialize error handler */
  error_handler_init(&(mpu6050_data->error_handler), 
                     mpu6050_tag,
                     mpu6050_max_retries, 
                     mpu6050_initial_retry_interval,
                     mpu6050_max_backoff_interval, 
                     priv_mpu6050_reset,
                     mpu6050_data, 
                     mpu6050_initial_retry_interval,
                     mpu6050_max_backoff_interval);

  mpu6050_data->i2c_address = mpu6050_i2c_address;
  mpu6050_data->i2c_bus     = mpu6050_i2c_bus;
  mpu6050_data->gyro_x      = mpu6050_data->gyro_y  = mpu6050_data->gyro_z  = 0.0f;
  mpu6050_data->accel_x     = mpu6050_data->accel_y = mpu6050_data->accel_z = 0.0f;
  mpu6050_data->state       = k_mpu6050_uninitialized; /* Start in uninitialized state */

  /* Initialize the I2C bus using common setup */
  esp_err_t ret = common_setup_i2c(mpu6050_i2c_bus, 
                                  mpu6050_scl_io, 
                                  mpu6050_sda_io, 
                                  mpu6050_i2c_freq_hz, 
                                  mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "I2C Error", 
              "Failed to initialize I2C driver: %s", 
              esp_err_to_name(ret));
    return ret;
  }

  /* Wake up the MPU6050 sensor */
  ret = priv_i2c_write_reg_byte(k_mpu6050_pwr_mgmt_1_cmd, 
                                k_mpu6050_power_on_cmd,
                                mpu6050_i2c_bus, 
                                mpu6050_i2c_address, 
                                mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "Power On Failed", 
              "Unable to wake up MPU6050 sensor from sleep mode");
    mpu6050_data->state = k_mpu6050_power_on_error;
    return ret;
  }

  /* Delay to allow the sensor to power on */
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Reset the MPU6050 sensor */
  ret = priv_i2c_write_reg_byte(k_mpu6050_pwr_mgmt_1_cmd, 
                                k_mpu6050_reset_cmd,
                                mpu6050_i2c_bus, 
                                mpu6050_i2c_address, 
                                mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "Reset Failed", 
              "Unable to reset MPU6050 sensor to default state");
    mpu6050_data->state = k_mpu6050_reset_error;
    return ret;
  }

  /* Delay to allow the reset to take effect */
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Wake up the sensor again after reset */
  ret = priv_i2c_write_reg_byte(k_mpu6050_pwr_mgmt_1_cmd, 
                                k_mpu6050_power_on_cmd,
                                mpu6050_i2c_bus, 
                                mpu6050_i2c_address, 
                                mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "Power On Failed", 
              "Unable to wake up MPU6050 sensor after reset");
    mpu6050_data->state = k_mpu6050_power_on_error;
    return ret;
  }

  /* Delay to allow the sensor to wake up */
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Configure the sample rate divider */
  ret = priv_i2c_write_reg_byte(k_mpu6050_smplrt_div_cmd, 
                                mpu6050_sample_rate_div,
                                mpu6050_i2c_bus, 
                                mpu6050_i2c_address, 
                                mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "Config Error", 
              "Failed to set sample rate divider for MPU6050");
    return ret;
  }

  /* Configure the Digital Low Pass Filter (DLPF) */
  ret = priv_i2c_write_reg_byte(k_mpu6050_config_cmd, 
                                mpu6050_config_dlpf,
                                mpu6050_i2c_bus, 
                                mpu6050_i2c_address, 
                                mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "Config Error", 
              "Failed to configure digital low pass filter settings");
    return ret;
  }

  /* Configure the gyroscope full-scale range */
  ret = priv_i2c_write_reg_byte(k_mpu6050_gyro_config_cmd,
                                mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_config,
                                mpu6050_i2c_bus, 
                                mpu6050_i2c_address, 
                                mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "Config Error", 
              "Failed to set gyroscope full-scale range");
    return ret;
  }

  /* Configure the accelerometer full-scale range */
  ret = priv_i2c_write_reg_byte(k_mpu6050_accel_config_cmd,
                                mpu6050_accel_configs[mpu6050_accel_config_idx].accel_config,
                                mpu6050_i2c_bus, 
                                mpu6050_i2c_address, 
                                mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "Config Error", 
              "Failed to set accelerometer full-scale range");
    return ret;
  }

  /* Verify the sensor by reading the WHO_AM_I register */
  uint8_t who_am_i = 0;
  ret              = priv_i2c_read_reg_bytes(k_mpu6050_who_am_i_cmd, 
                                             &who_am_i, 
                                             1, 
                                             mpu6050_i2c_bus, 
                                             mpu6050_i2c_address, 
                                             mpu6050_tag);
  if (ret != ESP_OK || who_am_i != k_mpu6050_who_am_i_response) {
    log_error(mpu6050_tag, 
              "Verification Failed", 
              "Invalid WHO_AM_I register response from MPU6050: 0x%02X", 
              who_am_i);
    return ret;
  }

  /* Create a binary semaphore for data readiness */
  mpu6050_data->data_ready_sem = xSemaphoreCreateBinary();
  if (mpu6050_data->data_ready_sem == NULL) {
    log_error(mpu6050_tag, 
              "Memory Error", 
              "Failed to allocate memory for data ready semaphore");
    return ESP_FAIL;
  }

  /* Configure the MPU6050 to generate Data Ready interrupts */
  ret = priv_i2c_write_reg_byte(k_mpu6050_int_enable_cmd, 
                                k_mpu6050_int_enable_data_rdy,
                                mpu6050_i2c_bus, 
                                mpu6050_i2c_address, 
                                mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "Interrupt Error", 
              "Failed to enable data ready interrupt on MPU6050");
    return ret;
  }

  /* Configure the INT (interrupt) pin on the ESP32 */
  gpio_config_t io_conf = {
    .intr_type     = GPIO_INTR_NEGEDGE, /* MPU6050 INT pin is active low */
    .pin_bit_mask  = (1ULL << mpu6050_int_io),
    .mode          = GPIO_MODE_INPUT,
    .pull_up_en    = GPIO_PULLUP_DISABLE,
    .pull_down_en  = GPIO_PULLDOWN_ENABLE
  };
  ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "GPIO Error", 
              "Failed to configure interrupt pin for MPU6050");
    return ret;
  }

  /* Install GPIO ISR service */
  ret = gpio_install_isr_service(0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    log_error(mpu6050_tag, 
              "ISR Error", 
              "Failed to install GPIO interrupt service routine");
    return ret;
  }

  /* Add ISR handler */
  ret = gpio_isr_handler_add(mpu6050_int_io, 
                             priv_mpu6050_interrupt_handler, 
                             (void*) mpu6050_data);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "ISR Error", 
              "Failed to register interrupt handler for MPU6050");
    return ret;
  }

  mpu6050_data->state = k_mpu6050_ready; /* Sensor is initialized */
  log_info(mpu6050_tag, 
           "Init Complete", 
           "MPU6050 sensor initialization completed successfully");
  return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_data_t* const sensor_data)
{
  if (sensor_data == NULL) {
    log_error(mpu6050_tag, "Read Error", "Invalid sensor data pointer");
    return ESP_ERR_INVALID_ARG;
  }

  /* Read accelerometer data */
  uint8_t accel_data[MPU6050_ACCEL_DATA_SIZE];
  esp_err_t ret = priv_i2c_read_reg_bytes(k_mpu6050_accel_xout_h_cmd, 
                                          accel_data, 
                                          MPU6050_ACCEL_DATA_SIZE,
                                          mpu6050_i2c_bus, 
                                          mpu6050_i2c_address, 
                                          mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "Accel Read Error", 
              "Failed to read accelerometer data: %s", 
              esp_err_to_name(ret));
    sensor_data->state = k_mpu6050_error;
    ERROR_COMMUNICATION(&(sensor_data->error_handler), ret, ERROR_SEVERITY_MEDIUM, 
                        "Failed to read accelerometer data");
    return ret;
  }

  /* Read gyroscope data */
  uint8_t gyro_data[MPU6050_GYRO_DATA_SIZE];
  ret = priv_i2c_read_reg_bytes(k_mpu6050_gyro_xout_h_cmd, 
                               gyro_data, 
                               MPU6050_GYRO_DATA_SIZE,
                               mpu6050_i2c_bus, 
                               mpu6050_i2c_address, 
                               mpu6050_tag);
  if (ret != ESP_OK) {
    log_error(mpu6050_tag, 
              "Gyro Read Error", 
              "Failed to read gyroscope data: %s", 
              esp_err_to_name(ret));
    sensor_data->state = k_mpu6050_error;
    ERROR_COMMUNICATION(&(sensor_data->error_handler), ret, ERROR_SEVERITY_MEDIUM, 
                        "Failed to read gyroscope data");
    return ret;
  }

  /* Convert raw accelerometer data to g */
  int16_t accel_x_raw = (int16_t)((accel_data[0] << 8) | accel_data[1]);
  int16_t accel_y_raw = (int16_t)((accel_data[2] << 8) | accel_data[3]);
  int16_t accel_z_raw = (int16_t)((accel_data[4] << 8) | accel_data[5]);
  
  sensor_data->accel_x = (float)accel_x_raw / mpu6050_accel_configs[mpu6050_accel_config_idx].accel_scale;
  sensor_data->accel_y = (float)accel_y_raw / mpu6050_accel_configs[mpu6050_accel_config_idx].accel_scale;
  sensor_data->accel_z = (float)accel_z_raw / mpu6050_accel_configs[mpu6050_accel_config_idx].accel_scale;

  /* Convert raw gyroscope data to degrees per second */
  int16_t gyro_x_raw = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
  int16_t gyro_y_raw = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
  int16_t gyro_z_raw = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);
  
  sensor_data->gyro_x = (float)gyro_x_raw / mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_scale;
  sensor_data->gyro_y = (float)gyro_y_raw / mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_scale;
  sensor_data->gyro_z = (float)gyro_z_raw / mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_scale;

  sensor_data->state = k_mpu6050_data_updated;
  
  /* Record successful status in error handler */
  error_handler_record_status(&(sensor_data->error_handler), ESP_OK);
  
  return ESP_OK;
}

void mpu6050_reset_on_error(mpu6050_data_t* const sensor_data)
{
  /* Check if the state indicates any error */
  if (sensor_data->state & k_mpu6050_error) {
    /* TODO: Reset error handler */
  }
}

void mpu6050_tasks(void* const sensor_data)
{
  mpu6050_data_t* const mpu6050_data = (mpu6050_data_t* const)sensor_data;
  while (1) {
    if (mpu6050_read(mpu6050_data) == ESP_OK) {
      char* const json = mpu6050_data_to_json(mpu6050_data);
      send_sensor_data_to_webserver(json);
      file_write_enqueue("mpu6050.txt", json);
      free(json);
    } else {
      mpu6050_reset_on_error(mpu6050_data);
    }
    vTaskDelay(mpu6050_polling_rate_ticks);
  }
}

esp_err_t mpu6050_cleanup(void* const sensor_data)
{
  mpu6050_data_t* const mpu6050_data = (mpu6050_data_t*)sensor_data;
  log_info(mpu6050_tag, "Cleanup Start", "Beginning MPU6050 sensor cleanup");

  esp_err_t ret = ESP_OK;

  /* Disable data ready interrupt on MPU6050 */
  esp_err_t temp_ret = priv_i2c_write_reg_byte(k_mpu6050_int_enable_cmd, 
                                               0x00, /* Disable all interrupts */
                                               mpu6050_i2c_bus, 
                                               mpu6050_i2c_address, 
                                               mpu6050_tag);
  if (temp_ret != ESP_OK) {
    log_warn(mpu6050_tag, 
             "Interrupt Warning", 
             "Failed to disable MPU6050 interrupts: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }

  /* Reset GPIO interrupt pin to input mode with no pull-up/down */
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << mpu6050_int_io),
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE
  };

  temp_ret = gpio_config(&io_conf);
  if (temp_ret != ESP_OK) {
    log_warn(mpu6050_tag, 
             "GPIO Warning", 
             "Failed to reset GPIO pin configuration: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }

  /* Remove GPIO interrupt handler */
  temp_ret = gpio_isr_handler_remove(mpu6050_int_io);
  if (temp_ret != ESP_OK) {
    log_warn(mpu6050_tag, 
             "ISR Warning", 
             "Failed to remove GPIO interrupt handler: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }

  /* Put MPU6050 in sleep mode to save power */
  temp_ret = priv_i2c_write_reg_byte(k_mpu6050_pwr_mgmt_1_cmd, 
                                     k_mpu6050_power_down_cmd,
                                     mpu6050_i2c_bus, 
                                     mpu6050_i2c_address, 
                                     mpu6050_tag);
  if (temp_ret != ESP_OK) {
    log_warn(mpu6050_tag, 
             "Power Warning", 
             "Failed to put MPU6050 in sleep mode: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }

  /* Delete the data ready semaphore if it exists */
  if (mpu6050_data && mpu6050_data->data_ready_sem) {
    vSemaphoreDelete(mpu6050_data->data_ready_sem);
    mpu6050_data->data_ready_sem = NULL;
  }

  /* Clean up error handler */
  if (mpu6050_data) {
    temp_ret = error_handler_cleanup(&(mpu6050_data->error_handler));
    if (temp_ret != ESP_OK) {
      log_warn(mpu6050_tag, 
               "Error Handler Warning", 
               "Failed to clean up error handler: %s", 
               esp_err_to_name(temp_ret));
      ret = temp_ret;
    }
  }

  /* Clean up I2C resources using common cleanup */
  temp_ret = common_cleanup_i2c(mpu6050_i2c_bus, mpu6050_scl_io, mpu6050_sda_io, mpu6050_tag);
  if (temp_ret != ESP_OK) {
    log_warn(mpu6050_tag, 
             "I2C Warning", 
             "Failed to clean up I2C resources: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }

  /* Reset sensor data structure */
  if (mpu6050_data) {
    mpu6050_data->gyro_x  = mpu6050_data->gyro_y  = mpu6050_data->gyro_z  = 0.0f;
    mpu6050_data->accel_x = mpu6050_data->accel_y = mpu6050_data->accel_z = 0.0f;
    mpu6050_data->state   = k_mpu6050_uninitialized;
  }

  log_info(mpu6050_tag, 
           "Cleanup Complete", 
           "MPU6050 sensor cleanup %s", 
           (ret == ESP_OK) ? "successful" : "completed with warnings");

  return ret;
}
