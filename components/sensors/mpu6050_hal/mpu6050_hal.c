#include "sensor_hal.h"
#include "common/i2c.h"
#include <esp_log.h>

/* TODO: interrupt, FIFO, PWR_MGMT_2(remove axises we don't need to safe power */

/* Constants ******************************************************************/

/**
 * @brief Configure the sample rate divider for the MPU6050 sensor.
 *
 * Why is SMPLRT_DIV Important?
 * The MPU6050 can internally sample data at a default rate of 1 kHz. However, 
 * depending on your application, you might not need such frequent data updates, 
 * and processing all the data can create unnecessary load on the microcontroller.
 * 
 * Benefits of adjusting the sample rate divider:
 * - Reducing processing load: Lower sample rates mean less data to process, 
 *   reducing the burden on the microcontroller's CPU and memory.
 * - Power saving: Lower data processing requirements can lead to lower overall 
 *   system power consumption, which is important for battery-powered systems.
 * - Matching system requirements: Control systems, such as robotic platforms or 
 *   IMUs, may only require data updates at 50 Hz or 100 Hz, making higher data 
 *   rates unnecessary.
 *
 * Set up the sample rate divider to 100 Hz.
 * With the MPU6050's default sample rate of 1 kHz for the gyro, 
 * setting the divider to 9 results in a 100 Hz sample rate.
 *
 * Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV)
 */
const uint8_t  mpu6050_sample_rate_div    = 9;
const char    *mpu6050_tag                = "MPU6050";
const uint8_t  mpu6050_scl_io             = GPIO_NUM_22;
const uint8_t  mpu6050_sda_io             = GPIO_NUM_21;
const uint32_t mpu6050_i2c_freq_hz        = 100000;
const uint32_t mpu6050_polling_rate_ticks = pdMS_TO_TICKS(5 * 1000);
const uint8_t  mpu6050_i2c_address        = 0x68;
const uint8_t  mpu6050_i2c_bus            = I2C_NUM_0;

/* Static const array of accelerometer configurations and scaling factors */
/* Note: (2^16)/2 = 32768 */
static const mpu6050_accel_config_t mpu6050_accel_configs[] = {
  {k_mpu6050_accel_fs_2g,  32768.0 / 2.0},  /**< ±2g range, scaling factor = 32768 / 2 = 16384 LSB/g */
  {k_mpu6050_accel_fs_4g,  32768.0 / 4.0},  /**< ±4g range, scaling factor = 32768 / 4 = 8192 LSB/g */
  {k_mpu6050_accel_fs_8g,  32768.0 / 8.0},  /**< ±8g range, scaling factor = 32768 / 8 = 4096 LSB/g */
  {k_mpu6050_accel_fs_16g, 32768.0 / 16.0}, /**< ±16g range, scaling factor = 32768 / 16 = 2048 LSB/g */
};

/* Static const array of gyroscope configurations and scaling factors */
/* Note: (2^16)/2 = 32768 */
static const mpu6050_gyro_config_t mpu6050_gyro_configs[] = {
  {k_mpu6050_gyro_fs_250dps,  32768.0 / 250.0},  /**< ±250°/s, scaling factor = 32768 / 250 = 131.072 LSB/°/s */
  {k_mpu6050_gyro_fs_500dps,  32768.0 / 500.0},  /**< ±500°/s, scaling factor = 32768 / 500 = 65.536 LSB/°/s */
  {k_mpu6050_gyro_fs_1000dps, 32768.0 / 1000.0}, /**< ±1000°/s, scaling factor = 32768 / 1000 = 32.768 LSB/°/s */
  {k_mpu6050_gyro_fs_2000dps, 32768.0 / 2000.0}, /**< ±2000°/s, scaling factor = 32768 / 2000 = 16.384 LSB/°/s */
};

static const uint8_t mpu6050_gyro_config_idx  = 1; /* Index of chosen values from above (0: ±250°/s, 1: ±500°/s, etc.) */
static const uint8_t mpu6050_accel_config_idx = 1; /* Index of chosen values from above (0: ±2g, 1: ±4g, etc.) */

/* Public Functions ***********************************************************/

esp_err_t mpu6050_init(void *sensor_data)
{
  sensor_data_t  *all_sensor_data = (sensor_data_t *)sensor_data;
  mpu6050_data_t *mpu6050_data    = &all_sensor_data->mpu6050_data;
  ESP_LOGI(mpu6050_tag, "Starting Configuration");

  mpu6050_data->i2c_address = mpu6050_i2c_address;
  mpu6050_data->i2c_bus     = mpu6050_i2c_bus;
  mpu6050_data->gyro_x      = mpu6050_data->gyro_y  = mpu6050_data->gyro_z  = -1.0;
  mpu6050_data->accel_x     = mpu6050_data->accel_y = mpu6050_data->accel_z = -1.0;
  mpu6050_data->state       = k_mpu6050_uninitialized; /* Start in uninitialized state */

  esp_err_t ret = priv_i2c_init(mpu6050_scl_io, mpu6050_sda_io, 
                                mpu6050_i2c_freq_hz, mpu6050_i2c_bus,
                                mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "I2C driver install failed: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Power on the MPU6050 sensor */
  ret = priv_i2c_write_byte(k_mpu6050_power_on_cmd, mpu6050_i2c_bus, 
                            mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "MPU6050 power on failed");
    mpu6050_data->state = k_mpu6050_power_on_error;
    return ret;
  }

  /* Delay to allow the sensor to power on */
  vTaskDelay(10 / portTICK_PERIOD_MS);

  /* Reset the MPU6050 sensor */
  ret = priv_i2c_write_byte(k_mpu6050_reset_cmd, mpu6050_i2c_bus, 
                            mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "MPU6050 reset failed");
    mpu6050_data->state = k_mpu6050_reset_error;
    return ret;
  }

  /* Delay to allow the reset to take effect */
  vTaskDelay(10 / portTICK_PERIOD_MS);

  /* Configure the sample rate divider to 100 Hz (if div is 9) */
  ret = priv_i2c_write_byte(mpu6050_sample_rate_div, mpu6050_i2c_bus, 
                            mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "MPU6050 sample rate configuration failed");
    return ret;
  }

  /* Configure the Digital Low Pass Filter (DLPF). */
  /**
   * @brief Configure the Digital Low Pass Filter (DLPF) for noise reduction.
   *
   * Why is the DLPF important?
   * The DLPF helps to filter out high-frequency noise in the sensor data, which 
   * is especially useful when dealing with vibration from mechanical systems such 
   * as robots. A lower bandwidth setting allows you to smooth out the data, 
   * removing jitter and unwanted noise.
   *
   * For example, setting the DLPF to 44 Hz bandwidth:
   * - Noise reduction: Smooths out data by filtering high-frequency noise.
   * - Vibration handling: Especially useful for systems like hexapods, where 
   *   mechanical movement introduces noise.
   *
   * In this case, DLPF_CFG = 3 provides a 44 Hz bandwidth.
   */
  ret = priv_i2c_write_byte(k_mpu6050_config_dlpf_44hz, mpu6050_i2c_bus, 
                            mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "MPU6050 DLPF configuration failed");
    return ret;
  }

  /* Configure the gyroscope full-scale range to ±500°/s. */
  /**
   * @brief Configure the full-scale range for the gyroscope.
   *
   * Why is Gyroscope Range Important?
   * The gyroscope range determines the maximum angular velocity the sensor can 
   * measure before the data saturates. A smaller range (e.g., ±250°/s) provides 
   * more sensitivity but can saturate if the object is rotating quickly.
   * Conversely, a larger range (e.g., ±2000°/s) allows the sensor to measure higher 
   * rotational speeds but at the cost of reduced sensitivity.
   * 
   * - ±500°/s: A good balance for general-purpose robotics applications, offering 
   *   sufficient sensitivity while being able to handle moderate rotational speeds 
   *   without saturating.
   */
  ret = priv_i2c_write_byte(mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_config, 
                            mpu6050_i2c_bus, mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "MPU6050 gyroscope configuration failed");
    return ret;
  }

  /* Configure the accelerometer full-scale range to ±4g. */
  /**
   * @brief Configure the full-scale range for the accelerometer.
   *
   * Why is Accelerometer Range Important?
   * The accelerometer range defines the maximum measurable acceleration. A smaller 
   * range (e.g., ±2g) provides higher sensitivity and precision for detecting small 
   * accelerations, but it can saturate if exposed to strong forces. Conversely, 
   * a larger range (e.g., ±16g) allows the sensor to measure stronger forces, but 
   * at the cost of reduced precision.
   * 
   * - ±4g: A moderate setting suitable for most robotic applications, providing 
   *   enough sensitivity for small movements while still being able to handle 
   *   normal forces during walking or turning without saturation.
   */
  ret = priv_i2c_write_byte(mpu6050_accel_configs[mpu6050_accel_config_idx].accel_config, 
                            mpu6050_i2c_bus, mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "MPU6050 accelerometer configuration failed");
    return ret;
  }

  /* Verify the sensor by reading the WHO_AM_I register */
  uint8_t who_am_i = 0;
  ret = priv_i2c_read_bytes(&who_am_i, 1, mpu6050_i2c_bus, 
                            mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK || who_am_i != k_mpu6050_who_am_i_response) {
    ESP_LOGE(mpu6050_tag, "MPU6050 WHO_AM_I verification failed (read: 0x%02X)", who_am_i);
    return ret;
  }

  mpu6050_data->state = k_mpu6050_ready; /* Sensor is initialized */
  ESP_LOGI(mpu6050_tag, "Sensor Configuration Complete");
  return ESP_OK;
}

void mpu6050_read(mpu6050_data_t *sensor_data)
{
  if (sensor_data == NULL) {
    ESP_LOGE(mpu6050_tag, "Sensor data pointer is NULL");
    return;
  }

  uint8_t accel_data[6], gyro_data[6];
  esp_err_t ret = priv_i2c_read_bytes(accel_data, 6, sensor_data->i2c_bus, 
                                      sensor_data->i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "Failed to read accelerometer data from MPU6050");
    sensor_data->state = k_mpu6050_error;
    return;
  }

  ret = priv_i2c_read_bytes(gyro_data, 6, mpu6050_i2c_bus, 
                            mpu6050_i2c_address, mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "Failed to read gyroscope data from MPU6050");
    sensor_data->state = k_mpu6050_error;
    return;
  }

  int16_t accel_x_raw = (int16_t)((accel_data[0] << 8) | accel_data[1]);
  int16_t accel_y_raw = (int16_t)((accel_data[2] << 8) | accel_data[3]);
  int16_t accel_z_raw = (int16_t)((accel_data[4] << 8) | accel_data[5]);

  int16_t gyro_x_raw = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
  int16_t gyro_y_raw = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
  int16_t gyro_z_raw = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);

  sensor_data->accel_x = accel_x_raw * mpu6050_accel_configs[mpu6050_accel_config_idx].accel_scale;
  sensor_data->accel_y = accel_y_raw * mpu6050_accel_configs[mpu6050_accel_config_idx].accel_scale;
  sensor_data->accel_z = accel_z_raw * mpu6050_accel_configs[mpu6050_accel_config_idx].accel_scale;

  sensor_data->gyro_x = gyro_x_raw * mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_scale;
  sensor_data->gyro_y = gyro_y_raw * mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_scale;
  sensor_data->gyro_z = gyro_z_raw * mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_scale;

  ESP_LOGI(mpu6050_tag, "Accel: [%f, %f, %f] g, Gyro: [%f, %f, %f] deg/s", 
      sensor_data->accel_x, sensor_data->accel_y, sensor_data->accel_z,
      sensor_data->gyro_x, sensor_data->gyro_y, sensor_data->gyro_z);
}

void mpu6050_tasks(void *sensor_data)
{
  sensor_data_t  *all_sensor_data = (sensor_data_t *)sensor_data;
  mpu6050_data_t *mpu6050_data    = &all_sensor_data->mpu6050_data;
  while (1) {
    mpu6050_read(mpu6050_data);
    vTaskDelay(mpu6050_polling_rate_ticks);
  }
}
