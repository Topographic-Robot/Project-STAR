#include "mpu6050_hal.h"
#include "common/i2c.h"
#include <esp_log.h>

/* TODO: interrupt, FIFO, PWR_MGMT_2(remove axises we don't need to safe power */

/* Constants ******************************************************************/

/**
 * @brief Configure the sample rate divider for the MPU6050 sensor.
 *
 * Why is SMPLRT_DIV Important?
 * The MPU6050's sensors (gyroscope and accelerometer) can internally sample data 
 * at a high frequency, but depending on your application, you might not need to 
 * process that much data. SMPLRT_DIV gives you control over how often the sensor 
 * sends new data to the microcontroller.
 *
 * Reducing the sample rate can be useful for:
 * 
 * - Reducing the load on the processor: Slower sample rates mean less data to process,
 *   which reduces the amount of work the microcontroller needs to do.
 *
 * - Saving power: Processing less data can lead to lower overall system power 
 *   consumption, which is critical for battery-powered systems where efficient 
 *   use of resources is essential.
 * 
 * - Adjusting for system requirements: Some control loops (such as in robotic 
 *   systems or IMU-based systems) might only need 50 Hz or 100 Hz data to perform 
 *   well, rather than the maximum possible sample rate. Lower sample rates can 
 *   match the needs of these control systems without adding unnecessary data overhead.
 *
 * Set up the sample rate divider to 100 Hz.
 * The MPU6050 operates with a default gyro output rate of 1 kHz, so setting the
 * divider to 9 gives us a 100 Hz sample rate.
 */
const uint8_t  mpu6050_sample_rate_div    = 9;
const uint8_t  mpu6050_i2c_address        = 0x68;
const char    *mpu6050_tag                = "MPU6050";
const uint8_t  mpu6050_scl_io             = GPIO_NUM_22;
const uint8_t  mpu6050_sda_io             = GPIO_NUM_21;
const uint32_t mpu6050_freq_hz            = 100000;
const uint32_t mpu6050_polling_rate_ticks = pdMS_TO_TICKS(5 * 1000);

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

static const uint8_t mpu6050_gyro_config_idx  = 1; /* index of chosen values from above */
static const uint8_t mpu6050_accel_config_idx = 1; /* index of chosen values from above */

/* Public Functions ***********************************************************/

esp_err_t mpu6050_init(mpu6050_data_t *sensor_data, bool first_time)
{
  /* Initialize the struct, but not the semaphore until the state is 0x00.
   * This is to prevent memory allocation that might not be used. If we call
   * this function and it creates a new mutex very time then that is very bad.
   */
  if (first_time) {
    sensor_data->sensor_mutex = NULL; /* Set NULL, and change it later when its ready */
  }
  /* Start with invalid values since the sensor hasn't been read yet */
  sensor_data->gyro_x  = sensor_data->gyro_y  = sensor_data->gyro_z  = -1.0;
  sensor_data->accel_x = sensor_data->accel_y = sensor_data->accel_z = -1.0;

  sensor_data->state = k_mpu6050_uninitialized; /* Start in uninitialized state */

  /* Initialize the I2C bus with specified SCL, SDA pins, frequency, and bus number */
  esp_err_t ret = priv_i2c_init(mpu6050_scl_io, mpu6050_sda_io, mpu6050_freq_hz, 
                                sensor_data->i2c_bus, mpu6050_tag);

  if (ret != ESP_OK) {
    /* Log an error if the I2C driver installation fails */
    ESP_LOGE(mpu6050_tag, "I2C driver install failed: %s", esp_err_to_name(ret));
    return ret; /* Return the error code if initialization fails */
  }

  /* Power on the MPU6050 sensor */
  ret = priv_i2c_write_byte(k_mpu6050_power_on_cmd, sensor_data->i2c_bus, mpu6050_tag);
  if (ret != ESP_OK) {
    /* Log an error if powering on the MPU6050 fails */
    ESP_LOGE(mpu6050_tag, "MPU6050 power on failed");

    /* Update state */
    sensor_data->state = k_mpu6050_power_on_error;

    return ret; /* Return the error code if power on fails */
  }

  /* Delay for 10ms to allow the sensor to power on */
  vTaskDelay(10 / portTICK_PERIOD_MS);
  
  /* Reset the MPU6050 sensor */
  ret = priv_i2c_write_byte(k_mpu6050_reset_cmd, sensor_data->i2c_bus, mpu6050_tag);
  if (ret != ESP_OK) {
    /* Log an error if resetting the MPU6050 fails */
    ESP_LOGE(mpu6050_tag, "MPU6050 reset failed");

    /* Try a power cycle */
    ret = priv_i2c_write_byte(k_mpu6050_power_down_cmd, sensor_data->i2c_bus, mpu6050_tag);
    if (ret != ESP_OK) {
      /* Set and return the error */
      sensor_data->state = k_mpu6050_power_cycle_error;
      return ret;
    } else {
      /* Call this function again to turn it back on */
      mpu6050_init(sensor_data, false);
    }

    /* Update the state */
    sensor_data->state = k_mpu6050_reset_error;

    return ret; /* Return the error code if reset fails */
  }
  
  /* Delay for 10ms to allow the reset to take effect */
  vTaskDelay(10 / portTICK_PERIOD_MS);

  /* Set up the sample rate divider to 100 Hz. */
  /* The MPU6050 operates with a default gyro output rate of 1 kHz, so setting the */
  /* global constant mpu6050_sample_rate_div to 9 gives us a 100 Hz sample rate. */
  ret = priv_i2c_write_byte(mpu6050_sample_rate_div, sensor_data->i2c_bus, mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "MPU6050 sample rate configuration failed");
    return ret;
  }
  
  /* Configure the Digital Low Pass Filter (DLPF). */
  /* We choose 44 Hz bandwidth (DLPF_CFG = 3) to reduce high-frequency noise and smooth */
  /* the sensor readings, particularly useful to filter out vibrations from the hexapod's */
  /* mechanical movements. */
  ret = priv_i2c_write_byte(k_mpu6050_config_dlpf_44hz, sensor_data->i2c_bus, mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "MPU6050 DLPF configuration failed");
    return ret;
  }
  
  /* Configure the gyroscope full-scale range to ±500°/s. */
  /* This range provides enough sensitivity for normal walking and turning in the hexapod, */
  /* while also being able to handle faster rotations without saturating the sensor. */
  ret = priv_i2c_write_byte(mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_config, 
                            sensor_data->i2c_bus, mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "MPU6050 gyroscope configuration failed");
    return ret;
  }
  
  /* Configure the accelerometer full-scale range to ±4g. */
  /* This allows for a moderate range of accelerations that occur during walking and */
  /* turning, without being too sensitive to small movements or saturating during normal */
  /* operation. */
  ret = priv_i2c_write_byte(mpu6050_accel_configs[mpu6050_accel_config_idx].accel_config,
                            sensor_data->i2c_bus, mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "MPU6050 accelerometer configuration failed");
    return ret;
  }
  
  /* Verify the sensor by reading from the WHO_AM_I register. */
  /* The expected value for the MPU6050 is k_mpu6050_who_am_i_response, */
  /* so we check if the sensor is responding correctly. */
  uint8_t who_am_i = 0;
  ret = priv_i2c_read_bytes(&who_am_i, 1, sensor_data->i2c_bus, mpu6050_tag);
  if (ret != ESP_OK || who_am_i != k_mpu6050_who_am_i_response) {
    ESP_LOGE(mpu6050_tag, "MPU6050 WHO_AM_I verification failed (read: 0x%02X)", who_am_i);
    return ret;
  }

  /* At this point, no errors occurred and the sensor is initialized */
  /* Verify that this sensor didn't already have its mutex set */
  if (sensor_data->sensor_mutex == NULL) {
    /* Now we set the mutex value only once */
    sensor_data->sensor_mutex = xSemaphoreCreateMutex();
    if (sensor_data->sensor_mutex == NULL) {
      ESP_LOGE(mpu6050_tag, "ESP32 ran out of memory");

      sensor_data->state = k_mpu6050_error;
      return ESP_ERR_NO_MEM;
    }
  }

  sensor_data->state = k_mpu6050_ready; /* Sensor is initialized */
  return ESP_OK;
}

void mpu6050_read(mpu6050_data_t *sensor_data)
{
  /* Check if the sensor data is NULL */
  if (sensor_data == NULL) {
    ESP_LOGE(mpu6050_tag, "Sensor data pointer is NULL");
    return;
  }

  /* Try to take the semaphore before accessing shared data */
  if (xSemaphoreTake(sensor_data->sensor_mutex, 2 * mpu6050_polling_rate_ticks) != pdTRUE) {
    ESP_LOGW(mpu6050_tag, "Failed to take sensor mutex");
    return;
  }

  /* Array to store raw accelerometer and gyroscope data (6 bytes each) */
  uint8_t accel_data[6], gyro_data[6];

  /* Read 6 bytes of accelerometer data from the MPU6050 sensor over I2C */
  esp_err_t ret = priv_i2c_read_bytes(accel_data, 6, sensor_data->i2c_bus, mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "Failed to read accelerometer data from MPU6050");
    sensor_data->state = k_mpu6050_error;
    xSemaphoreGive(sensor_data->sensor_mutex); /* Ensure the semaphore is released */
    return;
  }

  /* Read 6 bytes of gyroscope data from the MPU6050 sensor over I2C */
  ret = priv_i2c_read_bytes(gyro_data, 6, sensor_data->i2c_bus, mpu6050_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(mpu6050_tag, "Failed to read gyroscope data from MPU6050");
    sensor_data->state = k_mpu6050_error;
    xSemaphoreGive(sensor_data->sensor_mutex); /* Ensure the semaphore is released */
    return;
  }

  /* Combine high and low bytes into 16-bit values for accelerometer and gyroscope data */
  int16_t accel_x_raw = (int16_t)((accel_data[0] << 8) | accel_data[1]);
  int16_t accel_y_raw = (int16_t)((accel_data[2] << 8) | accel_data[3]);
  int16_t accel_z_raw = (int16_t)((accel_data[4] << 8) | accel_data[5]);

  int16_t gyro_x_raw = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
  int16_t gyro_y_raw = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
  int16_t gyro_z_raw = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);

  /* Combine high and low bytes into 16-bit signed values by shifting the high byte 
   * left by 8 bits and adding the low byte to form the final 16-bit value. */

  /* Convert raw accelerometer data to g using the scaling factor */
  sensor_data->accel_x = accel_x_raw * mpu6050_accel_configs[mpu6050_accel_config_idx].accel_scale;
  sensor_data->accel_y = accel_y_raw * mpu6050_accel_configs[mpu6050_accel_config_idx].accel_scale;
  sensor_data->accel_z = accel_z_raw * mpu6050_accel_configs[mpu6050_accel_config_idx].accel_scale;

  /* Convert raw gyroscope data to degrees per second using the scaling factor */
  sensor_data->gyro_x = gyro_x_raw * mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_scale;
  sensor_data->gyro_y = gyro_y_raw * mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_scale;
  sensor_data->gyro_z = gyro_z_raw * mpu6050_gyro_configs[mpu6050_gyro_config_idx].gyro_scale;

  /* Log the converted values */
  ESP_LOGI(mpu6050_tag, "Accel: [%f, %f, %f] g, Gyro: [%f, %f, %f] deg/s", 
           sensor_data->accel_x, sensor_data->accel_y, sensor_data->accel_z,
           sensor_data->gyro_x, sensor_data->gyro_y, sensor_data->gyro_z);

  /* Give the semaphore back after accessing the shared data */
  xSemaphoreGive(sensor_data->sensor_mutex);
}

void mpu6050_reset_on_error(mpu6050_data_t *sensor_data) 
{
  /* Check if the state indicates any error using a bitwise AND with k_mpu6050_error */
  if (sensor_data->state & k_mpu6050_error) {
    ESP_LOGI(mpu6050_tag, "Error detected. Attempting to reset the MPU6050 sensor.");

    /* Attempt to initialize/reset the sensor */
    if (mpu6050_init(sensor_data, false) == ESP_OK) {
      /* If successful, set the state to ready */
      sensor_data->state = k_mpu6050_ready;
      ESP_LOGI(mpu6050_tag, "MPU6050 sensor reset successfully. State is now ready.");
    } else {
      /* If reset fails, set the state to reset error */
      sensor_data->state = k_mpu6050_reset_error;
      ESP_LOGE(mpu6050_tag, "Failed to reset the MPU6050 sensor. State set to reset error.");
    }
  }
}

void mpu6050_tasks(void *sensor_data)
{
  mpu6050_data_t *mpu6050_data = (mpu6050_data_t *)sensor_data;
  mpu6050_read(mpu6050_data);
  mpu6050_reset_on_error(mpu6050_data);

  vTaskDelay(mpu6050_polling_rate_ticks);
}
