#ifndef TOPOROBO_MPU6050_HAL_H
#define TOPOROBO_MPU6050_HAL_H

/* MPU6050 6-axis Gyroscope and Accelerometer Sensor IC */
/* Communicates over I2C protocol with configurable address 0x68 or 0x69 */

/*******************************************************************************
 *
 *     +----------------------------------------+
 *     |                MPU6050                 |
 *     |                                        |
 *     |   +----------------------------------+ |
 *     |   |                                  | |
 *     |   |   +--------------------------+   | |
 *     |   |   |        +-----------------+   | |
 *     |   |   | VCC    | 3.3V or 5V      |----------->| VCC
 *     |   |   +--------------------------+   | |
 *     |   |   | GND    | Ground          |----------->| GND
 *     |   |   +--------------------------+   | |
 *     |   |   | SDA    | I2C Data         |---------->| GPIO_NUM_21 (100,000Hz)
 *     |   |   +--------------------------+   | |
 *     |   |   | SCL    | I2C Clock        |---------->| GPIO_NUM_22 (100,000Hz)
 *     |   |   +--------------------------+   | |
 *     |   |   | AD0    | I2C Address Pin  |---------->| GND (or VCC for 0x69)
 *     |   |   +--------------------------+   | |
 *     |   |   | INT    | Interrupt Pin    |---------->| Floating (optional)
 *     |   |   +--------------------------+   | |
 *     |   |                                  | |
 *     |   +----------------------------------+ |
 *     +----------------------------------------+
 *
 *     Block Diagram for Wiring
 *
 *     +----------------------------------------------------+
 *     |                    MPU6050                         |
 *     |                                                    |
 *     |   +---------------+    +-------------------+       |
 *     |   | Accelerometer |--->| Signal Processing |       |
 *     |   | Sensor        |    | Unit              |       |
 *     |   +---------------+    +-------------------+       |
 *     |                                                    |
 *     |   +------------+     +-------------------+         |
 *     |   | Gyroscope  |---->| Signal Processing |         |
 *     |   | Sensor     |     | Unit              |         |
 *     |   +------------+     +-------------------+         |
 *     |                                                    |
 *     |   +------------------+                             |
 *     |   | I2C Interface    |<----------------------------|
 *     |   | (SDA, SCL)       |                             |
 *     |   +------------------+                             |
 *     |                                                    |
 *     |   +------------------+                             |
 *     |   | Power Supply Unit|                             |
 *     |   | (PSU)            |                             |
 *     |   +------------------+                             |
 *     +----------------------------------------------------+
 *
 *     Internal Structure
 *
 ******************************************************************************/

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/* Constants ******************************************************************/

extern const uint8_t  mpu6050_i2c_address;        /* I2C address for MPU6050 */
extern const char    *mpu6050_tag;                /* Tag for logs */
extern const uint8_t  mpu6050_scl_io;             /* GPIO pin for I2C Serial Clock Line */
extern const uint8_t  mpu6050_sda_io;             /* GPIO pin for I2C Serial Data Line */
extern const uint32_t mpu6050_freq_hz;            /* I2C Bus Frequency in Hz */
extern const uint32_t mpu6050_polling_rate_ticks; /* Polling rate (5 seconds) */

/* Enums **********************************************************************/

/**
 * @enum mpu6050_commands_t
 * @brief Enum to represent the I2C commadns for the MPU6050 sensor.
 *
 * This enum defines the possible I2C commands for the MPU6050 sensor.
 */
typedef enum : uint8_t {
  k_mpu6050_power_down_cmd   = 0x40, /**< Command to power down the sensor (sleep mode) */
  k_mpu6050_power_on_cmd     = 0x00, /**< Command to power on the sensor (wake up from sleep) */
  k_mpu6050_smplrt_div_cmd   = 0x19, /**< Sample Rate Divider */
  k_mpu6050_config_cmd       = 0x1A, /**< Configuration for DLPF (Digital Low Pass Filter) and FSYNC (external Frame Sync) */
  k_mpu6050_gyro_config_cmd  = 0x1B, /**< Gyroscope Configuration (full-scale range) */
  k_mpu6050_accel_config_cmd = 0x1C, /**< Accelerometer Configuration (full-scale range) */
  k_mpu6050_accel_xout_h_cmd = 0x3B, /**< Accelerometer X-axis High byte */
  k_mpu6050_accel_xout_l_cmd = 0x3C, /**< Accelerometer X-axis Low byte */
  k_mpu6050_accel_yout_h_cmd = 0x3D, /**< Accelerometer Y-axis High byte */
  k_mpu6050_accel_yout_l_cmd = 0x3E, /**< Accelerometer Y-axis Low byte */
  k_mpu6050_accel_zout_h_cmd = 0x3F, /**< Accelerometer Z-axis High byte */
  k_mpu6050_accel_zout_l_cmd = 0x40, /**< Accelerometer Z-axis Low byte */
  k_mpu6050_gyro_xout_h_cmd  = 0x43, /**< Gyroscope X-axis High byte */
  k_mpu6050_gyro_xout_l_cmd  = 0x44, /**< Gyroscope X-axis Low byte */
  k_mpu6050_gyro_yout_h_cmd  = 0x45, /**< Gyroscope Y-axis High byte */
  k_mpu6050_gyro_yout_l_cmd  = 0x46, /**< Gyroscope Y-axis Low byte */
  k_mpu6050_gyro_zout_h_cmd  = 0x47, /**< Gyroscope Z-axis High byte */
  k_mpu6050_gyro_zout_l_cmd  = 0x48, /**< Gyroscope Z-axis Low byte */
  k_mpu6050_pwr_mgmt_1_cmd   = 0x6B, /**< Power Management 1 */
  k_mpu6050_who_am_i_cmd     = 0x75, /**< WHO_AM_I register */
  /* Below are currently unused: */
  k_mpu6050_pwr_mgmt_2_cmd   = 0x6C, /**< Power Management 2 */
  k_mpu6050_fifo_en_cmd      = 0x23, /**< FIFO Enable register */
  k_mpu6050_fifo_count_h_cmd = 0x72, /**< FIFO Count High byte */
  k_mpu6050_fifo_count_l_cmd = 0x73, /**< FIFO Count Low byte */
  k_mpu6050_fifo_r_w_cmd     = 0x74, /**< FIFO Read/Write */
  k_mpu6050_temp_out_h_cmd   = 0x41, /**< Temperature High byte */
  k_mpu6050_temp_out_l_cmd   = 0x42, /**< Temperature Low byte */
  k_mpu6050_int_enable_cmd   = 0x38, /**< Interrupt Enable register */
  k_mpu6050_int_status_cmd   = 0x3A, /**< Interrupt Status register */
  /* Below are unused since they are for factory-level testing */
  k_mpu6050_self_test_x_cmd  = 0x0D, /**< Self-test register for X-axis (for factory-level testing) */
  k_mpu6050_self_test_y_cmd  = 0x0E, /**< Self-test register for Y-axis (for factory-level testing) */
  k_mpu6050_self_test_z_cmd  = 0x0F, /**< Self-test register for Z-axis (for factory-level testing) */
  k_mpu6050_self_test_a_cmd  = 0x10, /**< Self-test register for Accelerometer (for factory-level testing) */
} mpu6050_commands_t;

/**
 * @enum mpu6050_states_t
 * @brief Enum to represent the state of the MPU6050 sensor.
 *
 * This enum defines the possible states for the MPU6050 sensor.
 */
typedef enum : uint8_t {
  k_mpu6050_ready               = 0x00, /**< Sensor is ready to read data */
  k_mpu6050_data_updated        = 0x01, /**< Sensor data has been updated */
  k_mpu6050_uninitialized       = 0x10, /**< Sensor is not initialized */
  k_mpu6050_error               = 0xF0, /**< A general catch-all error */
  k_mpu6050_power_on_error      = 0xA1, /**< An error occurred during power on */
  k_mpu6050_power_down_error    = 0xA2, /**< An error occurred during power down (sleep) */
  k_mpu6050_reset_error         = 0xA3, /**< An error occurred during reset */
  k_mpu6050_dlp_config_error    = 0xA4, /**< An error occurred while setting DLPF configuration */
  k_mpu6050_sensor_config_error = 0xA5, /**< An error occurred while setting sensor configurations (accel, gyro) */
  k_mpu6050_power_cycle_error   = 0xA6, /**< An error occurred during the power cycle */
} mpu6050_states_t;

/* Structs ********************************************************************/

/**
 * @struct mpu6050_data_t
 * @brief Structure to store MPU6050 sensor data.
 *
 * This structure holds the I2C bus number used for communication,
 * the accelerometer and gyroscope values measured by the MPU6050 sensor,
 * and a state flag used to track the sensor's status.
 *
 * These flags are set in the `mpu6050_states_t` enum, always verify
 * with the enum.
 *
 * Additionally, the structure contains a mutex (`sensor_mutex`) to ensure
 * thread-safe access when the sensor data is being read or updated.
 */
typedef struct {
  uint8_t           i2c_bus;      /**< I2C bus number used for communication */
  float             accel_x;      /**< Measured X-axis acceleration */
  float             accel_y;      /**< Measured Y-axis acceleration */
  float             accel_z;      /**< Measured Z-axis acceleration */
  float             gyro_x;       /**< Measured X-axis gyroscope data */
  float             gyro_y;       /**< Measured Y-axis gyroscope data */
  float             gyro_z;       /**< Measured Z-axis gyroscope data */
  float             temperature;  /**< Measured temperature from the sensor */
  uint8_t           state;        /**< Sensor state, set in `mpu6050_states_t` */
  SemaphoreHandle_t sensor_mutex; /**< Mutex for protecting access to sensor data */
} mpu6050_data_t;

/* Public Functions ***********************************************************/

#endif /* TOPOROBO_MPU6050_HAL_H */

