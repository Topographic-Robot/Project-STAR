/* components/sensors/mpu6050_hal/include/mpu6050_hal.h */

#ifndef TOPOROBO_MPU6050_HAL_H
#define TOPOROBO_MPU6050_HAL_H

/* MPU6050 6-axis Gyroscope and Accelerometer Sensor IC */
/* Communicates over I2C protocol with configurable address 0x68 or 0x69 */

/*******************************************************************************
 *
 *     +-----------------------+
 *     |       MPU6050         |
 *     |-----------------------|
 *     | VCC  | 3.3V or 5V     |----------> VCC
 *     | GND  | Ground         |----------> GND
 *     | SDA  | I2C Data       |----------> GPIO_NUM_21 (100,000Hz)
 *     | SCL  | I2C Clock      |----------> GPIO_NUM_22 (100,000Hz)
 *     | XDA  | Aux I2C Data   |----------> Floating (leave unconnected if unused)
 *     | XCL  | Aux I2C Clock  |----------> Floating (leave unconnected if unused)
 *     | ADD  | I2C Address Pin|----------> GND (or VCC for 0x69 address)
 *     | INT  | Interrupt Pin  |----------> Floating (optional)
 *     +-----------------------+
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

extern const uint8_t  mpu6050_i2c_address;        /**< I2C address for MPU6050 */
extern const uint8_t  mpu6050_i2c_bus;            /**< I2C bus for MPU6050 */
extern const char    *mpu6050_tag;                /**< Tag for logs */
extern const uint8_t  mpu6050_scl_io;             /**< GPIO pin for I2C Serial Clock Line */
extern const uint8_t  mpu6050_sda_io;             /**< GPIO pin for I2C Serial Data Line */
extern const uint32_t mpu6050_i2c_freq_hz;        /**< I2C Bus Frequency in Hz */
extern const uint32_t mpu6050_polling_rate_ticks; /**< Polling rate (5 seconds) */

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
extern const uint8_t mpu6050_sample_rate_div;

/**
 * @brief Configure the Digital Low Pass Filter (DLPF) setting for the MPU6050 sensor.
 *
 * Why is DLPF Important?
 * The DLPF controls the low-pass filter cutoff frequency for the accelerometer and gyroscope data.
 * By setting an appropriate DLPF value, you can reduce high-frequency noise, which is beneficial 
 * for applications where smooth data output is important.
 *
 * Benefits of adjusting the DLPF setting:
 * - Noise reduction: Lowering the DLPF frequency reduces high-frequency noise, making the 
 *   data cleaner and more stable, which can be useful in applications sensitive to vibrations 
 *   and rapid movements.
 * - Improved stability: A lower cutoff frequency can improve measurement stability, particularly 
 *   in stationary or low-speed movements, by filtering out fast oscillations.
 * - Reduced latency: A higher DLPF cutoff frequency allows faster response times, making it suitable 
 *   for applications requiring quick reaction to movement, like gaming and high-speed robotics.
 *
 * Set up the DLPF to balance noise reduction and response time.
 * For example, setting the DLPF to 44 Hz (k_mpu6050_config_dlpf_44hz) provides a balance between
 * noise reduction and responsiveness, which works well for general applications that need 
 * clean, real-time motion data.
 *
 * DLPF Setting Reference:
 * - 260 Hz: Minimal filtering, fastest response.
 * - 44 Hz: Moderate filtering, suitable for many applications.
 * - 5 Hz: Strong filtering, suitable for applications requiring minimal noise at the expense of response time.
 */
extern const uint8_t mpu6050_config_dlpf;

/* Enums **********************************************************************/

/**
 * @enum mpu6050_commands_t
 * @brief Enum to represent the I2C commadns for the MPU6050 sensor.
 *
 * This enum defines the possible I2C commands for the MPU6050 sensor.
 */
typedef enum : uint8_t {
  /* Below are power commands: */
  k_mpu6050_power_down_cmd    = 0x40, /**< Command to power down the sensor (sleep mode) */
  k_mpu6050_power_on_cmd      = 0x00, /**< Command to power on the sensor (wake up from sleep) */
  k_mpu6050_reset_cmd         = 0x80, /**< Command to reset the sensor */
  k_mpu6050_pwr_mgmt_1_cmd    = 0x6B, /**< Power Management 1 */
  /* Below are config commands: */
  k_mpu6050_smplrt_div_cmd    = 0x19, /**< Sample Rate Divider */
  k_mpu6050_config_cmd        = 0x1A, /**< Configuration for DLPF (Digital Low Pass Filter) and FSYNC (external Frame Sync) */
  k_mpu6050_gyro_config_cmd   = 0x1B, /**< Gyroscope Configuration (full-scale range) */
  k_mpu6050_accel_config_cmd  = 0x1C, /**< Accelerometer Configuration (full-scale range) */
  k_mpu6050_who_am_i_cmd      = 0x75, /**< WHO_AM_I register */
  /* Below are specific configuration values: */
  k_mpu6050_who_am_i_response = 0x68, /**< The sensor should return this when who am i command is called */
  k_mpu6050_config_dlpf_260hz = 0x00, /**< DLPF: 260Hz, 0ms delay */
  k_mpu6050_config_dlpf_184hz = 0x01, /**< DLPF: 184Hz, 2.0ms delay */
  k_mpu6050_config_dlpf_94hz  = 0x02, /**< DLPF: 94Hz, 3.0ms delay */
  k_mpu6050_config_dlpf_44hz  = 0x03, /**< DLPF: 44Hz, 4.9ms delay */
  k_mpu6050_config_dlpf_21hz  = 0x04, /**< DLPF: 21Hz, 8.5ms delay */
  k_mpu6050_config_dlpf_10hz  = 0x05, /**< DLPF: 10Hz, 13.8ms delay */
  k_mpu6050_config_dlpf_5hz   = 0x06, /**< DLPF: 5Hz, 19.0ms delay */
  k_mpu6050_gyro_fs_250dps    = 0x00, /**< Gyroscope full scale: ±250°/s */
  k_mpu6050_gyro_fs_500dps    = 0x08, /**< Gyroscope full scale: ±500°/s */
  k_mpu6050_gyro_fs_1000dps   = 0x10, /**< Gyroscope full scale: ±1000°/s */
  k_mpu6050_gyro_fs_2000dps   = 0x18, /**< Gyroscope full scale: ±2000°/s */
  k_mpu6050_accel_fs_2g       = 0x00, /**< Accelerometer full scale: ±2g */
  k_mpu6050_accel_fs_4g       = 0x08, /**< Accelerometer full scale: ±4g */
  k_mpu6050_accel_fs_8g       = 0x10, /**< Accelerometer full scale: ±8g */
  k_mpu6050_accel_fs_16g      = 0x18, /**< Accelerometer full scale: ±16g */
  /* Below are High and Low Byte commands: */
  k_mpu6050_accel_xout_h_cmd  = 0x3B, /**< Accelerometer X-axis High byte */
  k_mpu6050_accel_xout_l_cmd  = 0x3C, /**< Accelerometer X-axis Low byte */
  k_mpu6050_accel_yout_h_cmd  = 0x3D, /**< Accelerometer Y-axis High byte */
  k_mpu6050_accel_yout_l_cmd  = 0x3E, /**< Accelerometer Y-axis Low byte */
  k_mpu6050_accel_zout_h_cmd  = 0x3F, /**< Accelerometer Z-axis High byte */
  k_mpu6050_accel_zout_l_cmd  = 0x40, /**< Accelerometer Z-axis Low byte */
  k_mpu6050_gyro_xout_h_cmd   = 0x43, /**< Gyroscope X-axis High byte */
  k_mpu6050_gyro_xout_l_cmd   = 0x44, /**< Gyroscope X-axis Low byte */
  k_mpu6050_gyro_yout_h_cmd   = 0x45, /**< Gyroscope Y-axis High byte */
  k_mpu6050_gyro_yout_l_cmd   = 0x46, /**< Gyroscope Y-axis Low byte */
  k_mpu6050_gyro_zout_h_cmd   = 0x47, /**< Gyroscope Z-axis High byte */
  k_mpu6050_gyro_zout_l_cmd   = 0x48, /**< Gyroscope Z-axis Low byte */
  /* Below are currently unused: */
  k_mpu6050_pwr_mgmt_2_cmd    = 0x6C, /**< Power Management 2 */
  k_mpu6050_fifo_en_cmd       = 0x23, /**< FIFO Enable register */
  k_mpu6050_fifo_count_h_cmd  = 0x72, /**< FIFO Count High byte */
  k_mpu6050_fifo_count_l_cmd  = 0x73, /**< FIFO Count Low byte */
  k_mpu6050_fifo_r_w_cmd      = 0x74, /**< FIFO Read/Write */
  k_mpu6050_temp_out_h_cmd    = 0x41, /**< Temperature High byte */
  k_mpu6050_temp_out_l_cmd    = 0x42, /**< Temperature Low byte */
  k_mpu6050_int_enable_cmd    = 0x38, /**< Interrupt Enable register */
  k_mpu6050_int_status_cmd    = 0x3A, /**< Interrupt Status register */
  /* Below are unused since they are for factory-level testing */
  k_mpu6050_self_test_x_cmd   = 0x0D, /**< Self-test register for X-axis (for factory-level testing) */
  k_mpu6050_self_test_y_cmd   = 0x0E, /**< Self-test register for Y-axis (for factory-level testing) */
  k_mpu6050_self_test_z_cmd   = 0x0F, /**< Self-test register for Z-axis (for factory-level testing) */
  k_mpu6050_self_test_a_cmd   = 0x10, /**< Self-test register for Accelerometer (for factory-level testing) */
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
 * @struct mpu6050_accel_config_t
 * @brief Structure that holds both the register value and the scaling factor for 
 *        accelerometer.
 *
 * The MPU6050 accelerometer outputs 16-bit signed integers representing the 
 * acceleration in the X, Y, and Z axes. The raw data values range from -32768 to 
 * 32767. The actual acceleration in gravitational units (g) depends on the 
 * configured full-scale range.
 *
 * To convert raw data to real-world values in g (gravitational units), we need 
 * a scaling factor. This scaling factor is calculated as:
 * 
 *   Scaling Factor = (Full-Scale Range) / (Maximum Raw Value)
 */
typedef struct {
  uint8_t accel_config; /**< Register value to set full-scale range in MPU6050 */
  float   accel_scale;  /**< Scaling factor for converting raw data to g */
} mpu6050_accel_config_t;

/**
 * @struct mpu6050_gyro_config_t
 * @brief Structure that holds both the register value and the scaling factor for 
 *        gyroscope.
 *
 * The MPU6050 gyroscope outputs 16-bit signed integers representing the angular 
 * velocity in the X, Y, and Z axes. The raw data values range from -32768 to 32767.
 * The actual angular velocity in degrees per second (°/s) depends on the configured 
 * full-scale range.
 *
 * To convert raw data to real-world values in °/s, we need a scaling factor.
 * This scaling factor is calculated as:
 * 
 *   Scaling Factor = (Full-Scale Range) / (Maximum Raw Value)
 */
typedef struct {
  uint8_t gyro_config; /**< Register value to set full-scale range in MPU6050 */
  float   gyro_scale;  /**< Scaling factor for converting raw data to °/s */
} mpu6050_gyro_config_t;

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
 */
typedef struct {
  uint8_t           i2c_address;  /**< I2C address used for communication */
  uint8_t           i2c_bus;      /**< I2C bus number used for communication */
  float             accel_x;      /**< Measured X-axis acceleration */
  float             accel_y;      /**< Measured Y-axis acceleration */
  float             accel_z;      /**< Measured Z-axis acceleration */
  float             gyro_x;       /**< Measured X-axis gyroscope data */
  float             gyro_y;       /**< Measured Y-axis gyroscope data */
  float             gyro_z;       /**< Measured Z-axis gyroscope data */
  float             temperature;  /**< Measured temperature from the sensor */
  uint8_t           state;        /**< Sensor state, set in `mpu6050_states_t` */
} mpu6050_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initialize the MPU6050 sensor over I2C.
 *
 * This function initializes the I2C driver for the MPU6050 sensor and sets
 * it up for measurement mode. It powers on the device, resets it, and configures
 * the sensor for accelerometer and gyroscope data collection. If the device fails
 * to be configured, this function will attempt to reset the device and configure it again.
 * However, this might not always work, and calling the function `mpu6050_reset_on_error` 
 * until the `mpu6050_data_t`'s `state` flag is `0x00` is recommended.
 *
 * @param[in,out] sensor_data Pointer to the `mpu6050_data_t` structure that 
 *   will hold the I2C bus number. The `i2c_bus` member will be set during 
 *   initialization.
 *
 * @return
 *   - ESP_OK on success.
 *   - An error code from the `esp_err_t` enumeration on failure.
 *
 * @note 
 *   - This should only be called once.
 *   - Delays are introduced after power on, reset, and sensor settings to ensure 
 *     proper initialization.
 *   The chosen configuration includes:
 *     - DLPF: 44 Hz to filter out high-frequency noise, useful for reducing 
 *       vibrations and smoothing the sensor data.
 *     - Gyroscope Full Scale: ±500°/s, providing enough sensitivity for regular 
 *       movements and turning, but capable of handling faster angular rates without 
 *       saturating.
 *     - Accelerometer Full Scale: ±4g, allowing for a moderate range of linear 
 *       acceleration without losing precision during regular motion.
 */
esp_err_t mpu6050_init(void *sensor_data);

/**
 * @brief Reads acceleration and gyroscope data from the MPU6050 sensor.
 *
 * This function reads data from the MPU6050 sensor and converts the
 * raw data into acceleration (g-force) and gyroscope (degrees per second) values.
 * 
 * @param[in,out] sensor_data Pointer to a `mpu6050_data_t` struct that contains:
 *   - `i2c_bus`: The I2C bus number to use for communication (input).
 *   - `accel`: Will be updated with the acceleration data (output).
 *   - `gyro`: Will be updated with the gyroscope data (output).
 *            If the reading fails, the values will be set to defaults or error codes.
 *
 * @note Ensure the MPU6050 is initialized before calling this function.
 */
void mpu6050_read(mpu6050_data_t *sensor_data);

/**
 * @brief Checks if the MPU6050 sensor encountered an error and attempts to reset it.
 *
 * This function checks the `state` flag of the `mpu6050_data_t` structure to 
 * determine if an error occurred. If an error is detected, it runs the MPU6050
 * initialization function to reset the sensor. If the reset is successful, the 
 * `state` flag will be updated to indicate that the sensor is ready.
 *
 * @param[in,out] sensor_data Pointer to a `mpu6050_data_t` struct that contains:
 *   - `state`: The current state of the sensor (input/output). 
 *              A non-zero value indicates an error. After a successful reset, 
 *              this flag is reset.
 */
void mpu6050_reset_on_error(mpu6050_data_t *sensor_data);

/**
 * @brief Executes periodic tasks for the MPU6050 sensor.
 * 
 * This function reads acceleration and gyroscope data from the MPU6050 sensor and
 * checks for any errors in the sensor state. If an error is detected, the sensor 
 * is reset to attempt recovery. The function also introduces a delay based 
 * on the polling rate to ensure continuous operation at the desired intervals.
 * 
 * @param[in,out] sensor_data Pointer to the `mpu6050_data_t` structure that
 *                            contains sensor data used for reading and error checking.
 * 
 * @note The delay is calculated from the `mpu6050_polling_rate_ticks` global variable
 *       which defines the pollign rate in system ticks.
 */
void mpu6050_tasks(void *sensor_data);

#endif /* TOPOROBO_MPU6050_HAL_H */

