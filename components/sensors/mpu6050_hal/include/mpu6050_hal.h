/* components/sensors/mpu6050_hal/include/mpu6050_hal.h */

/* MPU6050 HAL (Hardware Abstraction Layer) Header File
 * This file provides the interface for interacting with the MPU6050 6-axis accelerometer
 * and gyroscope. The MPU6050 sensor outputs acceleration and gyroscope data through
 * an I2C interface and integrates an internal Digital Motion Processor (DMP) to
 * handle complex motion data fusion.
 *
 * This header file defines the functions, constants, structures, and enumerations
 * required to initialize, configure, read data from, and handle errors in the MPU6050
 * sensor.
 *
 *******************************************************************************
 *
 * Note: The MPU6050's INT pin is an **aggregate interrupt output**. This means
 * that it asserts (typically goes low) when any of the enabled interrupt sources
 * are active. The microcontroller must read the `INT_STATUS` register to determine
 * which interrupt(s) have occurred.
 *
 *******************************************************************************
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
 *     | INT  | Interrupt Pin  |----------> GPIO_NUM_26 (D26)
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
 *     |   | INT Pin          |<----------------------------|
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
 *******************************************************************************/

#ifndef TOPOROBO_MPU6050_HAL_H
#define TOPOROBO_MPU6050_HAL_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Constants ******************************************************************/

/**
 * @brief The I2C address for the MPU6050 sensor.
 *
 * This constant defines the fixed I2C address of the MPU6050 sensor used
 * for communication. By default, the address is 0x68 when the ADD pin is connected
 * to GND. Connecting the ADD pin to VCC changes the address to 0x69, allowing
 * flexibility if multiple sensors are needed on the same I2C bus.
 */
extern const uint8_t mpu6050_i2c_address;

/**
 * @brief The I2C bus number used by the ESP32 for communication with the MPU6050 sensor.
 *
 * Defines the I2C bus that the ESP32 uses to interface with the MPU6050 sensor.
 * This allows for flexibility when multiple I2C buses are available on the ESP32.
 */
extern const uint8_t mpu6050_i2c_bus;

/**
 * @brief Tag for logging messages related to the MPU6050 sensor.
 *
 * Used as a tag in ESP_LOG messages, allowing easy identification of MPU6050-related log entries,
 * which is useful for debugging and monitoring sensor operations.
 */
extern const char *mpu6050_tag;

/**
 * @brief GPIO pin used for the I2C Serial Clock Line (SCL).
 *
 * Specifies the GPIO pin number connected to the I2C SCL line. Must be set based on wiring
 * configurations between the ESP32 and MPU6050.
 */
extern const uint8_t mpu6050_scl_io;

/**
 * @brief GPIO pin used for the I2C Serial Data Line (SDA).
 *
 * Specifies the GPIO pin number connected to the I2C SDA line. Must be set based on wiring
 * configurations between the ESP32 and MPU6050.
 */
extern const uint8_t mpu6050_sda_io;

/**
 * @brief I2C bus frequency in Hertz for communication with the MPU6050 sensor.
 *
 * Defines the frequency of the I2C bus used for communication. The standard
 * frequency of 100,000 Hz (100 kHz) ensures reliable data transfer. Higher
 * frequencies might be achievable depending on wiring and sensor quality.
 */
extern const uint32_t mpu6050_i2c_freq_hz;

/**
 * @brief Polling rate for the MPU6050 sensor in system ticks.
 *
 * Specifies the interval for reading data from the MPU6050 sensor in the
 * `mpu6050_tasks` function, allowing for consistent and efficient data collection.
 */
extern const uint32_t mpu6050_polling_rate_ticks;

/**
 * @brief Configures the sample rate divider for the MPU6050 sensor.
 *
 * Adjusting the sample rate divider can reduce processing load, save power, and
 * align with application requirements. By setting the divider to 9, the sensor
 * samples data at 100 Hz (with the default 1 kHz internal rate).
 *
 * Benefits:
 * - Reduces load on the CPU by adjusting data rates to match application needs.
 * - Saves power, particularly for battery-powered applications.
 *
 * Calculation:
 * - Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV).
 */
extern const uint8_t mpu6050_sample_rate_div;

/**
 * @brief Configures the Digital Low Pass Filter (DLPF) setting for the MPU6050 sensor.
 *
 * Sets the DLPF to filter out high-frequency noise, making data cleaner and more
 * stable for motion-sensitive applications.
 *
 * Benefits:
 * - Noise reduction: filters out high-frequency noise.
 * - Improved stability: better for low-speed movement detection.
 * - Reduced latency: provides faster response times.
 *
 * Example DLPF Settings:
 * - 260 Hz: minimal filtering, fastest response.
 * - 44 Hz: balanced filtering, suitable for general applications.
 */
extern const uint8_t mpu6050_config_dlpf;

/**
 * @brief GPIO pin used for the MPU6050 interrupt (INT pin).
 *
 * Specifies the GPIO pin number connected to the MPU6050 INT pin. This pin is used to receive
 * interrupt signals from the MPU6050 sensor, indicating events like new data availability.
 */
extern const uint8_t mpu6050_int_io;  /**< GPIO pin used for the MPU6050 interrupt */

/* Enums **********************************************************************/

/**
 * @enum mpu6050_interrupt_mode_
 * @brief Enumeration of all possible interrupt modes for the MPU6050 sensor.
 *
 * The MPU6050 sensor supports multiple interrupt sources that can be enabled simultaneously.
 * Each interrupt source corresponds to a specific event or condition detected by the sensor.
 * The INT pin on the MPU6050 is an aggregate output, asserting when any of the enabled interrupts occur.
 * To determine the specific interrupt(s) that triggered, the microcontroller must read the INT_STATUS register.
 *
 * Interrupt Modes:
 * - `k_mpu6050_int_mode_data_ready`: New data is available to read from the sensor.
 * - `k_mpu6050_int_mode_motion_detection`: Motion exceeding a specified threshold has been detected.
 * - `k_mpu6050_int_mode_zero_motion`: The sensor has detected no motion for a specified duration.
 * - `k_mpu6050_int_mode_free_fall`: The sensor has detected conditions indicative of free-fall.
 * - `k_mpu6050_int_mode_fifo_overflow`: The FIFO buffer has overflowed, indicating data was not read in time.
 * - `k_mpu6050_int_mode_i2c_mst`: An interrupt related to the I2C Master module has occurred.
 * - `k_mpu6050_int_mode_dmp`: A Digital Motion Processor (DMP) interrupt has occurred.
 * - `k_mpu6050_int_mode_pll_ready`: The Phase-Locked Loop (PLL) is ready.
 */
typedef enum {
  k_mpu6050_int_mode_data_ready,       /**< Data Ready Interrupt */
  k_mpu6050_int_mode_motion_detection, /**< Motion Detection Interrupt */
  k_mpu6050_int_mode_zero_motion,      /**< Zero Motion Detection Interrupt */
  k_mpu6050_int_mode_free_fall,        /**< Free-Fall Detection Interrupt */
  k_mpu6050_int_mode_fifo_overflow,    /**< FIFO Overflow Interrupt */
  k_mpu6050_int_mode_i2c_mst,          /**< I2C Master Interrupt */
  k_mpu6050_int_mode_dmp,              /**< DMP Interrupt */
  k_mpu6050_int_mode_pll_ready,        /**< PLL Ready Interrupt */
} mpu6050_interrupt_mode_t;

/**
 * @enum mpu6050_int_enable_bits_
 * @brief Enumeration of interrupt enable bits for the MPU6050 sensor.
 *
 * These constants represent the bits used to enable specific interrupts in the
 * MPU6050's INT_ENABLE register.
 */
typedef enum : uint8_t {
  k_mpu6050_int_enable_data_rdy   = 0x01, /**< Data Ready interrupt enable bit */
  k_mpu6050_int_enable_dmp_int    = 0x02, /**< DMP interrupt enable bit */
  k_mpu6050_int_enable_pll_rdy    = 0x04, /**< PLL Ready interrupt enable bit */
  k_mpu6050_int_enable_i2c_mst    = 0x08, /**< I2C Master interrupt enable bit */
  k_mpu6050_int_enable_fifo_oflow = 0x10, /**< FIFO Overflow interrupt enable bit */
  k_mpu6050_int_enable_zmot       = 0x20, /**< Zero Motion Detection interrupt enable bit */
  k_mpu6050_int_enable_mot        = 0x40, /**< Motion Detection interrupt enable bit */
  k_mpu6050_int_enable_ff         = 0x80, /**< Free-Fall Detection interrupt enable bit */
} mpu6050_int_enable_bits_t;

/**
 * @enum mpu6050_event_bits_
 * @brief Enumeration of event bits for the MPU6050 interrupt events.
 *
 * These constants represent the bits used in the event group to signal specific
 * interrupt events from the MPU6050 sensor.
 */
typedef enum : uint8_t {
  k_mpu6050_event_data_ready      = 0x01, /**< Bit for Data Ready event */
  k_mpu6050_event_motion_detected = 0x02, /**< Bit for Motion Detected event */
  k_mpu6050_event_zero_motion     = 0x04, /**< Bit for Zero Motion event */
  k_mpu6050_event_free_fall       = 0x08, /**< Bit for Free-Fall event */
  k_mpu6050_event_fifo_overflow   = 0x10, /**< Bit for FIFO Overflow event */
  k_mpu6050_event_i2c_mst         = 0x20, /**< Bit for I2C Master event */
  k_mpu6050_event_dmp             = 0x40, /**< Bit for DMP event */
  k_mpu6050_event_pll_ready       = 0x80, /**< Bit for PLL Ready event */
} mpu6050_event_bits_t;

/**
 * @enum mpu6050_commands_
 * @brief Enum representing the I2C commands for the MPU6050 sensor.
 *
 * Defines commands for configuring and operating the MPU6050 sensor, including power management,
 * data reading, and setting up sensitivity levels for accelerometer and gyroscope.
 */
typedef enum : uint8_t {
  /* Below are power commands: */
  k_mpu6050_power_down_cmd      = 0x40, /**< Command to power down the sensor (sleep mode) */
  k_mpu6050_power_on_cmd        = 0x00, /**< Command to power on the sensor (wake up from sleep) */
  k_mpu6050_reset_cmd           = 0x80, /**< Command to reset the sensor */
  k_mpu6050_pwr_mgmt_1_cmd      = 0x6B, /**< Power Management 1 */
  /* Below are config commands: */
  k_mpu6050_smplrt_div_cmd      = 0x19, /**< Sample Rate Divider */
  k_mpu6050_config_cmd          = 0x1A, /**< Configuration for DLPF (Digital Low Pass Filter) and FSYNC (external Frame Sync) */
  k_mpu6050_gyro_config_cmd     = 0x1B, /**< Gyroscope Configuration (full-scale range) */
  k_mpu6050_accel_config_cmd    = 0x1C, /**< Accelerometer Configuration (full-scale range) */
  k_mpu6050_who_am_i_cmd        = 0x75, /**< WHO_AM_I register */
  /* Interrupt configuration commands: */
  k_mpu6050_int_enable_cmd      = 0x38, /**< Interrupt Enable register */
  k_mpu6050_int_status_cmd      = 0x3A, /**< Interrupt Status register */
  /* Below are specific configuration values: */
  k_mpu6050_who_am_i_response   = 0x68, /**< The sensor should return this when who am i command is called */
  k_mpu6050_config_dlpf_260hz   = 0x00, /**< DLPF: 260Hz, 0ms delay */
  k_mpu6050_config_dlpf_184hz   = 0x01, /**< DLPF: 184Hz, 2.0ms delay */
  k_mpu6050_config_dlpf_94hz    = 0x02, /**< DLPF: 94Hz, 3.0ms delay */
  k_mpu6050_config_dlpf_44hz    = 0x03, /**< DLPF: 44Hz, 4.9ms delay */
  k_mpu6050_config_dlpf_21hz    = 0x04, /**< DLPF: 21Hz, 8.5ms delay */
  k_mpu6050_config_dlpf_10hz    = 0x05, /**< DLPF: 10Hz, 13.8ms delay */
  k_mpu6050_config_dlpf_5hz     = 0x06, /**< DLPF: 5Hz, 19.0ms delay */
  k_mpu6050_gyro_fs_250dps      = 0x00, /**< Gyroscope full scale: ±250°/s */
  k_mpu6050_gyro_fs_500dps      = 0x08, /**< Gyroscope full scale: ±500°/s */
  k_mpu6050_gyro_fs_1000dps     = 0x10, /**< Gyroscope full scale: ±1000°/s */
  k_mpu6050_gyro_fs_2000dps     = 0x18, /**< Gyroscope full scale: ±2000°/s */
  k_mpu6050_accel_fs_2g         = 0x00, /**< Accelerometer full scale: ±2g */
  k_mpu6050_accel_fs_4g         = 0x08, /**< Accelerometer full scale: ±4g */
  k_mpu6050_accel_fs_8g         = 0x10, /**< Accelerometer full scale: ±8g */
  k_mpu6050_accel_fs_16g        = 0x18, /**< Accelerometer full scale: ±16g */
  /* Below are High and Low Byte commands: */
  k_mpu6050_accel_xout_h_cmd    = 0x3B, /**< Accelerometer X-axis High byte */
  k_mpu6050_accel_xout_l_cmd    = 0x3C, /**< Accelerometer X-axis Low byte */
  k_mpu6050_accel_yout_h_cmd    = 0x3D, /**< Accelerometer Y-axis High byte */
  k_mpu6050_accel_yout_l_cmd    = 0x3E, /**< Accelerometer Y-axis Low byte */
  k_mpu6050_accel_zout_h_cmd    = 0x3F, /**< Accelerometer Z-axis High byte */
  k_mpu6050_accel_zout_l_cmd    = 0x40, /**< Accelerometer Z-axis Low byte */
  k_mpu6050_gyro_xout_h_cmd     = 0x43, /**< Gyroscope X-axis High byte */
  k_mpu6050_gyro_xout_l_cmd     = 0x44, /**< Gyroscope X-axis Low byte */
  k_mpu6050_gyro_yout_h_cmd     = 0x45, /**< Gyroscope Y-axis High byte */
  k_mpu6050_gyro_yout_l_cmd     = 0x46, /**< Gyroscope Y-axis Low byte */
  k_mpu6050_gyro_zout_h_cmd     = 0x47, /**< Gyroscope Z-axis High byte */
  k_mpu6050_gyro_zout_l_cmd     = 0x48, /**< Gyroscope Z-axis Low byte */
  /* Below are currently unused: */
  k_mpu6050_pwr_mgmt_2_cmd      = 0x6C, /**< Power Management 2 */
  k_mpu6050_fifo_en_cmd         = 0x23, /**< FIFO Enable register */
  k_mpu6050_fifo_count_h_cmd    = 0x72, /**< FIFO Count High byte */
  k_mpu6050_fifo_count_l_cmd    = 0x73, /**< FIFO Count Low byte */
  k_mpu6050_fifo_r_w_cmd        = 0x74, /**< FIFO Read/Write */
  k_mpu6050_temp_out_h_cmd      = 0x41, /**< Temperature High byte */
  k_mpu6050_temp_out_l_cmd      = 0x42, /**< Temperature Low byte */
  /* Below are unused since they are for factory-level testing */
  k_mpu6050_self_test_x_cmd     = 0x0D, /**< Self-test register for X-axis (for factory-level testing) */
  k_mpu6050_self_test_y_cmd     = 0x0E, /**< Self-test register for Y-axis (for factory-level testing) */
  k_mpu6050_self_test_z_cmd     = 0x0F, /**< Self-test register for Z-axis (for factory-level testing) */
  k_mpu6050_self_test_a_cmd     = 0x10, /**< Self-test register for Accelerometer (for factory-level testing) */
} mpu6050_commands_t;

/**
 * @enum mpu6050_states_
 * @brief Enum representing possible states of the MPU6050 sensor.
 *
 * Provides distinct states for tracking the status of the MPU6050 sensor,
 * particularly in error handling and data acquisition tasks.
 */
typedef enum : uint8_t {
  k_mpu6050_ready            = 0x00, /**< Sensor is ready to read data */
  k_mpu6050_data_updated     = 0x01, /**< Sensor data has been updated */
  k_mpu6050_uninitialized    = 0x10, /**< Sensor is not initialized */
  k_mpu6050_error            = 0xF0, /**< General error state */
  k_mpu6050_power_on_error   = 0xA1, /**< Error occurred during power on */
  k_mpu6050_reset_error      = 0xA3, /**< Error during reset command */
  k_mpu6050_dlp_config_error = 0xA4, /**< Error setting DLPF configuration */
} mpu6050_states_t;

/* Structs ********************************************************************/

/**
 * @struct mpu6050_accel_config_
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
 * @struct mpu6050_gyro_config_
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
 * @struct mpu6050_data_
 * @brief Structure to store MPU6050 sensor data and state.
 *
 * This structure holds essential data for interfacing with the MPU6050 sensor,
 * including accelerometer and gyroscope readings, state, and sensor configuration.
 *
 * Fields:
 * - `i2c_address`: I2C address for communication.
 * - `i2c_bus`: I2C bus that the ESP32 uses.
 * - `accel_x`, `accel_y`, `accel_z`: Acceleration readings in g-force.
 * - `gyro_x`, `gyro_y`, `gyro_z`: Gyroscope readings in degrees per second.
 * - `state`: Tracks sensor's operational state.
 * - `data_ready_sem`: Semaphore to signal that new data is ready to be read.
 */
typedef struct {
  uint8_t           i2c_address;    /**< I2C address used for communication */
  uint8_t           i2c_bus;        /**< I2C bus number used for communication */
  float             accel_x;        /**< Measured X-axis acceleration */
  float             accel_y;        /**< Measured Y-axis acceleration */
  float             accel_z;        /**< Measured Z-axis acceleration */
  float             gyro_x;         /**< Measured X-axis gyroscope data */
  float             gyro_y;         /**< Measured Y-axis gyroscope data */
  float             gyro_z;         /**< Measured Z-axis gyroscope data */
  float             temperature;    /**< Measured temperature from the sensor */
  uint8_t           state;          /**< Sensor state, set in `mpu6050_states_t` */
  SemaphoreHandle_t data_ready_sem; /**< Semaphore to signal data ready */
} mpu6050_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Convert MPU6050 data to JSON.
 *
 * @param[in] data Pointer to `mpu6050_data_t` structure containing sensor data.
 * @return A JSON-formatted string representing the sensor data.
 * @note The returned string should be freed by the caller to prevent memory leaks.
 */
char *mpu6050_data_to_json(const mpu6050_data_t *data);

/**
 * @brief Initializes the MPU6050 sensor for accelerometer and gyroscope data collection.
 *
 * The `mpu6050_init` function configures the I2C interface for communication with
 * the MPU6050 sensor, initializing the accelerometer and gyroscope in continuous
 * measurement mode. It also sets the desired sample rate, DLPF settings, and initial
 * state. If initialization fails, an error state is set in the `mpu6050_data_t` structure.
 *
 * @param[in,out] sensor_data Pointer to `mpu6050_data_t` structure for initializing sensor data.
 *                            - `i2c_address`: I2C address for the MPU6050 (input).
 *                            - `state`: Updated to indicate initialization success or failure (output).
 *
 * @return
 * - `ESP_OK` on successful initialization.
 * - An error code from `esp_err_t` if initialization fails.
 *
 * @note This function should be called once during setup to prepare the MPU6050
 * sensor for data acquisition.
 */
esp_err_t mpu6050_init(void *sensor_data);

/**
 * @brief Reads accelerometer and gyroscope data from the MPU6050 sensor.
 *
 * This function retrieves the latest data from the MPU6050 sensor, including acceleration
 * and angular velocity measurements. The data is converted and stored in the `mpu6050_data_t`
 * structure. If a read error occurs, the structure's fields are not updated.
 *
 * @param[in,out] sensor_data Pointer to `mpu6050_data_t` structure:
 *                            - `accel_x`, `accel_y`, `accel_z`: Updated with acceleration data in g-force (output).
 *                            - `gyro_x`, `gyro_y`, `gyro_z`: Updated with angular velocity data in degrees/second (output).
 *                            - `state`: Set to indicate the read status (output).
 *
 * @return
 * - `ESP_OK` on successful read.
 * - `ESP_FAIL` on unsuccessful read.
 *
 * @note Ensure the MPU6050 sensor is initialized using `mpu6050_init` before calling this function.
 */
esp_err_t mpu6050_read(mpu6050_data_t *sensor_data);

/**
 * @brief Manages error detection and recovery for the MPU6050 sensor.
 *
 * The `mpu6050_reset_on_error` function checks the sensor's state for errors and, if detected,
 * attempts to reinitialize the MPU6050. Successful reinitialization resets the state to `k_mpu6050_ready`.
 * If repeated attempts fail, the state is set to `k_mpu6050_reset_error`.
 *
 * @param[in,out] sensor_data Pointer to `mpu6050_data_t` structure:
 *                            - `state`: Current operational state (input/output).
 *
 * @note This function should be periodically called within `mpu6050_tasks` to handle errors and manage retries.
 */
void mpu6050_reset_on_error(mpu6050_data_t *sensor_data);

/**
 * @brief Periodically reads data from the MPU6050 sensor and manages errors.
 *
 * The `mpu6050_tasks` function runs continuously within a FreeRTOS task, reading data from the
 * MPU6050 sensor whenever the data ready interrupt occurs. If an error occurs,
 * it calls `mpu6050_reset_on_error` to handle retries.
 *
 * @param[in,out] sensor_data Pointer to `mpu6050_data_t` structure for:
 *                            - `accel_x`, `accel_y`, `accel_z`, `gyro_x`,
 *                            - `gyro_y`, `gyro_z`: Sensor data fields (output).
 *
 * @note Execute this function as part of a FreeRTOS task to maintain continuous data acquisition and error management.
 */
void mpu6050_tasks(void *sensor_data);

#endif /* TOPOROBO_MPU6050_HAL_H */

