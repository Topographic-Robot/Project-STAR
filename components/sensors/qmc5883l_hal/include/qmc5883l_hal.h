/* components/sensors/qmc5883l_hal/include/qmc5883l_hal.h */

/* QMC5883L HAL (Hardware Abstraction Layer) Header File
 * This file provides the interface for interacting with the QMC5883L 3-axis magnetometer sensor.
 * The QMC5883L is a digital magnetometer that provides data over I2C, suitable for applications
 * requiring magnetic field detection and orientation sensing.
 *
 *******************************************************************************
 *
 *    +-----------------------+
 *    |       QMC5883L        |
 *    |-----------------------|
 *    | VCC  | 3.3V or 5V     |----------> VCC
 *    | GND  | Ground         |----------> GND
 *    | SCL  | I2C Clock      |----------> GPIO_NUM_22 (100,000Hz)
 *    | SDA  | I2C Data       |----------> GPIO_NUM_21 (100,000Hz)
 *    | DRDY | Data Ready Pin |----------> Floating (optional)
 *    +-----------------------+
 *
 *    Block Diagram for Wiring
 *
 *    +----------------------------------------------------+
 *    |                    QMC5883L                        |
 *    |                                                    |
 *    |   +----------------+    +-------------------+      |
 *    |   | Magnetometer   |--->| Signal Processing |      |
 *    |   | Sensor         |    | Unit              |      |
 *    |   +----------------+    +-------------------+      |
 *    |                                                    |
 *    |   +------------------+                             |
 *    |   | I2C Interface    |<----------------------------|
 *    |   | (SDA, SCL)       |                             |
 *    |   +------------------+                             |
 *    |                                                    |
 *    |   +------------------+                             |
 *    |   | Power Supply Unit|                             |
 *    |   | (PSU)            |                             |
 *    |   +------------------+                             |
 *    +----------------------------------------------------+
 *
 *    Internal Structure
 *
 ******************************************************************************/

#ifndef TOPOROBO_QMC5883L_HAL_H
#define TOPOROBO_QMC5883L_HAL_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "freertos/semphr.h"

/* Constants ******************************************************************/

/**
 * @brief The I2C address for the QMC5883L sensor.
 *
 * This constant defines the fixed I2C address of the QMC5883L sensor used
 * for communication. This address must be used in all I2C communication
 * commands sent to the QMC5883L sensor.
 */
extern const uint8_t qmc5883l_i2c_address;

/**
 * @brief The I2C bus number used by the ESP32 for communication with the QMC5883L sensor.
 *
 * This constant defines the I2C bus that the ESP32 will use to interface
 * with the QMC5883L sensor. It should be set to the appropriate I2C bus number.
 */
extern const uint8_t qmc5883l_i2c_bus;

/**
 * @brief Tag for logging messages related to the QMC5883L sensor.
 *
 * This constant defines a tag used for ESP_LOG messages, categorizing log outpu
 * related to the QMC5883L sensor, which simplifies log review.
 */
extern const char *qmc5883l_tag;

/**
 * @brief GPIO pin used for the I2C Serial Clock Line (SCL).
 *
 * Specifies the GPIO pin number connected to the SCL line
 * of the I2C bus.
 */
extern const uint8_t qmc5883l_scl_io;

/**
 * @brief GPIO pin used for the I2C Serial Data Line (SDA).
 *
 * Specifies the GPIO pin number connected to the SDA line
 * of the I2C bus.
 */
extern const uint8_t qmc5883l_sda_io;

/**
 * @brief I2C bus frequency in Hertz for communication with the QMC5883L sensor.
 *
 * This constant defines the frequency of the I2C bus for communication
 * with the QMC5883L sensor.
 */
extern const uint32_t qmc5883l_i2c_freq_hz;

/**
 * @brief Polling rate for the QMC5883L sensor in ticks.
 *
 * Defines the interval at which the ESP32 reads data from
 * the QMC5883L sensor in the `qmc5883l_tasks` function.
 */
extern const uint32_t qmc5883l_polling_rate_ticks;

/**
 * @brief Output Data Rate (ODR) setting for the QMC5883L sensor.
 *
 * This constant specifies the data rate for magnetometer measurements.
 */
extern const uint8_t qmc5883l_odr_setting;

/**
 * @brief Maximum number of retry attempts for sensor reinitialization.
 *
 * Defines the maximum number of consecutive retry attempts for reinitialization
 * in case of error, after which the retry interval doubles.
 */
extern const uint8_t qmc5883l_max_retries;

/**
 * @brief Initial interval between retry attempts in ticks.
 *
 * Defines the initial interval between retry attempts for reinitialization,
 * used in exponential backoff strategy.
 */
extern const uint32_t qmc5883l_initial_retry_interval;

/**
 * @brief Maximum interval for exponential backoff between retries in ticks.
 *
 * Sets the upper limit for the retry interval in exponential backoff.
 */
extern const uint32_t qmc5883l_max_backoff_interval;

/* Enums **********************************************************************/

/**
 * @enum qmc5883l_commands_
 * @brief Enum to represent the I2C commands for the QMC5883L sensor.
 *
 * This enum defines the possible I2C commands for the QMC5883L sensor.
 */
typedef enum : uint8_t {
  /* Below are power and reset commands: */
  k_qmc5883l_reset_cmd       = 0x0B, /**< Command to reset the sensor */
  k_qmc5883l_set_reset_cmd   = 0x0B, /**< Command to set/reset period */
  /* Below are control and configuration commands: */
  k_qmc5883l_ctrl1_cmd       = 0x09, /**< Control Register 1: Mode, ODR, Range, OSR */
  k_qmc5883l_ctrl2_cmd       = 0x0A, /**< Control Register 2: Soft reset and standby mode */
  /* Below are data output registers: */
  k_qmc5883l_data_xout_l_cmd = 0x00, /**< X-axis low byte data */
  k_qmc5883l_data_xout_h_cmd = 0x01, /**< X-axis high byte data */
  k_qmc5883l_data_yout_l_cmd = 0x02, /**< Y-axis low byte data */
  k_qmc5883l_data_yout_h_cmd = 0x03, /**< Y-axis high byte data */
  k_qmc5883l_data_zout_l_cmd = 0x04, /**< Z-axis low byte data */
  k_qmc5883l_data_zout_h_cmd = 0x05, /**< Z-axis high byte data */
  /* Below are status and temperature commands: */
  k_qmc5883l_status_cmd      = 0x06, /**< Status register: DRDY, overflow, and data overflow */
  k_qmc5883l_temp_l_cmd      = 0x07, /**< Temperature low byte */
  k_qmc5883l_temp_h_cmd      = 0x08, /**< Temperature high byte */
  /* Below are specific configuration values for Control Register 1: */
  k_qmc5883l_mode_standby    = 0x00, /**< Standby mode */
  k_qmc5883l_mode_continuous = 0x01, /**< Continuous measurement mode */
  /* Output Data Rate (ODR) configurations */
  k_qmc5883l_odr_10hz        = 0x00, /**< Output Data Rate: 10 Hz */
  k_qmc5883l_odr_50hz        = 0x04, /**< Output Data Rate: 50 Hz */
  k_qmc5883l_odr_100hz       = 0x08, /**< Output Data Rate: 100 Hz */
  k_qmc5883l_odr_200hz       = 0x0C, /**< Output Data Rate: 200 Hz */
  /* Full-scale range configurations */
  k_qmc5883l_range_2g        = 0x00, /**< Full-scale range: ±2 Gauss */
  k_qmc5883l_range_8g        = 0x10, /**< Full-scale range: ±8 Gauss */
  /* Over-sampling ratio (OSR) configurations */
  k_qmc5883l_osr_512         = 0x00, /**< Over-sampling ratio: 512 */
  k_qmc5883l_osr_256         = 0x40, /**< Over-sampling ratio: 256 */
  k_qmc5883l_osr_128         = 0x80, /**< Over-sampling ratio: 128 */
  k_qmc5883l_osr_64          = 0xC0  /**< Over-sampling ratio: 64 */
} qmc5883l_commands_t;

/**
 * @enum qmc5883l_states_
 * @brief Enum to represent the state of the BH1750 sensor.
 *
 * This enum defines the possible states for the BH1750 sensor.
 */
typedef enum : uint8_t {
  k_qmc5883l_ready          = 0x00, /**< Sensor is ready to read data. */
  k_qmc5883l_data_updated   = 0x01, /**< Sensor data has been updated. */
  k_qmc5883l_uninitialized  = 0x10, /**< Sensor is not initialized. */
  k_qmc5883l_error          = 0xF0, /**< A general catch all error */
  k_qmc5883l_power_on_error = 0xA1, /**< An error occurred during power on. */
  k_qmc5883l_reset_error    = 0xA2, /**< An error occurred during reset. */
} qmc5883l_states_t;

/* Structs ********************************************************************/

/**
 * @struct qmc5883l_scale_
 * @brief Structure that holds both the register value and the scaling factor for
 *        the QMC5883L magnetometer.
 */
typedef struct {
  uint8_t range; /**< Register value to set the full-scale range in QMC5883L */
  float   scale; /**< Scaling factor for converting raw data to microteslas (µT) */
} qmc5883l_scale_t;

/**
 * @struct qmc5883l_data_
 * @brief Structure to store QMC5883L sensor data.
 *
 * This structure holds the I2C bus number used for communication,
 * the magnetic field strength in the X, Y, and Z axes measured by the QMC5883L sensor,
 * the calculated heading (yaw), and a state flag used to track the sensor's status.
 */
typedef struct {
  uint8_t    i2c_address;        /**< I2C address used for communication */
  uint8_t    i2c_bus;            /**< I2C bus number used for communication */
  float      mag_x;              /**< Measured X-axis magnetic field in µT */
  float      mag_y;              /**< Measured Y-axis magnetic field in µT */
  float      mag_z;              /**< Measured Z-axis magnetic field in µT */
  float      heading;            /**< Calculated heading (yaw) in degrees */
  uint8_t    state;              /**< Sensor state */
  uint8_t    retry_count;        /**< Retry counter for exponential backoff */
  uint32_t   retry_interval;     /**< Current retry interval */
  TickType_t last_attempt_ticks; /**< Tick count of the last reinitialization attempt */
} qmc5883l_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Convert QMC5883L data to JSON.
 *
 * @param[in] sensor_data Pointer to `qmc5883l_data_t` structure
 */
char *qmc5883l_data_to_json(const qmc5883l_data_t *data);

/**
 * @brief Initializes the QMC5883L sensor for continuous measurement mode.
 *
 * This function initializes the I2C driver for the QMC5883L sensor, powering on the device,
 * resetting it, and configuring it to start taking measurements. The sensor configuration
 * settings include the operating mode, output data rate, and range.
 *
 * @param[in,out] sensor_data Pointer to the `qmc5883l_data_t` structure containing:
 *                            - `i2c_address`: Address for I2C communication (input).
 *                            - `i2c_bus`: Bus number for I2C communication (input).
 *                            - `state`: Updated to indicate initialization status (output).
 *
 * @return
 * - `ESP_OK` on successful initialization.
 * - An error code from `esp_err_t` if initialization fails.
 *
 * @note Call this function once during setup to handle initial sensor configuration.
 */
esp_err_t qmc5883l_init(void *sensor_data);

/**
 * @brief Reads magnetic field data from the QMC5883L sensor.
 *
 * This function retrieves magnetic field data from the QMC5883L sensor, converting
 * the raw output values from the sensor's X, Y, and Z registers into microtesla units.
 * Additionally, it calculates the heading (yaw) based on the X and Y components.
 *
 * @param[in,out] sensor_data Pointer to `qmc5883l_data_t` structure that contains:
 *                            - `mag_x`, `mag_y`, `mag_z`: Updated with the magnetic field data in µT (output).
 *                            - `heading`: Updated with the calculated heading in degrees (output).
 *                            - `state`: Updated to indicate data retrieval status (output).
 *
 * @return
 * - `ESP_OK` on successful read.
 * - `ESP_FAIL` on unsuccessful read.
 *
 * @note Ensure the QMC5883L sensor is initialized with `qmc5883l_init` before calling this function.
 */
esp_err_t qmc5883l_read(qmc5883l_data_t *sensor_data);

/**
 * @brief Manages exponential backoff and retries for QMC5883L sensor reinitialization on error.
 *
 * This function checks the sensor's state for errors. If an error is present, it initiates
 * a reinitialization attempt, applying an exponential backoff mechanism that increases the
 * retry interval after each failure up to a maximum interval. On successful reinitialization,
 * it resets the retry counter and interval.
 *
 * **Logic and Flow:**
 * - On error (`state` set to non-zero), the function checks the time since the
 *   last reinitialization attempt.
 * - If the retry interval has elapsed, a reinitialization attempt is made.
 *   - On success, `state` is set to `k_qmc5883l_ready`, and the retry counter and
 *     interval are reset.
 *   - On failure, `retry_count` is incremented.
 * - When the max retries (`qmc5883l_max_retries`) are reached, `retry_count` resets,
 *   and the interval doubles up to a limit (`qmc5883l_max_backoff_interval`).
 *
 * @param[in,out] sensor_data Pointer to `qmc5883l_data_t` structure containing:
 *                            - `state`: Sensor state (input/output), where a non-zero
 *                              value indicates an error.
 *                            - `retry_count`: Retry attempt counter (input/output).
 *                            - `retry_interval`: Retry interval in ticks (input/output).
 *                            - `last_attempt_ticks`: Time of last reinitialization attempt (input/output).
 *
 * @note This function should be called within `qmc5883l_tasks` to ensure regular
 *       error monitoring and recovery attempts with exponential backoff.
 */
void qmc5883l_reset_on_error(qmc5883l_data_t *sensor_data);

/**
 * @brief Executes periodic tasks for the QMC5883L sensor, including data reading and error handling.
 *
 * The `qmc5883l_tasks` function is intended to run continuously in a loop. It periodically
 * reads data from the QMC5883L sensor at intervals defined by `qmc5883l_polling_rate_ticks`,
 * checking for errors and using `qmc5883l_reset_on_error` to manage retries with backoff.
 *
 * @param[in,out] sensor_data Pointer to `qmc5883l_data_t` structure containing:
 *                            - `mag_x`, `mag_y`, `mag_z`, `heading`: Holds the latest sensor data (output).
 *                            - `state`, `retry_count`, `retry_interval`: Managed for error recovery (input/output).
 *
 * @note Execute this function as part of a FreeRTOS task to maintain continuous
 *       data acquisition and error management for the QMC5883L sensor.
 */
void qmc5883l_tasks(void *sensor_data);

#endif /* TOPOROBO_QMC5883L_HAL_H */
