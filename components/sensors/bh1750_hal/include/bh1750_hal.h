/* components/sensors/bh1750_hal/include/bh1750_hal.h */

/* BH1750 HAL (Hardware Abstraction Layer) Header File
 * This file provides the interface for interacting with the BH1750 ambient light sensor.
 * The BH1750 is a digital 16-bit serial output type ambient light sensor IC, which communicates
 * over I2C with the ESP32. This header file defines the functions, constants, structures, and
 * enumerations required to control and read data from the BH1750 sensor.
 *
 *******************************************************************************
 *
 *    +-----------------------+
 *    |        BH1750         |
 *    |-----------------------|
 *    | VCC  | 2.4V to 3.6V   |----------> VCC
 *    | GND  | Ground         |----------> GND
 *    | SCL  | Serial Clock   |----------> GPIO_NUM_22 (100,000Hz)
 *    | SDA  | Serial Data    |----------> GPIO_NUM_21 (100,000Hz)
 *    | ADDR | I2C Address    |----------> GND
 *    +-----------------------+
 *
 *    Block Diagram for wiring
 *
 *    +----------------------------------------------------+
 *    |                    BH1750                          |
 *    |                                                    |
 *    |   +------------+     +-------------------+         |
 *    |   | Photodiode |---->| Transimpedance    |         |
 *    |   | (Sensor)   |     | Amplifier (TIA)   |         |
 *    |   +------------+     +-------------------+         |
 *    |                                                    |
 *    |   +------------------+      +----------------+     |
 *    |   | Analog-to-Digital|----->| Control Logic  |     |
 *    |   | Converter (ADC)  |      |                |     |
 *    |   +------------------+      +----------------+     |
 *    |                                                    |
 *    |   +---------------------+                          |
 *    |   | Serial Interface    |<-------------------------|
 *    |   | (I2C)               |                          |
 *    |   +---------------------+                          |
 *    |                                                    |
 *    |   +---------------------+                          |
 *    |   | Power Supply Unit   |                          |
 *    |   | (PSU)               |                          |
 *    |   +---------------------+                          |
 *    +----------------------------------------------------+
 *
 *    Internal Structure
 *
 *******************************************************************************/

#ifndef TOPOROBO_BH1750_HAL_H
#define TOPOROBO_BH1750_HAL_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Constants ******************************************************************/

/**
 * @brief The I2C address for the BH1750 sensor.
 *
 * This constant defines the fixed I2C address of the BH1750 sensor used
 * for communication. The default address is 0x23 when the ADDR pin is
 * connected to GND. This address must be used in all I2C communication
 * commands sent to the BH1750 sensor.
 */
extern const uint8_t bh1750_i2c_address;

/**
 * @brief The I2C bus number used by the ESP32 for communication with the BH1750 sensor.
 *
 * This constant defines the I2C bus that the ESP32 will use to interface
 * with the BH1750 sensor. It should be set to the appropriate I2C bus number
 * (e.g., I2C_NUM_0 or I2C_NUM_1) as configured in the ESP-IDF. This allows
 * for flexibility if multiple I2C buses are available or different devices
 * are attached to separate buses.
 */
extern const uint8_t bh1750_i2c_bus;

/**
 * @brief Tag for logging messages related to the BH1750 sensor.
 *
 * This constant defines a tag used for ESP_LOG messages, allowing for easy
 * identification of log output related to the BH1750 sensor in the ESP32's
 * logging system. It is particularly useful for debugging, as it categorizes
 * log entries associated with this sensor, simplifying the log review process.
 */
extern const char *bh1750_tag;

/**
 * @brief GPIO pin used for the I2C Serial Clock Line (SCL).
 *
 * This constant specifies the GPIO pin number connected to the SCL line
 * of the I2C bus, used by the ESP32 for clock signals to communicate with
 * the BH1750 sensor. The pin must be correctly set based on the wiring
 * configuration of the ESP32 and the sensor.
 */
extern const uint8_t bh1750_scl_io;

/**
 * @brief GPIO pin used for the I2C Serial Data Line (SDA).
 *
 * This constant specifies the GPIO pin number connected to the SDA line
 * of the I2C bus, used by the ESP32 to send and receive data from the
 * BH1750 sensor. Like `bh1750_scl_io`, this should be set based on the
 * physical connections between the ESP32 and the sensor.
 */
extern const uint8_t bh1750_sda_io;

/**
 * @brief I2C bus frequency in Hertz for communication with the BH1750 sensor.
 *
 * This constant defines the frequency of the I2C bus for communication
 * with the BH1750 sensor. A standard frequency of 100,000 Hz (100 kHz) is
 * recommended to ensure reliable data transfer without timing issues.
 * Higher frequencies might not be supported, depending on the wiring and
 * sensor quality.
 */
extern const uint32_t bh1750_i2c_freq_hz;

/**
 * @brief Polling rate for the BH1750 sensor in ticks.
 *
 * This constant sets the interval at which the ESP32 reads data from
 * the BH1750 sensor in the `bh1750_tasks` function. It is defined in
 * system ticks to facilitate timing within the FreeRTOS environment,
 * ensuring consistent and efficient data collection at the desired intervals.
 */
extern const uint32_t bh1750_polling_rate_ticks;

/**
 * @brief Maximum number of retry attempts for sensor reinitialization.
 *
 * This constant sets the maximum number of consecutive retry attempts tha
 * the system will make to reinitialize the BH1750 sensor in case of an error.
 * After reaching this limit, the retry interval is doubled as part of
 * the exponential backoff strategy. This allows the system to gracefully
 * handle intermittent sensor failures without continuously retrying.
 */
extern const uint8_t bh1750_max_retries;

/**
 * @brief Initial interval between retry attempts in seconds, converted to ticks.
 *
 * This constant defines the initial interval between retry attempts when the
 * BH1750 sensor encounters an error. The interval is used in the exponential
 * backoff strategy, doubling after each set of `bh1750_max_retries` attempts, up to
 * a maximum defined by `bh1750_max_backoff_interval`. This strategy prevents
 * excessive retries in quick succession, providing the sensor time to recover.
 */
extern const uint32_t bh1750_initial_retry_interval;

/**
 * @brief Maximum interval for exponential backoff between retries in seconds, converted to ticks.
 *
 * This constant defines the upper limit for the retry interval in the
 * exponential backoff mechanism. Once this maximum is reached, the interval
 * will no longer double, ensuring that retry attempts are eventually spaced
 * far enough apart to prevent frequent reinitialization. This helps to avoid
 * unnecessary load on the system while ensuring the sensor can recover when possible.
 */
extern const uint32_t bh1750_max_backoff_interval;

/* Enums **********************************************************************/

/**
 * @enum bh1750_commands_
 * @brief Enum to represent the I2C commands for the BH1750 sensor.
 *
 * This enum defines the possible I2C commands for the BH1750 sensor. Each command
 * is associated with a specific operation, such as powering on the sensor, setting
 * different measurement modes, or adjusting measurement timing.
 *
 * **Commands:**
 * - `k_bh1750_power_down_cmd`: Puts the sensor into a power-down state to save energy.
 * - `k_bh1750_power_on_cmd`: Powers on the sensor, allowing it to begin measurements.
 * - `k_bh1750_reset_cmd`: Resets the data register value without changing the measurement mode.
 * - `k_bh1750_cont_low_res_mode_cmd`: Sets the sensor to continuously measure light intensity
 *   at a low resolution (4 lx/bit).
 * - Additional high-resolution and one-time measurement commands are available but are currently
 *   unused in this implementation.
 */
typedef enum : uint8_t {
  k_bh1750_power_down_cmd                = 0x00, /**< Power down command */
  k_bh1750_power_on_cmd                  = 0x01, /**< Power on command */
  k_bh1750_reset_cmd                     = 0x07, /**< Reset command */
  k_bh1750_cont_low_res_mode_cmd         = 0x13, /**< Continuously low-resolution mode command */
  /* Below are currently unused: */
  k_bh1750_cont_high_res_mode_cmd        = 0x10, /**< Continuously high-resolution mode command */
  k_bh1750_cont_high_res_mode2_cmd       = 0x11, /**< Continuously high-resolution mode 2 command */
  k_bh1750_one_time_high_res_mode_cmd    = 0x20, /**< One-time high-resolution mode command */
  k_bh1750_one_time_high_res_mode2_cmd   = 0x21, /**< One-time high-resolution mode 2 command */
  k_bh1750_one_time_low_res_mode_cmd     = 0x23, /**< One-time low-resolution mode command */
  k_bh1750_change_meas_time_high_bit_cmd = 0x40, /**< Change measurement time high bit command */
  k_bh1750_change_meas_time_low_bit_cmd  = 0x60, /**< Change measurement time low bit command */
} bh1750_commands_t;

/**
 * @enum bh1750_states_
 * @brief Enum to represent the state of the BH1750 sensor.
 *
 * This enum defines the various states that the BH1750 sensor can be in during
 * its operation. Each state is represented by a unique value, allowing the system
 * to determine the current condition of the sensor and take appropriate action.
 *
 * **States:**
 * - `k_bh1750_ready`: The sensor is initialized and ready to read data.
 * - `k_bh1750_data_updated`: The sensor data has been updated and is ready for use.
 * - `k_bh1750_uninitialized`: The sensor is not initialized.
 * - `k_bh1750_error`: A general error has occurred.
 * - Additional specific error states (`k_bh1750_power_on_error`, `k_bh1750_reset_error`, etc.)
 *   provide more detailed information about what went wrong during sensor operation.
 */
typedef enum : uint8_t {
  k_bh1750_ready              = 0x00, /**< Sensor is ready to read data. */
  k_bh1750_data_updated       = 0x01, /**< Sensor data has been updated. */
  k_bh1750_uninitialized      = 0x10, /**< Sensor is not initialized. */
  k_bh1750_error              = 0xF0, /**< A general catch all error */
  k_bh1750_power_on_error     = 0xA1, /**< An error occurred during power on. */
  k_bh1750_reset_error        = 0xA2, /**< An error occurred during reset. */
  k_bh1750_cont_low_res_error = 0xA3, /**< An error occurred setting continuous high-resolution mode */
  k_bh1750_power_cycle_error  = 0xA4, /**< An error occurred when trying to power cycle */
} bh1750_states_t;

/* Structs ********************************************************************/

/**
 * @struct bh1750_data_
 * @brief Structure to store BH1750 sensor data and status information.
 *
 * The `bh1750_data_t` structure holds essential data for interfacing with
 * the BH1750 sensor, including the I2C address and bus number, as well as
 * sensor-specific data such as light intensity in lux, state, retry information,
 * and timing variables to manage error handling and reinitialization.
 *
 * **Fields:**
 * - `i2c_address`: The I2C address for communication.
 * - `i2c_bus`: The I2C bus number.
 * - `lux`: The latest light intensity reading from the sensor, in lux.
 * - `state`: The current state of the sensor, defined by the `bh1750_states_t` enum.
 * - `retry_count`: Tracks the number of consecutive reinitialization attempts.
 * - `retry_interval`: Current interval between retry attempts, which increases
 *   exponentially if retries continue to fail.
 * - `last_attempt_ticks`: Stores the tick count of the last reinitialization attempt,
 *   allowing for time-based retry control.
 */
typedef struct {
  uint8_t    i2c_address;        /**< I2C address */
  uint8_t    i2c_bus;            /**< I2C bus */
  float      lux;                /**< Light intensity in lux */
  uint8_t    state;              /**< Sensor state */
  uint8_t    retry_count;        /**< Retry counter for exponential backoff */
  uint32_t   retry_interval;     /**< Current retry interval */
  TickType_t last_attempt_ticks; /**< Tick count of the last reinitialization attempt */
} bh1750_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Convert BH1750 data to JSON.
 *
 * @param[in] sensor_data Pointer to `bh1750_data_t` structure
 */
char *bh1750_data_to_json(const bh1750_data_t *data);

/**
 * @brief Initializes the BH1750 sensor for continuous high-resolution mode.
 *
 * The `bh1750_init` function sets up the I2C interface for the BH1750 sensor and
 * configures it to measure ambient light in continuous high-resolution mode.
 * Initialization includes powering on the sensor, resetting it, and setting the
 * measurement mode. If initialization fails, the function sets an error state
 * within the `bh1750_data_t` structure.
 *
 * @param[in,out] sensor_data Pointer to the `bh1750_data_t` structure to hold
 *                            sensor-specific data, including I2C address and
 *                            bus configuration.
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
esp_err_t bh1750_init(void *sensor_data);

/**
 * @brief Reads light intensity data from the BH1750 sensor.
 *
 * This function retrieves the most recent light intensity measurement from the
 * BH1750 sensor via I2C. If the read is successful, `lux` in the `bh1750_data_t`
 * structure is updated with the new value in lux. If it fails, `lux` is set to
 * -1.0, and the `state` is updated to indicate an error.
 *
 * @param[in,out] sensor_data Pointer to `bh1750_data_t` structure:
 *                            - `lux`: Updated with the light intensity in lux (output).
 *                            - `state`: Updated to indicate the read status (output).
 *
 * @return
 * - `ESP_OK` on successful read.
 * - `ESP_FAIL` on unsuccessful read.
 *
 * @note Ensure the BH1750 sensor is initialized with `bh1750_init` before calling this function.
 */
esp_err_t bh1750_read(bh1750_data_t *sensor_data);

/**
 * @brief Manages exponential backoff and retries for BH1750 sensor reinitialization on error.
 *
 * This function handles reinitialization of the BH1750 sensor when errors are detected.
 * It employs an exponential backoff strategy, increasing the retry interval if repeated
 * attempts fail. The retry counter resets after each successful reinitialization.
 *
 * **Logic and Flow:**
 * - On error (`state` set to non-zero), the function checks the time since the
 *   last reinitialization attempt.
 * - If the retry interval has elapsed, a reinitialization attempt is made.
 *   - On success, `state` is set to `k_bh1750_ready`, and the retry counter and
 *     interval are reset.
 *   - On failure, `retry_count` is incremented.
 * - When the max retries (`bh1750_max_retries`) are reached, `retry_count` resets,
 *   and the interval doubles up to a limit (`bh1750_max_backoff_interval`).
 *
 * @param[in,out] sensor_data Pointer to `bh1750_data_t` structure containing:
 *                            - `state`: Sensor state (input/output), where a non-zero
 *                              value indicates an error.
 *                            - `retry_count`: Retry attempt counter (input/output).
 *                            - `retry_interval`: Retry interval in ticks (input/output).
 *                            - `last_attempt_ticks`: Time of last reinitialization attempt (input/output).
 *
 * @note This function should be called within `bh1750_tasks` to ensure regular
 *       error monitoring and recovery attempts with exponential backoff.
 */
void bh1750_reset_on_error(bh1750_data_t *sensor_data);

/**
 * @brief Executes periodic tasks for the BH1750 sensor, including data reading and error handling.
 *
 * The `bh1750_tasks` function is intended to run continuously in a loop. It periodically
 * reads data from the BH1750 sensor at intervals defined by `bh1750_polling_rate_ticks`,
 * checking for errors and using `bh1750_reset_on_error` to manage retries with backoff.
 *
 * @param[in,out] sensor_data Pointer to `bh1750_data_t` structure for sensor data:
 *                            - `lux`: Holds the latest light intensity data (output).
 *                            - `state`, `retry_count`, `retry_interval`: Managed for error recovery (input/output).
 *
 * @note Execute this function as part of a FreeRTOS task to maintain continuous
 *       data acquisition and error management for the BH1750 sensor.
 */
void bh1750_tasks(void *sensor_data);

#endif /* TOPOROBO_BH1750_HAL_H */

