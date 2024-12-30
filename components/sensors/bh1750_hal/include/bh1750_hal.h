/* components/sensors/bh1750_hal/include/bh1750_hal.h */

#ifndef TOPOROBO_BH1750_HAL_H
#define TOPOROBO_BH1750_HAL_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

/* Constants ******************************************************************/

extern const uint8_t    bh1750_i2c_address;            /**< I2C address of the BH1750 sensor (default 0x23 when ADDR pin is GND). */
extern const i2c_port_t bh1750_i2c_bus;                /**< I2C bus number used by the ESP32 to communicate with the BH1750 sensor. */
extern const char      *bh1750_tag;                    /**< Tag for ESP_LOG messages related to the BH1750 sensor. */
extern const uint8_t    bh1750_scl_io;                 /**< GPIO pin for the I2C Serial Clock Line (SCL). */
extern const uint8_t    bh1750_sda_io;                 /**< GPIO pin for the I2C Serial Data Line (SDA). */
extern const uint32_t   bh1750_i2c_freq_hz;            /**< I2C bus frequency in Hz for BH1750 communication (default 100 kHz). */
extern const uint32_t   bh1750_polling_rate_ticks;     /**< Polling rate for the BH1750 sensor in system ticks. */
extern const uint8_t    bh1750_max_retries;            /**< Maximum retry attempts for BH1750 sensor reinitialization. */
extern const uint32_t   bh1750_initial_retry_interval; /**< Initial retry interval in ticks for BH1750 reinitialization. */
extern const uint32_t   bh1750_max_backoff_interval;   /**< Maximum backoff interval in ticks for BH1750 reinitialization retries. */

/* Enums **********************************************************************/

/**
 * @enum bh1750_commands_t
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
 * @enum bh1750_states_t
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
 * @struct bh1750_data_t
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

