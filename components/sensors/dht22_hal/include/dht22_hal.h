/* components/sensors/dht22_hal/include/dht22_hal.h */

#ifndef TOPOROBO_DHT22_HAL_H
#define TOPOROBO_DHT22_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Constants ******************************************************************/

extern const char    *dht22_tag;                    /**< Logging tag for ESP_LOG messages related to the DHT22 sensor. */
extern const uint8_t  dht22_data_io;                /**< GPIO pin number for the DHT22 data line. */
extern const uint32_t dht22_polling_rate_ticks;     /**< Polling interval for DHT22 in system ticks. */
extern const uint8_t  dht22_bit_count;              /**< Total number of bits transmitted by the DHT22 sensor (40 bits). */
extern const uint8_t  dht22_max_retries;            /**< Maximum retry attempts for DHT22 reinitialization. */
extern const uint32_t dht22_initial_retry_interval; /**< Initial retry interval for DHT22 in system ticks. */
extern const uint32_t dht22_max_backoff_interval;   /**< Maximum backoff interval for DHT22 retries in system ticks. */
extern const uint32_t dht22_start_delay_ms;         /**< Start signal delay for DHT22 in milliseconds. */
extern const uint32_t dht22_response_timeout_us;    /**< Timeout for DHT22 sensor response in microseconds. */
extern const uint32_t dht22_bit_threshold_us;       /**< Timing threshold for distinguishing bits in DHT22 signal. */

/* Enums **********************************************************************/

/**
 * @brief Enumeration of DHT22 sensor states.
 *
 * Represents the possible operational states of the DHT22 temperature and humidity sensor,
 * including normal operation, data updates, initialization status, and error conditions.
 */
typedef enum : uint8_t {
  k_dht22_ready         = 0x00, /**< Sensor is initialized and ready to provide data. */
  k_dht22_data_updated  = 0x01, /**< New data is available from the sensor. */
  k_dht22_uninitialized = 0x10, /**< Sensor has not been initialized. */
  k_dht22_error         = 0xF0, /**< General catch-all error state. */
} dht22_states_t;

/* Structs ********************************************************************/

/**
 * @brief Structure for managing DHT22 sensor data and state.
 *
 * Contains the latest temperature and humidity readings from the DHT22 sensor,
 * as well as state and retry management variables for error handling and reinitialization.
 */
typedef struct {
  float      temperature_f;      /**< Latest temperature reading in Fahrenheit. */
  float      temperature_c;      /**< Latest temperature reading in Celsius. */
  float      humidity;           /**< Latest humidity reading as a percentage. */
  uint8_t    state;              /**< Current operational state of the sensor (see dht22_states_t). */
  uint8_t    retry_count;        /**< Number of consecutive reinitialization attempts. */
  uint32_t   retry_interval;     /**< Current interval between reinitialization attempts, in ticks. */
  TickType_t last_attempt_ticks; /**< Tick count of the last reinitialization attempt. */
} dht22_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Convert DHT22 data to JSON.
 *
 * @param[in] sensor_data Pointer to `dht22_data_t` structure
 */
char *dht22_data_to_json(const dht22_data_t *data);

/**
 * @brief Initializes the DHT22 sensor for temperature and humidity measurement.
 *
 * The `dht22_init` function sets up the GPIO pin connected to the data line
 * of the DHT22 sensor and initializes the provided `dht22_data_t` structure to
 * store sensor readings. During this process, the sensor's state is updated
 * to indicate readiness for data acquisition.
 *
 * @param[in,out] sensor_data Pointer to `dht22_data_t` structure to be initialized.
 *                            - `temperature_f`: Placeholder for temperature in Fahrenheit (output).
 *                            - `temperature_c`: Placeholder for temperature in Celsius (output).
 *                            - `humidity`: Placeholder for humidity in percentage (output).
 *                            - `state`: Set to `k_dht22_ready` upon successful initialization.
 *
 * @return
 * - `ESP_OK` on successful initialization.
 * - An `esp_err_t` error code if initialization fails.
 *
 * @note This function must be called before attempting to read data from the sensor.
 */
esp_err_t dht22_init(void *sensor_data);

/**
 * @brief Reads temperature and humidity data from the DHT22 sensor.
 *
 * This function retrieves temperature and humidity readings from the DHT22 sensor.
 * If the read operation succeeds, the provided `dht22_data_t` structure is updated
 * with the new data. If it fails, the `state` is updated to indicate an error.
 *
 * @param[in,out] sensor_data Pointer to `dht22_data_t` structure where data is stored:
 *                            - `temperature_f`: Updated with temperature in Fahrenheit (output).
 *                            - `temperature_c`: Updated with temperature in Celsius (output).
 *                            - `humidity`: Updated with humidity percentage (output).
 *                            - `state`: Updated with the new sensor state (output).
 *
 * @return
 * - `ESP_OK` on successful read.
 * - `ESP_FAIL` on unsuccessful read.
 *
 * @note Ensure the sensor is initialized using `dht22_init` before calling this function.
 */
esp_err_t dht22_read(dht22_data_t *sensor_data);

/**
 * @brief Manages error detection and recovery for the DHT22 sensor using exponential backoff.
 *
 * The `dht22_reset_on_error` function attempts to reinitialize the DHT22 sensor
 * upon detecting an error in the sensor's state. The reinitialization process
 * uses an exponential backoff strategy to reduce retry frequency over time,
 * thus avoiding unnecessary load on the system. Upon successful reinitialization,
 * the retry count and interval are reset to their initial values.
 *
 * **Logic and Flow:**
 * - Checks if an error is detected (`state` contains `k_dht22_error`).
 * - Verifies if enough time has elapsed since the last reinitialization attempt.
 * - Attempts to reinitialize the sensor if the retry interval has elapsed.
 * - Resets the retry count and interval if reinitialization succeeds.
 * - Increments retry count and doubles the retry interval upon failure, up to
 *   a maximum defined by `dht22_max_backoff_interval`.
 *
 * @param[in,out] sensor_data Pointer to `dht22_data_t` structure containing:
 *                            - `state`: Current sensor state (input/output).
 *                            - `retry_count`: Counter tracking retry attempts (input/output).
 *                            - `retry_interval`: Current interval for retries (input/output).
 *                            - `last_attempt_ticks`: Tick count of last reinitialization attempt (input/output).
 *
 * @note This function is intended to be periodically called within the sensor task to handle errors and manage retries.
 */
void dht22_reset_on_error(dht22_data_t *sensor_data);

/**
 * @brief Periodically reads data from the DHT22 sensor and manages error handling.
 *
 * The `dht22_tasks` function is designed to be called in a loop within a FreeRTOS
 * task to continuously read data from the DHT22 sensor and handle any errors.
 * Between reads, it waits for the interval specified by `dht22_polling_rate_ticks`,
 * and it calls `dht22_reset_on_error` to manage errors using an exponential backoff strategy.
 *
 * @param[in,out] sensor_data Pointer to `dht22_data_t` structure for:
 *                            - `temperature_f`, `temperature_c`, `humidity`: Updated sensor data (output).
 *                            - `state`, `retry_count`, `retry_interval`: Managed sensor state and retry parameters (input/output).
 *
 * @note This function should be executed as part of a FreeRTOS task to ensure continuous
 *       data acquisition and error management for the sensor.
 */
void dht22_tasks(void *sensor_data);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_DHT22_HAL_H */

