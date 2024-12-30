/* components/sensors/mq135_hal/include/mq135_hal.h */

#ifndef TOPOROBO_MQ135_HAL_H
#define TOPOROBO_MQ135_HAL_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Constants ******************************************************************/

extern const uint8_t  mq135_aout_pin;               /**< GPIO pin for analog output (AOUT) of the MQ135 sensor. */
extern const uint8_t  mq135_dout_pin;               /**< GPIO pin for digital output (DOUT) of the MQ135 sensor. */
extern const uint32_t mq135_polling_rate_ticks;     /**< Polling rate for MQ135 sensor reads in system ticks. */
extern const uint32_t mq135_warmup_time_ms;         /**< Warm-up time for MQ135 sensor in milliseconds. */
extern const uint8_t  mq135_max_retries;            /**< Maximum retry attempts for MQ135 error recovery. */
extern const uint32_t mq135_initial_retry_interval; /**< Initial retry interval for MQ135 error recovery in ticks. */
extern const uint32_t mq135_max_backoff_interval;   /**< Maximum backoff interval for MQ135 retries in ticks. */

/* Enums **********************************************************************/

/**
 * @enum mq135_states_
 * @brief Enumeration of MQ135 sensor states.
 *
 * This enum defines the possible operational states of the MQ135 sensor.
 * Each state indicates a specific phase in the sensor's lifecycle or an error
 * condition that requires handling.
 *
 * **States:**
 * - `k_mq135_ready`: The sensor is ready for data collection.
 * - `k_mq135_warming_up`: The sensor is still stabilizing during the warm-up period.
 * - `k_mq135_error`: A general error has occurred.
 * - `k_mq135_read_error`: An error occurred while reading the analog output.
 */
typedef enum : uint8_t {
  k_mq135_ready      = 0x00, /**< Sensor is ready for data collection */
  k_mq135_warming_up = 0x01, /**< Sensor is stabilizing (warm-up period) */
  k_mq135_error      = 0xF0, /**< General error */
  k_mq135_read_error = 0xA1, /**< Error reading analog value */
} mq135_states_t;

/* Structs ********************************************************************/

/**
 * @struct mq135_data_
 * @brief Structure to store MQ135 sensor data and status.
 *
 * This structure encapsulates all relevant data and status information for
 * the MQ135 sensor, including the raw ADC value, calculated gas concentration,
 * retry handling, and the current state.
 */
typedef struct {
  uint16_t   raw_adc_value;      /**< Raw ADC value from the analog output */
  float      gas_concentration;  /**< Gas concentration in ppm (calculated) */
  uint8_t    state;              /**< Current state of the sensor */
  TickType_t warmup_start_ticks; /**< Tick count of warm-up start */
  uint8_t    retry_count;        /**< Retry counter for error recovery */
  uint32_t   retry_interval;     /**< Current retry interval */
} mq135_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Convert MQ135 sensor data to a JSON string.
 *
 * This function generates a JSON representation of the MQ135 sensor's
 * current data, including the calculated gas concentration in ppm.
 *
 * @param[in] data Pointer to the `mq135_data_t` structure containing the
 *                 latest sensor readings.
 * @return A dynamically allocated JSON string representing the sensor data.
 *         The caller must free the memory using `free()`.
 */
char *mq135_data_to_json(const mq135_data_t *data);

/**
 * @brief Initialize the MQ135 sensor.
 *
 * This function configures the GPIO and ADC settings for the MQ135 sensor and
 * starts the warm-up period. During the warm-up period, the sensor is not
 * considered ready for data collection.
 *
 * @param[in,out] sensor_data Pointer to the `mq135_data_t` structure where
 *                            sensor-specific data and state will be stored.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mq135_init(void *sensor_data);

/**
 * @brief Read gas concentration data from the MQ135 sensor.
 *
 * This function reads the raw analog value from the sensor's AOUT pin and
 * calculates the gas concentration in ppm. If the warm-up period has not
 * completed, the function returns an error.
 *
 * @param[in,out] sensor_data Pointer to the `mq135_data_t` structure where
 *                            the latest sensor data will be stored.
 * @return ESP_OK on success, or ESP_FAIL if the sensor is still warming up
 *         or if an error occurs during data collection.
 */
esp_err_t mq135_read(mq135_data_t *sensor_data);

/**
 * @brief Reset the MQ135 sensor on error.
 *
 * This function handles error recovery for the MQ135 sensor, including
 * reinitialization and retry attempts with exponential backoff. If the number
 * of retries exceeds the maximum allowed, the retry interval is increased.
 *
 * @param[in,out] sensor_data Pointer to the `mq135_data_t` structure.
 */
void mq135_reset_on_error(mq135_data_t *sensor_data);

/**
 * @brief Execute periodic tasks for the MQ135 sensor.
 *
 * This function performs regular data collection from the MQ135 sensor,
 * including handling the warm-up period and sending the data to a web server
 * in JSON format. It also manages error recovery using `mq135_reset_on_error`.
 *
 * @param[in,out] sensor_data Pointer to the `mq135_data_t` structure.
 */
void mq135_tasks(void *sensor_data);

#endif /* TOPOROBO_MQ135_HAL_H */

