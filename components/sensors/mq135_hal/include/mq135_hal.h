/* components/sensors/mq135_hal/include/mq135_hal.h */

/* MQ135 HAL (Hardware Abstraction Layer) Header File
 *
 * This file provides the interface for interacting with the MQ135 gas sensor.
 * The MQ135 is an air quality sensor capable of detecting a variety of gases,
 * including CO2, NH3, NOx, alcohol, benzene, and smoke. It outputs an analog
 * signal proportional to the gas concentration, which is read by the ESP32's
 * ADC (Analog-to-Digital Converter).
 *
 * The sensor requires a warm-up period for its heating element to stabilize.
 * Accurate measurements depend on proper calibration for the target application
 * and environmental conditions.
 *
 *******************************************************************************
 *
 *    +-----------------------+
 *    |        MQ135          |
 *    |-----------------------|
 *    | VCC  | 5V             |----------> VCC
 *    | GND  | Ground         |----------> GND
 *    | AOUT | Analog Output  |----------> GPIO_NUM_34 (ADC Input)
 *    | DOUT | Digital Output |----------> Optional GPIO
 *    +-----------------------+
 *
 *    Block Diagram for Wiring
 *
 *    +----------------------------------------------------+
 *    |                       MQ135                        |
 *    |                                                    |
 *    |   +-------------------+                            |
 *    |   | Gas Sensor        |                            |
 *    |   | (Heater and       |                            |
 *    |   | Sensing Element)  |                            |
 *    |   +-------------------+                            |
 *    |                                                    |
 *    |   +-------------------+     +------------------+   |
 *    |   | Signal Conditioning |-->| Analog Output    |   |
 *    |   | Circuit             |    |                 |   |
 *    |   +---------------------+    +-----------------+   |
 *    |                                                    |
 *    |   +---------------------+                          |
 *    |   | Power Supply Unit   |                          |
 *    |   | (PSU)               |                          |
 *    |   +---------------------+                          |
 *    +----------------------------------------------------+
 *
 *******************************************************************************/

#ifndef TOPOROBO_MQ135_HAL_H
#define TOPOROBO_MQ135_HAL_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Constants ******************************************************************/

/**
 * @brief GPIO pin for the analog output (AOUT) of the MQ135 sensor.
 *
 * This pin connects to the ADC (Analog-to-Digital Converter) of the ESP32,
 * which reads the analog voltage output by the sensor. The voltage represents
 * the gas concentration.
 */
extern const uint8_t mq135_aout_pin;

/**
 * @brief GPIO pin for the digital output (DOUT) of the MQ135 sensor.
 *
 * The digital output is a binary signal that can be used to trigger alerts
 * when the gas concentration exceeds a predefined threshold. This pin is
 * optional and can be ignored if only the analog output is used.
 */
extern const uint8_t mq135_dout_pin;

/**
 * @brief Polling rate for reading data from the MQ135 sensor.
 *
 * This constant defines the interval at which the ESP32 polls the MQ135 sensor
 * for new data, in FreeRTOS ticks. The polling rate should allow sufficien
 * time for the sensor to stabilize between readings.
 */
extern const uint32_t mq135_polling_rate_ticks;

/**
 * @brief Warm-up time in milliseconds for the MQ135 sensor.
 *
 * The MQ135 requires a stabilization period after power-up for its heating
 * element to reach the optimal operating temperature. Accurate measurements
 * cannot be taken during this warm-up period.
 */
extern const uint32_t mq135_warmup_time_ms;

/**
 * @brief Maximum number of retry attempts for error recovery.
 *
 * This constant defines the maximum number of retries the system will perform
 * to recover from sensor read or initialization errors before increasing the
 * retry interval using exponential backoff.
 */
extern const uint8_t mq135_max_retries;

/**
 * @brief Initial interval between retries in milliseconds, converted to ticks.
 *
 * This constant defines the initial retry interval for error recovery. The
 * interval doubles after each set of retries until it reaches the maximum
 * backoff interval.
 */
extern const uint32_t mq135_initial_retry_interval;

/**
 * @brief Maximum interval for exponential backoff between retries in ticks.
 *
 * This constant sets the upper limit for the retry interval during exponential
 * backoff. Once this limit is reached, the interval will no longer increase.
 */
extern const uint32_t mq135_max_backoff_interval;

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
 * starts the warm-up period. During the warm-up period, the sensor is no
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
 * calculates the gas concentration in ppm. If the warm-up period has no
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

