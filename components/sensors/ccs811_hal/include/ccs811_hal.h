/* components/sensors/ccs811_hal/include/ccs811_hal.h */

#ifndef TOPOROBO_CCS811_HAL_H
#define TOPOROBO_CCS811_HAL_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

/* Constants ******************************************************************/

extern const uint8_t    ccs811_i2c_address; /**< Default I2C address for the CCS811 sensor (default 0x5A). */
extern const i2c_port_t ccs811_i2c_bus;     /**< I2C bus number used for communication. */
extern const char      *ccs811_tag;         /**< Tag used for logging messages related to the CCS811 sensor. */
extern const uint8_t    ccs811_scl_io;      /**< GPIO pin for I2C clock line (SCL). */
extern const uint8_t    ccs811_sda_io;      /**< GPIO pin for I2C data line (SDA). */
extern const uint8_t    ccs811_wake_io;     /**< GPIO pin for the sensor's wake-up function. */
extern const uint8_t    ccs811_rst_io;      /**< GPIO pin for the sensor's reset function. */
extern const uint8_t    ccs811_int_io;      /**< GPIO pin for the sensor's interrupt function (optional). */

/* Enums **********************************************************************/

/**
 * @enum ccs811_states_t
 * @brief Enumeration of CCS811 sensor states.
 *
 * This enum defines the possible states of the CCS811 sensor, including ready,
 * data updated, uninitialized, and various error states.
 */
typedef enum : uint8_t {
  k_ccs811_ready           = 0x00, /**< Sensor is ready for data */
  k_ccs811_data_updated    = 0x01, /**< Data has been updated */
  k_ccs811_uninitialized   = 0x10, /**< Sensor is not initialized */
  k_ccs811_error           = 0xF0, /**< General error */
  k_ccs811_power_on_error  = 0xA1, /**< Error during power on */
  k_ccs811_reset_error     = 0xA2, /**< Error during reset */
  k_ccs811_app_start_error = 0xA3, /**< Error starting application */
  k_ccs811_read_error      = 0xA4, /**< Error reading sensor data */
} ccs811_states_t;

/* Structs ********************************************************************/

/**
 * @struct ccs811_data_t
 * @brief Structure for storing CCS811 sensor data and status.
 *
 * This structure encapsulates all relevant data and state information for the
 * CCS811 sensor, including the most recent measurements and retry handling
 * variables for error recovery.
 */
typedef struct {
  uint8_t    i2c_address;        /**< I2C address */
  uint8_t    i2c_bus;            /**< I2C bus */
  uint16_t   eco2;               /**< Equivalent CO2 level in ppm */
  uint16_t   tvoc;               /**< Total Volatile Organic Compounds in ppb */
  uint8_t    state;              /**< Current state of the sensor */
  uint8_t    retry_count;        /**< Retry counter for error handling */
  uint32_t   retry_interval;     /**< Current retry interval */
  TickType_t last_attempt_ticks; /**< Time of last reinitialization attempt */
} ccs811_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Convert CCS811 sensor data to a JSON string.
 *
 * @param[in] data Pointer to the `ccs811_data_t` structure containing the
 *                 latest sensor readings.
 * @return A dynamically allocated JSON string representing the sensor data.
 *         The caller must free the memory using `free()`.
 */
char *ccs811_data_to_json(const ccs811_data_t *data);

/**
 * @brief Initialize the CCS811 sensor.
 *
 * This function configures the I2C interface and initializes the CCS811
 * sensor for continuous air quality monitoring.
 *
 * @param[in,out] sensor_data Pointer to the `ccs811_data_t` structure where
 *                            sensor-specific data and state will be stored.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t ccs811_init(void *sensor_data);

/**
 * @brief Read eCO2 and TVOC data from the CCS811 sensor.
 *
 * @param[in,out] sensor_data Pointer to the `ccs811_data_t` structure where
 *                            the latest sensor data will be stored.
 * @return ESP_OK on success, or ESP_FAIL on failure.
 */
esp_err_t ccs811_read(ccs811_data_t *sensor_data);

/**
 * @brief Handle reinitialization and error recovery for the CCS811 sensor.
 *
 * This function implements exponential backoff for retries when the sensor
 * encounters an error.
 *
 * @param[in,out] sensor_data Pointer to the `ccs811_data_t` structure.
 */
void ccs811_reset_on_error(ccs811_data_t *sensor_data);

/**
 * @brief Execute periodic tasks for the CCS811 sensor.
 *
 * This function reads data from the sensor at regular intervals and sends it
 * to a web server. It also handles error recovery.
 *
 * @param[in,out] sensor_data Pointer to the `ccs811_data_t` structure.
 */
void ccs811_tasks(void *sensor_data);

#endif /* TOPOROBO_CCS811_HAL_H */

