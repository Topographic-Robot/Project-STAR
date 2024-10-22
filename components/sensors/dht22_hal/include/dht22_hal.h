#ifndef TOPOROBO_DHT22_HAL_H
#define TOPOROBO_DHT22_HAL_H

/* Digital Temperature and Humidity Sensor IC */
/* Uses a proprietary single-bus communication protocol with pulse width encoding */

/*******************************************************************************
 *
 *     +----------------------------------------+
 *     |                DHT22                   |
 *     |                                        |
 *     |   +----------------------------------+ |
 *     |   |                                  | |
 *     |   |   +--------------------------+   | |
 *     |   |   |        +----------+      |   | |
 *     |   |   | VCC    | 3.3V to 6V      |---------->| VCC
 *     |   |   +--------------------------+   | |
 *     |   |   | DATA   | Data Out        |---------->| GPIO_NUM_4
 *     |   |   +--------------------------+   | |
 *     |   |   | NC     | Not Connected   |   | |
 *     |   |   +--------------------------+   | |
 *     |   |   | GND    | Ground          |---------->| GND
 *     |   |   +--------------------------+   | |
 *     |   |                                  | |
 *     |   +----------------------------------+ |
 *     +----------------------------------------+
 *
 *     Block Diagram for Wiring
 *
 *     +----------------------------------------------------+
 *     |                    DHT22                           |
 *     |                                                    |
 *     |   +------------+     +-------------------+         |
 *     |   | Humidity   |---->| Signal Processing |         |
 *     |   | Sensor     |     | Unit              |         |
 *     |   +------------+     +-------------------+         |
 *     |                                                    |
 *     |   +------------+     +-------------------+         |
 *     |   | Temperature|---->| Signal Processing |         |
 *     |   | Sensor     |     | Unit              |         |
 *     |   +------------+     +-------------------+         |
 *     |                                                    |
 *     |   +------------------+                             |
 *     |   | 1-Wire Digital   |<----------------------------|
 *     |   | Communication    |                             |
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

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/* Constants ******************************************************************/

extern const char    *dht22_tag;                /* Tag for logging */
extern const uint8_t  dht22_data_io;            /* GPIO pin for DHT22 data line */
extern const uint32_t dht22_polling_rate_ticks; /* Polling rate (5 seconds) */
extern const uint8_t  dht22_bit_count;          /* Total number of bits from DHT22 */

/* Enums **********************************************************************/

/**
 * @enum dht22_states_t
 * @brief Enum to represent the state of the DHT22 sensor.
 *
 * This enum defines the possible states for the DHT22 sensor.
 */
typedef enum {
  k_dht22_ready         = 0x00, /**< Sensor is ready to read data. */
  k_dht22_data_updated  = 0x01, /**< Sensor data has been updated. */
  k_dht22_uninitialized = 0x10, /**< Sensor is not initialized. */
  k_dht22_error         = 0xF0, /**< A general catch all error */
} dht22_states_t;

/* Data Structures *************************************************************/

/**
 * @struct dht22_data_t
 * @brief Structure to store DHT22 sensor data.
 * 
 * Holds humidity and temperature values in Fahrenheit. The mutex ensures thread-safe 
 * access to the sensor data.
 */
typedef struct {
  float             temperature_f; /**< Temperature in Fahrenheit. */
  float             temperature_c; /**< Temperature in Celsius. */
  float             humidity;      /**< Humidity in percentage. */
  uint8_t           state;         /**< Sensor state, set in `dht22_states_t` enum. */
  SemaphoreHandle_t sensor_mutex;  /**< Mutex for protecting access to sensor data. */
} dht22_data_t;

/* Public Functions ************************************************************/

/**
 * @brief Initializes the DHT22 sensor and the data structure.
 * 
 * Sets up the GPIO pin and prepares the dht22_data_t structure for use. A mutex is
 * created to protect access to the sensor data during concurrent tasks.
 * 
 * @param[in,out] sensor_data Pointer to dht22_data_t structure to initialize.
 *
 * @param[in] first_time Boolean to let the function know if you have already
 *   called this before, its not recommended to run it as 'first_time' multiple
 *   times since memory allocation for a Semaphore occurs and can run into 
 *   memory issues if ran too much. Run it as 'first_time' once then set it to
 *   false when re-running the function.
 *
 *
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t dht22_init(dht22_data_t *sensor_data, bool first_time);

/**
 * @brief Reads data from the DHT22 sensor.
 * 
 * Reads humidity and temperature data from the DHT22 sensor and stores the values in
 * the provided dht22_data_t structure. The function uses a mutex to ensure thread-safe
 * access to the sensor data.
 * 
 * @param[in,out] sensor_data Pointer to dht22_data_t structure where data will be stored.
 */
void dht22_read(dht22_data_t *sensor_data);

/**
 * @brief Task to periodically read the DHT22 sensor data.
 * 
 * Runs in a loop to read humidity and temperature data from the DHT22 sensor every
 * 5 seconds. The data is stored in a dht22_data_t structure, with access protected
 * by a mutex.
 * 
 * @param sensor_data Pointer to the dht22_data_t structure for storing data.
 */
void dht22_tasks(void *sensor_data);

#endif /* TOPOROBO_DHT22_HAL_H */

