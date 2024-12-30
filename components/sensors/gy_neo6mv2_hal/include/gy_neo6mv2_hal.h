/* components/sensors/gy_neo6mv2_hal/include/gy_neo6mv2_hal.h */

#ifndef TOPOROBO_GY_NEO6MV2_HAL_H
#define TOPOROBO_GY_NEO6MV2_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "driver/uart.h"

/* Constants ******************************************************************/

extern const char       *gy_neo6mv2_tag;                    /**< Logging tag for ESP_LOG messages related to the GY-NEO6MV2 module. */
extern const uint8_t     gy_neo6mv2_tx_io;                  /**< GPIO pin for UART TX line to the GY-NEO6MV2 module. */
extern const uint8_t     gy_neo6mv2_rx_io;                  /**< GPIO pin for UART RX line from the GY-NEO6MV2 module. */
extern const uart_port_t gy_neo6mv2_uart_num;               /**< UART number used for GY-NEO6MV2 communication. */
extern const uint32_t    gy_neo6mv2_uart_baudrate;          /**< UART baud rate for GY-NEO6MV2 communication (default 9600). */
extern const uint32_t    gy_neo6mv2_polling_rate_ticks;     /**< Polling interval for GY-NEO6MV2 in system ticks. */
extern const uint8_t     gy_neo6mv2_max_retries;            /**< Maximum retry attempts for GY-NEO6MV2 reinitialization. */
extern const uint32_t    gy_neo6mv2_initial_retry_interval; /**< Initial retry interval for GY-NEO6MV2 in system ticks. */
extern const uint32_t    gy_neo6mv2_max_backoff_interval;   /**< Maximum backoff interval for GY-NEO6MV2 retries in ticks. */

/* Macros *********************************************************************/

#define gy_neo6mv2_sentence_buffer_size (128) /**< Maximum buffer size for NMEA sentences from the GY-NEO6MV2 module. */
#define gy_neo6mv2_max_satellites       (32)  /**< Maximum number of satellites' data to store in the buffer. */

/* Enums **********************************************************************/

/**
 * @brief Enumeration of GY-NEO6MV2 GPS module states.
 *
 * Represents the possible operational states of the GY-NEO6MV2 GPS module,
 * including normal operation, data updates, initialization status, and error conditions.
 */
typedef enum : uint8_t {
  k_gy_neo6mv2_ready         = 0x00, /**< Module is initialized and ready for operation. */
  k_gy_neo6mv2_data_updated  = 0x01, /**< New GPS data has been successfully retrieved. */
  k_gy_neo6mv2_uninitialized = 0x10, /**< Module is not initialized. */
  k_gy_neo6mv2_error         = 0xF0, /**< General catch-all error state. */
} gy_neo6mv2_states_t;

/* Structs ********************************************************************/

/**
 * @brief Structure to store GPS data from the GY-NEO6MV2 module.
 *
 * Contains GPS data such as latitude, longitude, speed, and UTC time, along with
 * diagnostic information like fix status, satellite count, horizontal dilution
 * of precision (HDOP), and retry management fields for error handling.
 */
typedef struct {
  float               latitude;           /**< Latitude in decimal degrees. Negative values indicate South. */
  float               longitude;          /**< Longitude in decimal degrees. Negative values indicate West. */
  float               speed;              /**< Speed over ground in meters per second. */
  char                time[11];           /**< UTC time in HHMMSS.SS format. */
  uint8_t             fix_status;         /**< GPS fix status (0: no fix, 1: fix acquired). */
  uint8_t             satellite_count;    /**< Number of satellites used in the solution. */
  float               hdop;               /**< Horizontal Dilution of Precision (accuracy; lower values are better). */
  gy_neo6mv2_states_t state;              /**< Current operational state of the GPS module. */
  uint8_t             retry_count;        /**< Number of consecutive reinitialization attempts. */
  uint32_t            retry_interval;     /**< Current interval between retry attempts, in ticks. */
  TickType_t          last_attempt_ticks; /**< Tick count of the last reinitialization attempt. */
} gy_neo6mv2_data_t;

/**
 * @brief Structure to store satellite information from GPGSV sentences.
 *
 * Holds details about a satellite, including its identifier (PRN), elevation,
 * azimuth, and signal strength (SNR), as parsed from GPGSV NMEA sentences.
 */
typedef struct {
  uint8_t  prn;       /**< Satellite ID (PRN - Pseudo Random Noise code). */
  uint8_t  elevation; /**< Satellite elevation angle in degrees above the horizon. */
  uint16_t azimuth;   /**< Satellite azimuth angle in degrees from true north. */
  uint8_t  snr;       /**< Signal-to-Noise Ratio (SNR), ranging from 0 to 99. */
} satellite_t;

/* Public Functions ***********************************************************/

/**
 * @brief Convert GY-NEO6MV2 data to JSON.
 *
 * This function formats the GPS data stored in a `gy_neo6mv2_data_t` structure into a JSON string.
 * The JSON includes the sensor type, latitude, longitude, speed, and time.
 *
 * @param[in] data Pointer to `gy_neo6mv2_data_t` structure containing GPS data.
 *
 * @return Pointer to a dynamically allocated JSON string. The caller is responsible for freeing the memory.
 */
char *gy_neo6mv2_data_to_json(const gy_neo6mv2_data_t *data);

/**
 * @brief Initializes the GY-NEO6MV2 GPS module over UART.
 *
 * This function sets up the UART connection for communication with the GY-NEO6MV2 GPS module.
 * It configures the UART with the specified baud rate and establishes a connection between
 * the ESP32 and the GPS module. Upon successful initialization, the `gy_neo6mv2_data_t`
 * structure is prepared to receive GPS data.
 *
 * @param[in,out] sensor_data Pointer to the `gy_neo6mv2_data_t` structure to be initialized.
 *                            - Initializes GPS data fields and state.
 *
 * @return
 * - `ESP_OK` on successful initialization.
 * - An error code from the `esp_err_t` enumeration on failure.
 *
 * @note Call this function once during setup to initialize the GPS module for data acquisition.
 */
esp_err_t gy_neo6mv2_init(void *sensor_data);

/**
 * @brief Reads GPS data from the GY-NEO6MV2 GPS module.
 *
 * This function reads data from the GPS module over UART, processes NMEA sentences,
 * and updates the `gy_neo6mv2_data_t` structure with the latest GPS information.
 *
 * @param[in,out] sensor_data Pointer to `gy_neo6mv2_data_t` structure:
 *                            - `latitude`, `longitude`, `speed`, `time`, and `fix_status` are
 *                              updated with the latest GPS data (output).
 *                            - `state` is set to indicate a successful read or error (output).
 *
 * @return
 * - `ESP_OK` on successful read.
 * - `ESP_FAIL` on unsuccessful read.
 *
 * @note Ensure the GPS module is initialized using `gy_neo6mv2_init` before calling this function.
 */
esp_err_t gy_neo6mv2_read(gy_neo6mv2_data_t *sensor_data);

/**
 * @brief Manages error detection and recovery for the GY-NEO6MV2 GPS module using exponential backoff.
 *
 * This function checks the operational state of the GPS module and attempts to reinitialize it
 * if an error is detected. It uses an exponential backoff strategy to manage the retry intervals,
 * preventing excessive retries and allowing the module time to recover.
 *
 * @param[in,out] sensor_data Pointer to `gy_neo6mv2_data_t` structure:
 *                            - `state`: Current operational state (input/output).
 *                            - `retry_count`: Counter tracking reinitialization attempts (input/output).
 *                            - `retry_interval`: Current retry interval in ticks (input/output).
 *                            - `last_attempt_ticks`: Time of the last reinitialization attempt (input/output).
 *
 * @note This function should be periodically called within `gy_neo6mv2_tasks` to handle GPS errors and manage retries.
 */
void gy_neo6mv2_reset_on_error(gy_neo6mv2_data_t *sensor_data);

/**
 * @brief Periodically reads data from the GY-NEO6MV2 GPS module and manages errors.
 *
 * This function runs in a loop, continuously reading GPS data at intervals defined by
 * `gy_neo6mv2_polling_rate_ticks`. It handles errors by calling `gy_neo6mv2_reset_on_error`
 * to manage retries with an exponential backoff strategy. The function is designed to run
 * as a FreeRTOS task.
 *
 * @param[in,out] sensor_data Pointer to `gy_neo6mv2_data_t` structure for:
 *                            - GPS data fields (output).
 *                            - Retry-related fields for error handling (input/output).
 *
 * @note Run this function as part of a FreeRTOS task to ensure continuous GPS data acquisition and error management.
 */
void gy_neo6mv2_tasks(void *sensor_data);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_GY_NEO6MV2_HAL_H */

