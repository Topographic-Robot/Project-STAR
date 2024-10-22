#ifndef TOPOROBO_GY_NEO6MV2_HAL_H
#define TOPOROBO_GY_NEO6MV2_HAL_H

/* GY-NEO6MV2 GPS Module */
/* Communicates over UART protocol */

/*******************************************************************************
 *
 *     +----------------------------------------+
 *     |              GY-NEO6MV2                |
 *     |                                        |
 *     |   +----------------------------------+ |
 *     |   |                                  | |
 *     |   |   +--------------------------+   | |
 *     |   |   |        +-----------------+   | |
 *     |   |   | VCC    | 3.3V or 5V      |----------->| VCC
 *     |   |   +--------------------------+   | |
 *     |   |   | GND    | Ground          |----------->| GND
 *     |   |   +--------------------------+   | |
 *     |   |   | TXD    | UART TX         |----------->| GPIO_NUM_17
 *     |   |   +--------------------------+   | |
 *     |   |   | RXD    | UART RX         |----------->| GPIO_NUM_16
 *     |   |   +--------------------------+   | |
 *     |   |                                  | |
 *     |   +----------------------------------+ |
 *     +----------------------------------------+
 *
 *     Block Diagram for Wiring
 *
 *     +----------------------------------------------------+
 *     |                  GY-NEO6MV2 GPS                    |
 *     |                                                    |
 *     |   +----------------+    +-------------------+      |
 *     |   | GPS Receiver   |--->| Signal Processing |      |
 *     |   | Antenna        |    | Unit              |      |
 *     |   +----------------+    +-------------------+      |
 *     |                                                    |
 *     |   +------------------+                             |
 *     |   | UART Interface   |<----------------------------|
 *     |   | (TX, RX)         |                             |
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


#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/* Constants ******************************************************************/

extern const char    *gy_neo6mv2_tag;             /* Tag for logs */
extern const uint8_t  gy_neo6mv2_tx_io;           /* GPIO pin for UART TX */
extern const uint8_t  gy_neo6mv2_rx_io;           /* GPIO pin for UART RX */
extern const uint32_t gy_neo6mv2_uart_baudrate;   /* UART baud rate */

/* Structs ********************************************************************/

/**
 * @struct gy_neo6mv2_data_t
 * @brief Structure to store GPS data read from the GY-NEO6MV2 module.
 *
 * This structure holds the data such as latitude, longitude, altitude, speed, and time.
 */
typedef struct {
  float             latitude;   /**< GPS latitude in degrees */
  float             longitude;  /**< GPS longitude in degrees */
  float             altitude;   /**< GPS altitude in meters */
  float             speed;      /**< Speed in meters per second */
  char              time[11];   /**< UTC Time in HHMMSS format */
  uint8_t           fix_status; /**< GPS fix status (0: no fix, 1: 2D fix, 2: 3D fix) */
  SemaphoreHandle_t data_mutex; /**< Mutex for protecting access to GPS data */
} gy_neo6mv2_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initialize the GY-NEO6MV2 GPS module over UART.
 *
 * This function sets up the UART connection for communication with the GPS module.
 *
 * @param[in,out] gps_data Pointer to the `gy_neo6mv2_data_t` structure that will hold the GPS data.
 * @param[in] first_time Boolean to indicate if the function is being called for the first time.
 *
 * @return
 *   - ESP_OK on success.
 *   - An error code from the `esp_err_t` enumeration on failure.
 */
esp_err_t gy_neo6mv2_init(gy_neo6mv2_data_t *gps_data, bool first_time);

/**
 * @brief Reads GPS data from the GY-NEO6MV2 GPS module.
 *
 * This function reads data from the GPS module and updates the GPS data structure.
 *
 * @param[in,out] gps_data Pointer to a `gy_neo6mv2_data_t` struct containing GPS data.
 */
void gy_neo6mv2_read(gy_neo6mv2_data_t *gps_data);

#endif /* TOPOROBO_GY_NEO6MV2_HAL_H */

