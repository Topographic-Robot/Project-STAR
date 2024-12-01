/* components/storage/sd_card_hal/include/sd_card_hal.h */

/* SD Card HAL (Hardware Abstraction Layer) Header File
 * This file provides the interface for interacting with an SD card module using SPI.
 * It includes the necessary functions for initializing and interacting with the SD card.
 *
 *******************************************************************************
 *
 *    +-----------------------+
 *    |    Micro SD Module    |
 *    |-----------------------|
 *    | 3.3V | Power          |----------> 3.3V
 *    | CS   | Chip Select    |----------> GPIO_NUM_5 (D5)
 *    | MOSI | Master Out     |----------> GPIO_NUM_23 (D23)
 *    | CLK  | Clock          |----------> GPIO_NUM_14 (D14)
 *    | MISO | Master In      |----------> GPIO_NUM_19 (D19)
 *    | GND  | Ground         |----------> GND
 *    +-----------------------+
 *
 *    Block Diagram for wiring
 *
 *    +----------------------------------------------------+
 *    |                  Micro SD Module                   |
 *    |                                                    |
 *    |   +-----------------+    +----------------------+  |
 *    |   | SD Card Slot    |--->| Data Storage         |  |
 *    |   |                 |    |                      |  |
 *    |   +-----------------+    +----------------------+  |
 *    |                                                    |
 *    |   +---------------------+    +------------------+  |
 *    |   | SPI Interface       |<-->| Data Transfer    |  |
 *    |   | (MOSI, MISO, CLK,   |    | Logic            |  |
 *    |   | CS)                 |    |                  |  |
 *    |   +---------------------+    +------------------+  |
 *    |                                                    |
 *    |   +---------------------+                          |
 *    |   | Level Shifter       |<-------------------------|
 *    |   | Circuit             |                          |
 *    |   +---------------------+                          |
 *    |                                                    |
 *    |   +---------------------+                          |
 *    |   | Power Supply Unit   |                          |
 *    |   | (PSU)               |                          |
 *    |   +---------------------+                          |
 *    +----------------------------------------------------+
 *
 *******************************************************************************/

#ifndef TOPOROBO_SD_CARD_HAL_H
#define TOPOROBO_SD_CARD_HAL_H

#include "esp_err.h"
#include "driver/spi_common.h"

/* Constants ******************************************************************/

/**
 * @brief GPIO pin used for the Chip Select (CS) signal of the SPI interface.
 *
 * This constant defines the GPIO pin connected to the CS signal of the SD card module.
 * Ensure this pin matches your hardware wiring.
 */
extern const uint8_t sd_card_cs_io;

/**
 * @brief GPIO pin used for the SPI MOSI signal.
 *
 * This constant defines the GPIO pin connected to the Master Out, Slave In (MOSI) signal
 * of the SPI interface.
 */
extern const uint8_t sd_card_mosi_io;

/**
 * @brief GPIO pin used for the SPI MISO signal.
 *
 * This constant defines the GPIO pin connected to the Master In, Slave Out (MISO) signal
 * of the SPI interface.
 */
extern const uint8_t sd_card_miso_io;

/**
 * @brief GPIO pin used for the SPI Clock (CLK) signal.
 *
 * This constant defines the GPIO pin connected to the clock signal of the SPI interface.
 */
extern const uint8_t sd_card_clk_io;

/**
 * @brief SPI bus frequency in Hertz for communication with the SD card module.
 *
 * This constant defines the frequency of the SPI bus for communication with the SD card module.
 * Standard frequencies, such as 1 MHz or higher, can be used depending on the hardware and requirements.
 */
extern const uint32_t sd_card_spi_freq_hz;

/**
 * @brief Default SPI host for the SD card interface.
 *
 * This constant defines the default SPI host (e.g., `SPI2_HOST` or `SPI3_HOST`) used
 * for the SD card interface. It can be configured at compile time to suit your hardware setup.
 */
extern const spi_host_device_t sd_card_spi_host;

/**
 * @brief Maximum number of files that can be opened simultaneously on the SD card.
 *
 * This variable defines the maximum number of files that can be open at once
 * in the FAT filesystem. It can be adjusted based on application requirements.
 */
extern const uint8_t sd_card_max_files;

/**
 * @brief Allocation unit size for the SD card filesystem in bytes.
 *
 * This variable defines the allocation unit size used when mounting the SD card.
 * The default is set to 16 KB. Adjust this value based on the expected file sizes
 * to optimize storage efficiency.
 */
extern const uint32_t sd_card_allocation_unit_size;

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the SD card for file operations.
 *
 * Mounts the SD card filesystem using FATFS. The card must be properly formatted
 * with a FAT filesystem for successful mounting. Uses the SPI host configured in
 * `sd_card_spi_host`.
 *
 * @return
 * - `ESP_OK` if the initialization is successful.
 * - `ESP_FAIL` if mounting fails.
 */
esp_err_t sd_card_init(void);

#endif /* TOPOROBO_SD_CARD_HAL_H */

