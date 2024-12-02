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
 * This constant specifies the GPIO pin number connected to the Chip Select (CS)
 * line of the SD card module. The CS pin is used to enable or disable the SD card
 * for SPI communication, ensuring proper coordination when multiple devices share
 * the same SPI bus. Ensure this pin matches the physical wiring between the SD card
 * module and the ESP32.
 */
extern const uint8_t sd_card_cs_io;

/**
 * @brief GPIO pin used for the SPI Master Out, Slave In (MOSI) signal.
 *
 * This constant defines the GPIO pin number connected to the Master Out, Slave In
 * (MOSI) line of the SPI interface. The MOSI line is used by the ESP32 to send data
 * to the SD card module during SPI communication. Ensure this pin matches your
 * hardware setup for proper data transmission.
 */
extern const uint8_t sd_card_mosi_io;

/**
 * @brief GPIO pin used for the SPI Master In, Slave Out (MISO) signal.
 *
 * This constant specifies the GPIO pin number connected to the Master In, Slave Out
 * (MISO) line of the SPI interface. The MISO line is used by the SD card module to
 * send data back to the ESP32 during SPI communication. Ensure this pin is correctly
 * wired for reliable data reception.
 */
extern const uint8_t sd_card_miso_io;

/**
 * @brief GPIO pin used for the SPI Clock (CLK) signal.
 *
 * This constant represents the GPIO pin number connected to the Clock (CLK) line of
 * the SPI interface. The CLK line provides the timing reference for SPI communication
 * between the ESP32 and the SD card module. Ensure this pin is correctly connected to
 * avoid communication issues caused by clock signal errors.
 */
extern const uint8_t sd_card_clk_io;

/**
 * @brief SPI bus frequency in Hertz for communication with the SD card module.
 *
 * This constant defines the frequency of the SPI clock line used for communication
 * with the SD card module. Standard values, such as 1 MHz or higher, can be used based
 * on the speed and reliability requirements of your application. Higher frequencies
 * enable faster data transfer but may require higher-quality wiring and a well-configured
 * SPI interface to prevent signal integrity issues.
 */
extern const uint32_t sd_card_spi_freq_hz;

/**
 * @brief Default SPI host for the SD card interface.
 *
 * This constant specifies the default SPI host used for communication with the SD card.
 * Typical values include `SPI2_HOST` (HSPI) or `SPI3_HOST` (VSPI). The host determines
 * which SPI controller on the ESP32 is used to manage communication. This setting should
 * match the configuration of the SPI peripheral in your project and be chosen based on
 * the available SPI hardware on your ESP32 board.
 */
extern const spi_host_device_t sd_card_spi_host;

/**
 * @brief Maximum number of files that can be opened simultaneously on the SD card.
 *
 * This constant specifies the maximum number of files that can be opened at the same
 * time within the FAT filesystem on the SD card. Increasing this value allows for
 * greater concurrency in file operations but consumes more system resources, such as
 * memory for file descriptors. Adjust this setting to match the needs of your application.
 */
extern const uint8_t sd_card_max_files;

/**
 * @brief Allocation unit size for the SD card filesystem in bytes.
 *
 * This setting specifies the cluster size (allocation unit size) to use when formatting
 * the SD card. It is only relevant if the `format_if_mount_failed` option is set to `true`
 * and the SD card is reformatted during mounting. If the SD card is already formatted
 * with a valid FAT filesystem, this setting is ignored, and the FAT driver automatically
 * uses the block/cluster size specified in the existing filesystem metadata.
 *
 * Including this setting is still important because:
 * 1. It ensures consistent formatting behavior if the SD card needs to be reformatted.
 * 2. It provides a default cluster size that balances storage efficiency and performance.
 *
 * Note: The FAT driver reads the cluster size from the SD card's FAT boot sector when
 * mounting an existing filesystem. This ensures compatibility, but if the existing
 * cluster size is incompatible with the application or hardware, mounting may fail.
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

