/* components/storage/sd_card_hal/include/sd_card_hal.h */

#ifndef TOPOROBO_SD_CARD_HAL_H
#define TOPOROBO_SD_CARD_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Constants ******************************************************************/

extern const char             *sd_card_tag;                  /**< Tag for Logging */
extern const char             *sd_card_mount_path;           /**< Root path for the mounting */
extern const uint8_t           sd_card_cs;                   /**< GPIO pin for SPI Chip Select (CS) */
extern const uint8_t           sd_card_data_to_card;         /**< GPIO pin for SPI communication into the SD card (DI) */
extern const uint8_t           sd_card_data_from_card;       /**< GPIO pin for SPI communication out of the SD card (DO) */
extern const uint8_t           sd_card_clk;                  /**< GPIO pin for SPI clock (SCLK / CLK) */
extern const uint8_t           sd_card_cd;                   /**< GPIO pin for Card Detect (CD) */
extern const uint32_t          sd_card_spi_freq_hz;          /**< SPI bus frequency in Hz for SD card */
extern const spi_host_device_t sd_card_spi_host;             /**< Default SPI host for SD card interface */
extern const uint8_t           sd_card_max_files;            /**< Maximum number of files that can be opened simultaneously */
extern const uint32_t          sd_card_allocation_unit_size; /**< Alllocation unit size for SD card filesystem in Bytes */
extern const uint32_t          sd_card_max_transfer_sz;      /**< Maximum transfer size for SPI transactions in Bytes */
extern const uint8_t           sd_card_max_retries;          /**< Maxiumum retries after init fails */
extern const uint32_t          sd_card_retry_delay_ms;       /**< The delay between retries */

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the SD card for file operations.
 *
 * Mounts the SD card filesystem using FATFS. The card must be properly formatted
 * with a FAT filesystem for successful mounting. Uses the SPI host configured in
 * `sd_card_spi_host`.
 *
 * @return
 * - `ESP_OK`         if the initialization is successful.
 * - `ESP_ERR_NO_MEM` if there is insufficient memory.
 * - `ESP_FAIL`       if mounting fails.
 */
esp_err_t sd_card_init(void);

/**
 * @brief Initializes the SD card detection system.
 *
 * Sets up the Card Detect (CD) pin and interrupt handler to monitor SD card
 * insertion and removal. This function should be called before using any
 * other SD card functions that rely on card detection.
 *
 * @return
 * - ESP_OK if the initialization is successful.
 * - ESP_FAIL if initialization fails.
 */
esp_err_t sd_card_detection_init(void);

/**
 * @brief Checks if the SD card is currently available.
 *
 * @return true if the SD card is available, false otherwise.
 */
bool sd_card_is_available(void);

/**
 * @brief Registers a callback function to be called when SD card availability changes.
 *
 * @param callback Function to call when SD card availability changes.
 * @return ESP_OK if successful, ESP_FAIL otherwise.
 */
esp_err_t sd_card_register_availability_callback(void (*callback)(bool available));

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_SD_CARD_HAL_H */

