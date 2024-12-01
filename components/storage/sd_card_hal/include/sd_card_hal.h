/* components/storage/sd_card_hal/include/sd_card_hal.h */

#ifndef TOBOROBO_SD_CARD_HAL_H
#define TOBOROBO_SD_CARD_HAL_H

#include "esp_err.h"

/**
 * @brief Initializes the SD card for file operations.
 *
 * Mounts the SD card filesystem using FATFS. The card must be properly formatted
 * with a FAT filesystem for successful mounting.
 *
 * @return
 * - ESP_OK if the initialization is successful.
 * - ESP_FAIL if mounting fails.
 */
esp_err_t sd_card_init(void);

#endif /* TOBOROBO_SD_CARD_HAL_H */

