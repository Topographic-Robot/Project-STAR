/* components/pstar_storage_hal/include/pstar_storage_spi_hal.h */

#ifndef PSTAR_STORAGE_SPI_HAL_H
#define PSTAR_STORAGE_SPI_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_storage_types.h"

#include "esp_err.h"

/**
 * @brief Sets up the SPI interface for SD card communication
 *
 * @param[in] sd_card        Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t storage_spi_setup(sd_card_hal_t* sd_card);

/**
 * @brief Checks if SPI mode is supported on this system
 *
 * @return true if SPI mode is supported, false otherwise
 */
bool storage_spi_is_supported(void);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_STORAGE_SPI_HAL_H */
