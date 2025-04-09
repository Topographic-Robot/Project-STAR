/* components/pstar_storage_hal/include/pstar_storage_sdio_hal.h */

#ifndef PSTAR_STORAGE_SDIO_HAL_H
#define PSTAR_STORAGE_SDIO_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_storage_types.h"

#include "esp_err.h"

/**
 * @brief Checks if SDIO mode is supported on this system
 * 
 * @note Currently always returns false as SDIO support is not yet implemented
 *
 * @return Always false (SDIO not supported in this version)
 */
bool storage_sdio_is_supported(void);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_STORAGE_SDIO_HAL_H */
