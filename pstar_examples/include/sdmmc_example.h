/* pstar_examples/include/sdmmc_example.h */

#ifndef SDMMC_EXAMPLE_H
#define SDMMC_EXAMPLE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Entry point for the SDMMC HAL (SPI/SDMMC Mode) example.
 * Initializes resources, performs file operations, and typically does not return.
 */
void example_sdmmc_app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* SDMMC_EXAMPLE_H */