/* main/include/system_setup.h */

#ifndef TOPOROBO_SYSTEM_SETUP_H
#define TOPOROBO_SYSTEM_SETUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/* Constants ******************************************************************/

extern const char* const system_setup_tag; /**< Tag for system setup */

/* Public Functions ***********************************************************/

/**
 * @brief Initializes system-level hardware resources
 * 
 * This function initializes hardware resources required by the system, such as:
 * - I2C buses for sensors and motors
 * - SPI bus for SD card
 * - UART for debugging and communication
 * - GPIO pins for various peripherals
 * 
 * @return ESP_OK if initialization succeeded, error code otherwise
 */
esp_err_t system_setup_init(void);

/**
 * @brief Cleans up system-level hardware resources
 * 
 * This function cleans up hardware resources used by the system.
 * 
 * @return ESP_OK if cleanup succeeded, error code otherwise
 */
esp_err_t system_setup_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_SYSTEM_SETUP_H */ 