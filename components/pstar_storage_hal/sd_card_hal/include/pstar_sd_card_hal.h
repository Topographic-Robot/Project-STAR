/* components/pstar_storage_hal/sd_card_hal/include/pstar_sd_card_hal.h */
/**
 * @file pstar_sd_card_hal.h
 * @brief Backward compatibility header for SD card HAL
 * 
 * This file provides backward compatibility with previous implementations.
 * New code should use pstar_storage_hal.h directly.
 */

#ifndef PSTAR_SD_CARD_HAL_H
#define PSTAR_SD_CARD_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include the new header files */
#include "pstar_storage_hal.h"
#include "pstar_storage_types.h"

/* Directly map old types to new types for backward compatibility */
typedef sd_card_hal_t pstar_sd_card_hal_t;
typedef sd_card_pin_config_t pstar_sd_card_pin_config_t;
typedef sd_card_task_config_t pstar_sd_card_task_config_t;
typedef sd_card_performance_t pstar_sd_card_performance_t;
typedef sd_interface_type_t pstar_sd_interface_type_t;
typedef sd_bus_width_t pstar_sd_bus_width_t;
typedef sd_state_t pstar_sd_state_t;
typedef sd_availability_cb_t pstar_sd_availability_cb_t;

/* Define old enum values using new ones */
#define PSTAR_SD_INTERFACE_NONE k_sd_interface_none
#define PSTAR_SD_INTERFACE_SPI k_sd_interface_spi
#define PSTAR_SD_INTERFACE_SDIO k_sd_interface_sdio
#define PSTAR_SD_INTERFACE_COUNT k_sd_interface_count

#define PSTAR_SD_BUS_WIDTH_1BIT k_sd_bus_width_1bit
#define PSTAR_SD_BUS_WIDTH_4BIT k_sd_bus_width_4bit

#define PSTAR_SD_STATE_IDLE k_sd_state_idle
#define PSTAR_SD_STATE_CARD_INSERTED k_sd_state_card_inserted
#define PSTAR_SD_STATE_INTERFACE_DISCOVERY k_sd_state_interface_discovery
#define PSTAR_SD_STATE_INTERFACE_READY k_sd_state_interface_ready
#define PSTAR_SD_STATE_ERROR k_sd_state_error
#define PSTAR_SD_STATE_FAILED k_sd_state_failed

/* Map old function names to new ones */
#define pstar_sd_card_init_default sd_card_init_default
#define pstar_sd_card_init_with_pins sd_card_init_with_pins
#define pstar_sd_card_set_task_config sd_card_set_task_config
#define pstar_sd_card_init sd_card_init
#define pstar_sd_card_is_available sd_card_is_available
#define pstar_sd_card_get_state sd_card_get_state
#define pstar_sd_card_get_current_interface sd_card_get_current_interface
#define pstar_sd_card_get_bus_width sd_card_get_bus_width
#define pstar_sd_card_interface_to_string sd_card_interface_to_string
#define pstar_sd_card_get_performance sd_card_get_performance
#define pstar_sd_card_force_remount sd_card_force_remount
#define pstar_sd_card_set_bus_width sd_card_set_bus_width
#define pstar_sd_card_cleanup sd_card_cleanup
#define pstar_sd_card_register_availability_callback sd_card_register_availability_callback

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_SD_CARD_HAL_H */
