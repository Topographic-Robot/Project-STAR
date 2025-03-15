/* components/pstar_bus/include/bus_common_types.h */

#ifndef PSTAR_BUS_COMMON_TYPES_H
#define PSTAR_BUS_COMMON_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Forward Declarations *******************************************************/

/* Forward declare pstar_bus_config_t and pstar_bus_event_t */
typedef struct pstar_bus_config pstar_bus_config_t;
typedef struct pstar_bus_event pstar_bus_event_t;

/* Forward declare pstar_bus_manager_t for use in function pointer types */
typedef struct pstar_bus_manager pstar_bus_manager_t;

/* Enums **********************************************************************/

/**
 * @brief Types of supported bus interfaces.
 */
typedef enum : uint8_t {
  k_pstar_bus_type_none,  /**< No bus type specified */
  k_pstar_bus_type_i2c,   /**< I2C bus */
  k_pstar_bus_type_spi,   /**< SPI bus */
  k_pstar_bus_type_uart,  /**< UART bus */
  k_pstar_bus_type_gpio,  /**< GPIO bus */
  k_pstar_bus_type_sdio,  /**< SDIO bus */
  k_pstar_bus_type_count, /**< Number of bus types */
} pstar_bus_type_t;

/**
 * @brief Common operation modes for buses.
 */
typedef enum : uint8_t {
  k_pstar_mode_none,      /**< No mode specified */
  k_pstar_mode_polling,   /**< Polling mode (synchronous) */
  k_pstar_mode_interrupt, /**< Interrupt mode (asynchronous) */
  k_pstar_mode_dma,       /**< DMA mode (high-performance asynchronous) */
  k_pstar_mode_combined,  /**< Combined mode (interrupt+DMA) */
  k_pstar_mode_count,     /**< Number of modes */
} pstar_common_mode_t;

/**
 * @brief IOCTL commands for SDIO bus.
 */
typedef enum : uint32_t {
  k_pstar_sdio_ioctl_get_card_info,      /**< Get card info (arg: sdmmc_card_t**) */
  k_pstar_sdio_ioctl_get_status,         /**< Get card status (arg: uint8_t*) */
  k_pstar_sdio_ioctl_get_csd,            /**< Get CSD register (arg: sdmmc_csd_t*) */
  k_pstar_sdio_ioctl_get_cid,            /**< Get CID register (arg: sdmmc_cid_t*) */
  k_pstar_sdio_ioctl_get_scr,            /**< Get SCR register (arg: sdmmc_scr_t*) */
  k_pstar_sdio_ioctl_get_ocr,            /**< Get OCR register (arg: uint32_t*) */
  k_pstar_sdio_ioctl_get_rca,            /**< Get RCA register (arg: uint16_t*) */
  k_pstar_sdio_ioctl_get_bus_width,      /**< Get bus width (arg: uint8_t*) */
  k_pstar_sdio_ioctl_set_bus_width,      /**< Set bus width (arg: uint8_t*) */
  k_pstar_sdio_ioctl_get_bus_freq,       /**< Get bus frequency (arg: uint32_t*) */
  k_pstar_sdio_ioctl_set_bus_freq,       /**< Set bus frequency (arg: uint32_t*) */
  k_pstar_sdio_ioctl_enable_card_detect, /**< Enable card detect (arg: bool*) */
  k_pstar_sdio_ioctl_custom,             /**< Custom command, device-specific */
} pstar_sdio_ioctl_cmd_t;

/* Public Functions ***********************************************************/

/**
 * @brief Convert bus type to string.
 * 
 * @param[in] type Bus type.
 * @return const char* String representation of the bus type.
 */
const char* pstar_bus_type_to_string(pstar_bus_type_t type);

/**
 * @brief Convert mode to string.
 * 
 * @param[in] mode Operation mode.
 * @return const char* String representation of the mode.
 */
const char* pstar_common_mode_to_string(pstar_common_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_COMMON_TYPES_H */
