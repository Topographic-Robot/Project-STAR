/* components/pstar_sdmmc_hal/include/pstar_sdmmc_hal.h */

#ifndef PSTAR_COMPONENT_SDMMC_HAL_H
#define PSTAR_COMPONENT_SDMMC_HAL_H

#include "pstar_bus_manager.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "sdkconfig.h" /* Needed for Kconfig macros */
#include "sdmmc_cmd.h" /* For sdmmc_card_t */

#ifdef __cplusplus
extern "C" {
#endif

/* --- Enums --- */

/** @brief SD Card Interface Mode */
typedef enum {
  PSTAR_SDMMC_MODE_SPI,        /**< Use SPI peripheral */
  PSTAR_SDMMC_MODE_SDMMC_1BIT, /**< Use SDMMC peripheral with 1 data line */
  PSTAR_SDMMC_MODE_SDMMC_4BIT  /**< Use SDMMC peripheral with 4 data lines */
} pstar_sdmmc_interface_mode_t;

/* --- Configuration --- */

/** @brief Configuration specific to SDMMC peripheral mode */
typedef struct {
  gpio_num_t clk_pin; /**< SDMMC CLK pin */
  gpio_num_t cmd_pin; /**< SDMMC CMD pin */
  gpio_num_t d0_pin;  /**< SDMMC D0 pin */
  gpio_num_t d1_pin;  /**< SDMMC D1 pin (4-bit mode only, set to -1 for 1-bit) */
  gpio_num_t d2_pin;  /**< SDMMC D2 pin (4-bit mode only, set to -1 for 1-bit) */
  gpio_num_t d3_pin;  /**< SDMMC D3 pin (4-bit mode only, set to -1 for 1-bit) */
} pstar_sdmmc_sdmmc_config_t;

/** @brief Configuration specific to SPI peripheral mode */
typedef struct {
  const char* spi_bus_name; /**< Name of the pre-configured SPI device in the bus manager */
} pstar_sdmmc_spi_config_t;

/**
 * @brief Configuration structure for the SDMMC HAL.
 */
typedef struct {
  pstar_sdmmc_interface_mode_t interface_mode; /**< Selected interface mode (SPI or SDMMC) */
  const char* mount_point; /**< Path where the FAT filesystem will be mounted (e.g., "/sdcard") */
  int
    max_files; /**< Maximum number of open files allowed simultaneously on the mounted filesystem */
  bool format_if_mount_failed; /**< If true, format the card if mounting fails */

  /* Union to hold mode-specific configuration */
  union {
    pstar_sdmmc_spi_config_t   spi;   /**< Configuration for SPI mode */
    pstar_sdmmc_sdmmc_config_t sdmmc; /**< Configuration for SDMMC mode */
  } mode_config;

} pstar_sdmmc_hal_config_t;

/* --- Opaque Handle --- */
typedef struct pstar_sdmmc_hal_dev_t* pstar_sdmmc_hal_handle_t;

/* --- Public Functions --- */

/**
 * @brief Initialize the SDMMC HAL using the specified interface mode.
 *
 * Performs the following based on the mode in the config:
 * - SPI Mode: Finds the SPI device config, initializes sdspi_host.
 * - SDMMC Mode: Configures SDMMC pins, initializes sdmmc_host.
 * - Common: Mounts the FAT filesystem, optionally formats on failure.
 *
 * @param[in] manager Pointer to the initialized bus manager (REQUIRED for SPI mode, can be NULL for SDMMC mode).
 * @param[in] config Configuration specifying the interface mode, pins/bus, mount point, etc. Must not be NULL.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, errors otherwise (e.g., bus not found, mount failed, memory error).
 */
esp_err_t pstar_sdmmc_hal_init(const pstar_bus_manager_t*      manager, /* Needed for SPI */
                               const pstar_sdmmc_hal_config_t* config,
                               pstar_sdmmc_hal_handle_t*       out_handle);

/**
 * @brief Deinitialize the SDMMC HAL.
 *
 * Unmounts the filesystem and releases SDMMC/SPI resources associated with the card.
 * Does NOT deinitialize the underlying SPI bus managed by pstar_bus_manager if SPI mode was used.
 *
 * @param[in] handle Handle obtained from pstar_sdmmc_hal_init.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
esp_err_t pstar_sdmmc_hal_deinit(pstar_sdmmc_hal_handle_t handle);

/**
 * @brief Get information about the mounted SD card.
 *
 * @param[in] handle Handle obtained from pstar_sdmmc_hal_init.
 * @param[out] out_card_info Pointer to store the card information structure. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle or out_card_info is NULL, ESP_ERR_INVALID_STATE if card not initialized/mounted.
 */
esp_err_t pstar_sdmmc_hal_get_card_info(pstar_sdmmc_hal_handle_t handle,
                                        sdmmc_card_t**           out_card_info);

/**
 * @brief Get the mount point path configured for this SD card instance.
 *
 * @param[in] handle Handle obtained from pstar_sdmmc_hal_init.
 * @return const char* Pointer to the mount point string, or NULL if handle is invalid.
 */
const char* pstar_sdmmc_hal_get_mount_point(pstar_sdmmc_hal_handle_t handle);

/**
 * @brief Check if the SD card is currently mounted.
 *
 * @param[in] handle Handle obtained from pstar_sdmmc_hal_init.
 * @return true if the card is mounted, false otherwise.
 */
bool pstar_sdmmc_hal_is_mounted(pstar_sdmmc_hal_handle_t handle);

/**
 * @brief Initialize the SDMMC HAL with default configuration from KConfig.
 *
 * Reads the interface mode and corresponding settings from Kconfig.
 * If SPI mode is selected, it creates/adds/initializes the required SPI bus configuration
 * within the provided bus manager.
 * If SDMMC mode is selected, it configures the SDMMC peripheral directly.
 *
 * @note This function will return ESP_ERR_NOT_SUPPORTED if the SDMMC HAL component
 *       is disabled via KConfig (`CONFIG_PSTAR_KCONFIG_SDMMC_ENABLED` is false).
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL (required for potential SPI setup).
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_SUPPORTED if component disabled,
 *                   or another error code if any step fails.
 */
esp_err_t pstar_sdmmc_hal_create_kconfig_default(pstar_bus_manager_t*      manager,
                                                 pstar_sdmmc_hal_handle_t* out_handle);

/**
 * @brief Initialize the SDMMC HAL with custom SPI configuration.
 *
 * Creates/adds/initializes the required SPI bus configuration within the manager.
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL.
 * @param[in] spi_bus_name Name for the SPI device configuration (e.g., "SDCard_SPI"). Must not be NULL.
 * @param[in] host SPI host device (e.g., SPI2_HOST).
 * @param[in] clk_speed_hz SPI clock speed in Hz (e.g., 20000000 for 20MHz).
 * @param[in] posi_pin GPIO pin number for POSI (MOSI).
 * @param[in] piso_pin GPIO pin number for PISO (MISO).
 * @param[in] sclk_pin GPIO pin number for SCLK.
 * @param[in] cs_pin GPIO pin number for Chip Select (CS).
 * @param[in] mount_point Path where the FAT filesystem will be mounted (e.g., "/sdcard"). Must not be NULL.
 * @param[in] max_files Maximum number of open files allowed simultaneously.
 * @param[in] format_if_mount_failed If true, format the card if mounting fails.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, or an error code if any step fails.
 */
esp_err_t pstar_sdmmc_hal_create_custom_spi(pstar_bus_manager_t*      manager,
                                            const char*               spi_bus_name,
                                            spi_host_device_t         host,
                                            int                       clk_speed_hz,
                                            gpio_num_t                posi_pin,
                                            gpio_num_t                piso_pin,
                                            gpio_num_t                sclk_pin,
                                            gpio_num_t                cs_pin,
                                            const char*               mount_point,
                                            int                       max_files,
                                            bool                      format_if_mount_failed,
                                            pstar_sdmmc_hal_handle_t* out_handle);

/**
 * @brief Initialize the SDMMC HAL with custom SDMMC peripheral configuration.
 *
 * Configures the SDMMC peripheral directly, bypassing the bus manager.
 *
 * @param[in] use_1_bit_mode If true, configures for 1-bit SDMMC mode, otherwise 4-bit.
 * @param[in] clk_pin GPIO pin number for SDMMC CLK.
 * @param[in] cmd_pin GPIO pin number for SDMMC CMD.
 * @param[in] d0_pin GPIO pin number for SDMMC D0.
 * @param[in] d1_pin GPIO pin number for SDMMC D1 (ignored if use_1_bit_mode is true).
 * @param[in] d2_pin GPIO pin number for SDMMC D2 (ignored if use_1_bit_mode is true).
 * @param[in] d3_pin GPIO pin number for SDMMC D3 (ignored if use_1_bit_mode is true).
 * @param[in] mount_point Path where the FAT filesystem will be mounted (e.g., "/sdcard"). Must not be NULL.
 * @param[in] max_files Maximum number of open files allowed simultaneously.
 * @param[in] format_if_mount_failed If true, format the card if mounting fails.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, or an error code if any step fails.
 */
esp_err_t pstar_sdmmc_hal_create_custom_sdmmc(bool                      use_1_bit_mode,
                                              gpio_num_t                clk_pin,
                                              gpio_num_t                cmd_pin,
                                              gpio_num_t                d0_pin,
                                              gpio_num_t                d1_pin,
                                              gpio_num_t                d2_pin,
                                              gpio_num_t                d3_pin,
                                              const char*               mount_point,
                                              int                       max_files,
                                              bool                      format_if_mount_failed,
                                              pstar_sdmmc_hal_handle_t* out_handle);

/**
 * @brief Registers the pins used by the SDMMC HAL component with the pin validator using KConfig values.
 *
 * Reads the configured interface mode and registers the corresponding pins (SPI or SDMMC).
 *
 * @return esp_err_t ESP_OK on success (or if validator/component is disabled), or an error code on failure.
 */
esp_err_t pstar_sdmmc_register_kconfig_pins(void);

/**
 * @brief Registers custom SPI pins used by the SDMMC HAL component with the pin validator.
 *
 * @param[in] posi_pin GPIO pin number for POSI to register.
 * @param[in] piso_pin GPIO pin number for PISO to register.
 * @param[in] sclk_pin GPIO pin number for SCLK to register.
 * @param[in] cs_pin GPIO pin number for CS to register.
 * @return esp_err_t ESP_OK on success (or if validator is disabled), or an error code on failure.
 */
esp_err_t
pstar_sdmmc_register_custom_spi_pins(int posi_pin, int piso_pin, int sclk_pin, int cs_pin);

/**
 * @brief Registers custom SDMMC pins used by the SDMMC HAL component with the pin validator.
 *
 * @param[in] use_1_bit_mode True if registering for 1-bit mode, false for 4-bit mode.
 * @param[in] clk_pin GPIO pin number for CLK to register.
 * @param[in] cmd_pin GPIO pin number for CMD to register.
 * @param[in] d0_pin GPIO pin number for D0 to register.
 * @param[in] d1_pin GPIO pin number for D1 to register (ignored if use_1_bit_mode is true).
 * @param[in] d2_pin GPIO pin number for D2 to register (ignored if use_1_bit_mode is true).
 * @param[in] d3_pin GPIO pin number for D3 to register (ignored if use_1_bit_mode is true).
 * @return esp_err_t ESP_OK on success (or if validator is disabled), or an error code on failure.
 */
esp_err_t pstar_sdmmc_register_custom_sdmmc_pins(bool use_1_bit_mode,
                                                 int  clk_pin,
                                                 int  cmd_pin,
                                                 int  d0_pin,
                                                 int  d1_pin,
                                                 int  d2_pin,
                                                 int  d3_pin);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_SDMMC_HAL_H */