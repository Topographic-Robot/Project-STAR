/* components/pstar_ili9341_hal/include/pstar_ili9341_hal.h */

#ifndef PSTAR_COMPONENT_ILI9341_HAL_H
#define PSTAR_COMPONENT_ILI9341_HAL_H

#include "pstar_bus_manager.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h" /* For SemaphoreHandle_t */
#include "freertos/semphr.h"   /* For SemaphoreHandle_t */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "sdkconfig.h" /* Needed for Kconfig macros */

#ifdef __cplusplus
extern "C" {
#endif

/* --- Constants --- */
#define ILI9341_WIDTH 240
#define ILI9341_HEIGHT 320

/* --- Enums --- */

/** @brief Rotation options for the display */
typedef enum {
  ILI9341_ROTATION_0   = 0, /**< Default portrait */
  ILI9341_ROTATION_90  = 1, /**< Landscape */
  ILI9341_ROTATION_180 = 2, /**< Portrait upside down */
  ILI9341_ROTATION_270 = 3  /**< Landscape upside down */
} pstar_ili9341_rotation_t;

/* --- Configuration --- */

/**
 * @brief Configuration structure for the ILI9341 HAL.
 */
typedef struct {
  const char* spi_bus_name;   /**< Name of the pre-configured SPI device in the bus manager */
  gpio_num_t  dc_pin;         /**< GPIO for Data/Command (D/C or RS) line */
  gpio_num_t  rst_pin;        /**< GPIO for Reset (RST) line (-1 if not used) */
  gpio_num_t  blk_pin;        /**< GPIO for Backlight (BLK) control (-1 if not used) */
  bool        blk_active_low; /**< True if backlight pin is active low */
  uint16_t    width;          /**< Display width in pixels (after rotation) */
  uint16_t    height;         /**< Display height in pixels (after rotation) */
  pstar_ili9341_rotation_t rotation; /**< Initial display rotation */
} pstar_ili9341_hal_config_t;

/* --- Opaque Handle --- */
typedef struct pstar_ili9341_hal_dev_t* pstar_ili9341_hal_handle_t;

/* --- Public Functions --- */

/**
 * @brief Initialize the ILI9341 LCD HAL using a bus managed by pstar_bus_manager.
 *
 * Performs the following:
 * - Finds the specified SPI device configuration in the bus manager.
 * - Configures DC, RST (optional), and BLK (optional) GPIO pins.
 * - Performs the hardware reset sequence (if RST pin is configured).
 * - Sends the ILI9341 initialization command sequence.
 * - Sets the initial rotation and clears the screen.
 * - Creates a mutex for thread-safe access.
 *
 * @param[in] manager Pointer to the initialized bus manager. Must not be NULL.
 * @param[in] config Configuration specifying the SPI bus name, GPIO pins, dimensions, and rotation. Must not be NULL.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, errors otherwise (e.g., bus not found, GPIO config error, SPI comm error, mutex creation error).
 */
esp_err_t pstar_ili9341_hal_init(const pstar_bus_manager_t*        manager,
                                 const pstar_ili9341_hal_config_t* config,
                                 pstar_ili9341_hal_handle_t*       out_handle);

/**
 * @brief Deinitialize the ILI9341 HAL.
 *
 * Frees the handle resources, including the mutex. Does NOT deinitialize the underlying SPI bus
 * managed by pstar_bus_manager. Optionally turns off the backlight and resets GPIO pins.
 *
 * @param[in] handle Handle obtained from pstar_ili9341_hal_init.
 * @param[in] turn_off_backlight If true, attempts to turn off the backlight (if BLK pin configured).
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL. Backlight command error is logged but doesn't prevent deinit.
 */
esp_err_t pstar_ili9341_hal_deinit(pstar_ili9341_hal_handle_t handle, bool turn_off_backlight);

/**
 * @brief Send a command byte to the LCD.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_ili9341_hal_init.
 * @param[in] command The command byte to send.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from SPI communication otherwise.
 */
esp_err_t pstar_ili9341_hal_send_command(pstar_ili9341_hal_handle_t handle, uint8_t command);

/**
 * @brief Send a data byte to the LCD.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_ili9341_hal_init.
 * @param[in] data The data byte to send.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from SPI communication otherwise.
 */
esp_err_t pstar_ili9341_hal_send_data(pstar_ili9341_hal_handle_t handle, uint8_t data);

/**
 * @brief Send multiple data bytes to the LCD.
 * Optimized for sending blocks of data (e.g., pixel colors).
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_ili9341_hal_init.
 * @param[in] data Pointer to the buffer containing data bytes.
 * @param[in] len Number of bytes to send from the buffer.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from SPI communication otherwise.
 */
esp_err_t pstar_ili9341_hal_send_data_block(pstar_ili9341_hal_handle_t handle,
                                            const uint8_t*             data,
                                            size_t                     len);

/**
 * @brief Fill a rectangular area on the screen with a specified color.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_ili9341_hal_init.
 * @param[in] x Starting X coordinate.
 * @param[in] y Starting Y coordinate.
 * @param[in] w Width of the rectangle.
 * @param[in] h Height of the rectangle.
 * @param[in] color 16-bit color value (RGB565 format).
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from SPI communication otherwise.
 */
esp_err_t pstar_ili9341_hal_fill_rect(pstar_ili9341_hal_handle_t handle,
                                      int16_t                    x,
                                      int16_t                    y,
                                      int16_t                    w,
                                      int16_t                    h,
                                      uint16_t                   color);

/**
 * @brief Fill the entire screen with a specified color.
 * Convenience function calling fill_rect with screen dimensions.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_ili9341_hal_init.
 * @param[in] color 16-bit color value (RGB565 format).
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from SPI communication otherwise.
 */
esp_err_t pstar_ili9341_hal_fill_screen(pstar_ili9341_hal_handle_t handle, uint16_t color);

/**
 * @brief Set the display rotation.
 * Updates the MADCTL register and internal width/height tracking.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_ili9341_hal_init.
 * @param[in] rotation The desired rotation value.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from SPI communication otherwise.
 */
esp_err_t pstar_ili9341_hal_set_rotation(pstar_ili9341_hal_handle_t handle,
                                         pstar_ili9341_rotation_t   rotation);

/**
 * @brief Control the backlight state (if BLK pin is configured).
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_ili9341_hal_init.
 * @param[in] on True to turn backlight on, false to turn off.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if BLK pin not configured, ESP_ERR_TIMEOUT if mutex times out, or error from gpio_set_level.
 */
esp_err_t pstar_ili9341_hal_set_backlight(pstar_ili9341_hal_handle_t handle, bool on);

/**
 * @brief Initialize the ILI9341 LCD with default configuration from KConfig.
 *
 * This convenience function creates and initializes an ILI9341 LCD using the
 * configuration values defined in KConfig. It performs the following steps:
 * 1. Creates an SPI device configuration using KConfig values.
 * 2. Adds the configuration to the bus manager.
 * 3. Initializes the SPI bus/device hardware.
 * 4. Initializes the ILI9341 HAL with the KConfig settings (pins, rotation, etc.).
 *
 * @note This function will return ESP_ERR_NOT_SUPPORTED if the ILI9341 component
 *       is disabled via KConfig (`CONFIG_PSTAR_KCONFIG_ILI9341_ENABLED` is false).
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL.
 *                    This manager WILL be modified (an SPI device config is added).
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_SUPPORTED if component disabled,
 *                   or another error code if any step fails.
 */
esp_err_t pstar_ili9341_hal_create_kconfig_default(pstar_bus_manager_t*        manager,
                                                   pstar_ili9341_hal_handle_t* out_handle);

/**
 * @brief Initialize the ILI9341 LCD with custom configuration.
 *
 * This function creates and initializes an ILI9341 LCD using custom configuration values.
 * It performs the following steps:
 * 1. Creates an SPI device configuration with provided parameters.
 * 2. Adds the configuration to the bus manager.
 * 3. Initializes the SPI bus/device hardware.
 * 4. Initializes the ILI9341 HAL with the specified settings.
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL.
 * @param[in] spi_bus_name Name for the SPI device configuration (e.g., "LCD_SPI"). Must not be NULL.
 * @param[in] host SPI host device (e.g., SPI2_HOST).
 * @param[in] clk_speed_hz SPI clock speed in Hz (e.g., 40000000 for 40MHz).
 * @param[in] spi_mode SPI mode (0, 1, 2, or 3).
 * @param[in] posi_pin GPIO pin number for POSI (Primary Out Secondary In).
 * @param[in] sclk_pin GPIO pin number for SCLK.
 * @param[in] cs_pin GPIO pin number for Chip Select (CS).
 * @param[in] dc_pin GPIO pin number for Data/Command (D/C).
 * @param[in] rst_pin GPIO pin number for Reset (-1 if not used).
 * @param[in] blk_pin GPIO pin number for Backlight (-1 if not used).
 * @param[in] blk_active_low True if backlight pin is active low.
 * @param[in] rotation Initial display rotation.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, or an error code if any step fails.
 */
esp_err_t pstar_ili9341_hal_create_custom(pstar_bus_manager_t*        manager,
                                          const char*                 spi_bus_name,
                                          spi_host_device_t           host,
                                          int                         clk_speed_hz,
                                          uint8_t                     spi_mode,
                                          gpio_num_t                  posi_pin,
                                          gpio_num_t                  sclk_pin,
                                          gpio_num_t                  cs_pin,
                                          gpio_num_t                  dc_pin,
                                          gpio_num_t                  rst_pin,
                                          gpio_num_t                  blk_pin,
                                          bool                        blk_active_low,
                                          pstar_ili9341_rotation_t    rotation,
                                          pstar_ili9341_hal_handle_t* out_handle);

/**
 * @brief Registers the SPI and control pins used by the ILI9341 component with the pin validator using KConfig values.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if both the
 * pin validator component and the ILI9341 component are enabled via Kconfig.
 *
 * @return esp_err_t ESP_OK on success (or if validator/component is disabled), or an error code on failure.
 */
esp_err_t pstar_ili9341_register_kconfig_pins(void);

/**
 * @brief Registers custom SPI and control pins used by the ILI9341 component with the pin validator.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if the
 * pin validator component is enabled via Kconfig.
 *
 * @param[in] posi_pin GPIO pin number for POSI to register.
 * @param[in] piso_pin GPIO pin number for PISO to register (-1 if not used).
 * @param[in] sclk_pin GPIO pin number for SCLK to register.
 * @param[in] cs_pin GPIO pin number for CS to register.
 * @param[in] dc_pin GPIO pin number for DC to register.
 * @param[in] rst_pin GPIO pin number for RST to register (-1 if not used).
 * @param[in] blk_pin GPIO pin number for BLK to register (-1 if not used).
 * @return esp_err_t ESP_OK on success (or if validator is disabled), or an error code on failure.
 */
esp_err_t pstar_ili9341_register_custom_pins(int posi_pin,
                                             int piso_pin,
                                             int sclk_pin,
                                             int cs_pin,
                                             int dc_pin,
                                             int rst_pin,
                                             int blk_pin);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_ILI9341_HAL_H */