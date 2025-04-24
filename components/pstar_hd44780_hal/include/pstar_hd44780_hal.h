/* components/pstar_hd44780_hal/include/pstar_hd44780_hal.h */

#ifndef PSTAR_COMPONENT_HD44780_HAL_H
#define PSTAR_COMPONENT_HD44780_HAL_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h" /* For SemaphoreHandle_t */
#include "freertos/semphr.h"   /* For SemaphoreHandle_t */

#include <stddef.h> /* For size_t */

#include "esp_err.h"
#include "sdkconfig.h" /* Needed for Kconfig macros */

#ifdef __cplusplus
extern "C" {
#endif

/* --- Configuration --- */

/**
 * @brief Configuration structure for the HD44780 HAL (4-bit mode).
 */
typedef struct {
  gpio_num_t rs_pin; /**< GPIO for Register Select (RS). */
  gpio_num_t e_pin;  /**< GPIO for Enable (E). */
  gpio_num_t d4_pin; /**< GPIO for Data line 4. */
  gpio_num_t d5_pin; /**< GPIO for Data line 5. */
  gpio_num_t d6_pin; /**< GPIO for Data line 6. */
  gpio_num_t d7_pin; /**< GPIO for Data line 7. */
  uint8_t    rows;   /**< Number of rows on the display (e.g., 2 for 16x2). */
  uint8_t    cols;   /**< Number of columns on the display (e.g., 16 for 16x2). */
} pstar_hd44780_hal_config_t;

/* --- Opaque Handle --- */
typedef struct pstar_hd44780_hal_dev_t* pstar_hd44780_hal_handle_t;

/* --- Public Functions --- */

/**
 * @brief Initialize the HD44780 LCD HAL in 4-bit mode.
 *
 * Configures the specified GPIO pins and initializes the LCD according to
 * the HD44780 datasheet sequence for 4-bit operation. Creates a mutex for
 * thread-safe access.
 *
 * @param[in] config Configuration specifying the GPIO pins, rows, and columns. Must not be NULL.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, errors otherwise (e.g., invalid pin, memory allocation failure, mutex creation error).
 */
esp_err_t pstar_hd44780_hal_init(const pstar_hd44780_hal_config_t* config,
                                 pstar_hd44780_hal_handle_t*       out_handle);

/**
 * @brief Deinitialize the HD44780 HAL.
 *
 * Frees the handle resources, including the mutex, and resets the GPIO pins.
 * Optionally clears the display before deinitialization.
 *
 * @param[in] handle Handle obtained from pstar_hd44780_hal_init.
 * @param[in] clear_display If true, attempts to clear the display before deinit.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL. Clear command errors are logged but don't prevent deinit.
 */
esp_err_t pstar_hd44780_hal_deinit(pstar_hd44780_hal_handle_t handle, bool clear_display);

/**
 * @brief Send a command byte to the LCD.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_hd44780_hal_init.
 * @param[in] command The command byte to send.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from GPIO operations otherwise.
 */
esp_err_t pstar_hd44780_hal_send_command(pstar_hd44780_hal_handle_t handle, uint8_t command);

/**
 * @brief Send a data byte (character) to the LCD.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_hd44780_hal_init.
 * @param[in] data The data byte (character ASCII code) to send.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from GPIO operations otherwise.
 */
esp_err_t pstar_hd44780_hal_send_data(pstar_hd44780_hal_handle_t handle, uint8_t data);

/**
 * @brief Clear the entire LCD display and return cursor to home (0, 0).
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_hd44780_hal_init.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from command sending otherwise.
 */
esp_err_t pstar_hd44780_hal_clear(pstar_hd44780_hal_handle_t handle);

/**
 * @brief Return the cursor to the home position (0, 0).
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_hd44780_hal_init.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from command sending otherwise.
 */
esp_err_t pstar_hd44780_hal_home(pstar_hd44780_hal_handle_t handle);

/**
 * @brief Set the cursor position on the LCD.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_hd44780_hal_init.
 * @param[in] col Column number (0-based).
 * @param[in] row Row number (0-based).
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if position is out of bounds, ESP_ERR_TIMEOUT if mutex times out, errors from command sending otherwise.
 */
esp_err_t pstar_hd44780_hal_set_cursor(pstar_hd44780_hal_handle_t handle, uint8_t col, uint8_t row);

/**
 * @brief Print a null-terminated string at the current cursor position.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_hd44780_hal_init.
 * @param[in] str The null-terminated string to print. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if str is NULL, ESP_ERR_TIMEOUT if mutex times out, errors from data sending otherwise.
 */
esp_err_t pstar_hd44780_hal_print_string(pstar_hd44780_hal_handle_t handle, const char* str);

/**
 * @brief Print formatted data to the LCD at the current cursor position.
 * Similar to printf. This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_hd44780_hal_init.
 * @param[in] format Format string.
 * @param[in] ... Variable arguments for the format string.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from data sending or formatting otherwise.
 */
esp_err_t pstar_hd44780_hal_printf(pstar_hd44780_hal_handle_t handle, const char* format, ...)
  __attribute__((format(printf, 2, 3)));

/**
 * @brief Initialize the HD44780 LCD with default configuration from KConfig.
 *
 * This convenience function creates and initializes an HD44780 LCD using the
 * configuration values defined in KConfig.
 *
 * @note This function will return ESP_ERR_NOT_SUPPORTED if the HD44780 component
 *       is disabled via KConfig (`CONFIG_PSTAR_KCONFIG_HD44780_ENABLED` is false).
 *
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_SUPPORTED if component disabled,
 *                   or another error code if initialization fails.
 */
esp_err_t pstar_hd44780_hal_create_kconfig_default(pstar_hd44780_hal_handle_t* out_handle);

/**
 * @brief Initialize the HD44780 LCD with custom configuration.
 *
 * This function creates and initializes an HD44780 LCD using custom GPIO pins and dimensions.
 *
 * @param[in] rs_pin GPIO for Register Select (RS).
 * @param[in] e_pin GPIO for Enable (E).
 * @param[in] d4_pin GPIO for Data line 4.
 * @param[in] d5_pin GPIO for Data line 5.
 * @param[in] d6_pin GPIO for Data line 6.
 * @param[in] d7_pin GPIO for Data line 7.
 * @param[in] rows Number of rows on the display.
 * @param[in] cols Number of columns on the display.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, or an error code if initialization fails.
 */
esp_err_t pstar_hd44780_hal_create_custom(gpio_num_t                  rs_pin,
                                          gpio_num_t                  e_pin,
                                          gpio_num_t                  d4_pin,
                                          gpio_num_t                  d5_pin,
                                          gpio_num_t                  d6_pin,
                                          gpio_num_t                  d7_pin,
                                          uint8_t                     rows,
                                          uint8_t                     cols,
                                          pstar_hd44780_hal_handle_t* out_handle);

/**
 * @brief Registers the pins used by the HD44780 component with the pin validator using KConfig values.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if both the
 * pin validator component and the HD44780 component are enabled via Kconfig.
 *
 * @return esp_err_t ESP_OK on success (or if validator/component is disabled), or an error code on failure.
 */
esp_err_t pstar_hd44780_register_kconfig_pins(void);

/**
 * @brief Registers custom pins used by the HD44780 component with the pin validator.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if the
 * pin validator component is enabled via Kconfig.
 *
 * @param[in] rs_pin GPIO pin number for RS to register.
 * @param[in] e_pin GPIO pin number for E to register.
 * @param[in] d4_pin GPIO pin number for D4 to register.
 * @param[in] d5_pin GPIO pin number for D5 to register.
 * @param[in] d6_pin GPIO pin number for D6 to register.
 * @param[in] d7_pin GPIO pin number for D7 to register.
 * @return esp_err_t ESP_OK on success (or if validator is disabled), or an error code on failure.
 */
esp_err_t pstar_hd44780_register_custom_pins(int rs_pin,
                                             int e_pin,
                                             int d4_pin,
                                             int d5_pin,
                                             int d6_pin,
                                             int d7_pin);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_HD44780_HAL_H */