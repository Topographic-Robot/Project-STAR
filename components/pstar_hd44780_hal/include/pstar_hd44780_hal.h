/* components/pstar_hd44780_hal/include/pstar_hd44780_hal.h */

#ifndef PSTAR_COMPONENT_HD44780_HAL_H
#define PSTAR_COMPONENT_HD44780_HAL_H

#include "pstar_bus_manager.h" /* Needed for I2C mode */

#include "driver/gpio.h"
#include "driver/i2c.h"        /* For i2c_port_t */
#include "freertos/FreeRTOS.h" /* For SemaphoreHandle_t */
#include "freertos/semphr.h"   /* For SemaphoreHandle_t */

#include <stdbool.h>
#include <stddef.h> /* For size_t */

#include "esp_err.h"
#include "sdkconfig.h" /* Needed for Kconfig macros */

#ifdef __cplusplus
extern "C" {
#endif

/* --- Enum for Interface Mode --- */
typedef enum {
  HD44780_MODE_PARALLEL, /**< 4-bit parallel interface */
  HD44780_MODE_I2C,      /**< I2C interface via PCF8574 backpack */
} pstar_hd44780_interface_mode_t;

/* --- Configuration --- */

/**
 * @brief Configuration structure for the HD44780 HAL.
 * Fields are used based on the selected interface mode.
 */
typedef struct {
  pstar_hd44780_interface_mode_t mode; /**< Interface mode (Parallel or I2C). */
  uint8_t                        rows; /**< Number of rows on the display (e.g., 2 for 16x2). */
  uint8_t                        cols; /**< Number of columns on the display (e.g., 16 for 16x2). */

  /* Parallel Mode Fields */
  gpio_num_t rs_pin; /**< Parallel: GPIO for Register Select (RS). */
  gpio_num_t e_pin;  /**< Parallel: GPIO for Enable (E). */
  gpio_num_t d4_pin; /**< Parallel: GPIO for Data line 4. */
  gpio_num_t d5_pin; /**< Parallel: GPIO for Data line 5. */
  gpio_num_t d6_pin; /**< Parallel: GPIO for Data line 6. */
  gpio_num_t d7_pin; /**< Parallel: GPIO for Data line 7. */

  /* I2C Mode Fields */
  const char* i2c_bus_name; /**< I2C: Name of the pre-configured I2C bus in the manager. */
                            /* I2C address is part of the bus configuration found by name. */
  bool i2c_backlight_on;    /**< I2C: Initial state of the backlight (true=on, false=off). */

} pstar_hd44780_hal_config_t;

/* --- Opaque Handle --- */
typedef struct pstar_hd44780_hal_dev_t* pstar_hd44780_hal_handle_t;

/* --- Public Functions --- */

/**
 * @brief Initialize the HD44780 LCD HAL based on the configuration mode.
 *
 * If Parallel mode: Configures the specified GPIO pins.
 * If I2C mode: Uses the specified bus name to interact via the bus manager. Assumes
 *             the bus identified by `i2c_bus_name` is already configured and initialized.
 *
 * Initializes the LCD according to the HD44780 datasheet sequence for 4-bit operation.
 * Creates a mutex for thread-safe access.
 *
 * @param[in] manager Pointer to the initialized bus manager (REQUIRED, even for parallel mode, though unused there).
 * @param[in] config Configuration specifying the mode, pins/bus, rows, and columns. Must not be NULL.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, errors otherwise (e.g., invalid config, memory allocation failure, mutex creation error, I2C bus not found).
 */
esp_err_t pstar_hd44780_hal_init(const pstar_bus_manager_t* manager, /* Manager needed for I2C */
                                 const pstar_hd44780_hal_config_t* config,
                                 pstar_hd44780_hal_handle_t*       out_handle);

/**
 * @brief Deinitialize the HD44780 HAL.
 *
 * Frees the handle resources, including the mutex.
 * If Parallel mode: Resets the GPIO pins.
 * If I2C mode: Optionally turns off the backlight.
 * Optionally clears the display before deinitialization.
 * Does NOT deinitialize the underlying I2C bus managed by pstar_bus_manager.
 *
 * @param[in] handle Handle obtained from pstar_hd44780_hal_init.
 * @param[in] clear_display If true, attempts to clear the display before deinit.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL. Clear/backlight command errors are logged but don't prevent deinit.
 */
esp_err_t pstar_hd44780_hal_deinit(pstar_hd44780_hal_handle_t handle, bool clear_display);

/**
 * @brief Send a command byte to the LCD.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_hd44780_hal_init.
 * @param[in] command The command byte to send.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from GPIO/I2C operations otherwise.
 */
esp_err_t pstar_hd44780_hal_send_command(pstar_hd44780_hal_handle_t handle, uint8_t command);

/**
 * @brief Send a data byte (character) to the LCD.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_hd44780_hal_init.
 * @param[in] data The data byte (character ASCII code) to send.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from GPIO/I2C operations otherwise.
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
 * @brief Turn the LCD backlight ON (I2C mode only).
 * Does nothing if the interface mode is Parallel.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_hd44780_hal_init.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not in I2C mode, ESP_ERR_TIMEOUT if mutex times out, or I2C error.
 */
esp_err_t pstar_hd44780_hal_backlight_on(pstar_hd44780_hal_handle_t handle);

/**
 * @brief Turn the LCD backlight OFF (I2C mode only).
 * Does nothing if the interface mode is Parallel.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_hd44780_hal_init.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not in I2C mode, ESP_ERR_TIMEOUT if mutex times out, or I2C error.
 */
esp_err_t pstar_hd44780_hal_backlight_off(pstar_hd44780_hal_handle_t handle);

/**
 * @brief Initialize the HD44780 LCD with default configuration from KConfig.
 *
 * This convenience function creates and initializes an HD44780 LCD using the
 * configuration values defined in KConfig (interface mode, pins/I2C settings, dimensions).
 * If I2C mode is selected, it creates/adds/initializes the required I2C bus configuration
 * within the provided bus manager.
 *
 * @note This function will return ESP_ERR_NOT_SUPPORTED if the HD44780 component
 *       is disabled via KConfig (`CONFIG_PSTAR_KCONFIG_HD44780_ENABLED` is false).
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL.
 *                    This manager WILL be modified if I2C mode is selected.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_SUPPORTED if component disabled,
 *                   or another error code if initialization fails.
 */
esp_err_t pstar_hd44780_hal_create_kconfig_default(pstar_bus_manager_t*        manager,
                                                   pstar_hd44780_hal_handle_t* out_handle);

/**
 * @brief Initialize the HD44780 LCD with custom parallel configuration.
 *
 * @param[in] manager Bus manager pointer (passed to init but unused in parallel mode).
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
esp_err_t pstar_hd44780_hal_create_custom_parallel(const pstar_bus_manager_t*  manager,
                                                   gpio_num_t                  rs_pin,
                                                   gpio_num_t                  e_pin,
                                                   gpio_num_t                  d4_pin,
                                                   gpio_num_t                  d5_pin,
                                                   gpio_num_t                  d6_pin,
                                                   gpio_num_t                  d7_pin,
                                                   uint8_t                     rows,
                                                   uint8_t                     cols,
                                                   pstar_hd44780_hal_handle_t* out_handle);

/**
 * @brief Initialize the HD44780 LCD with custom I2C configuration.
 *
 * This function creates/adds/initializes the required I2C bus configuration within the
 * provided bus manager before initializing the LCD HAL itself.
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL.
 *                    This manager WILL be modified (a bus config is added).
 * @param[in] i2c_bus_name Name for the I2C bus configuration (e.g., "LCD_I2C"). Must not be NULL.
 * @param[in] port I2C port number to use.
 * @param[in] addr I2C address of the LCD backpack.
 * @param[in] sda_pin GPIO pin number for SDA.
 * @param[in] scl_pin GPIO pin number for SCL.
 * @param[in] i2c_freq_hz I2C bus frequency in Hz.
 * @param[in] rows Number of rows on the display.
 * @param[in] cols Number of columns on the display.
 * @param[in] initial_backlight_on Initial state of the backlight.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, or an error code if initialization fails.
 */
esp_err_t pstar_hd44780_hal_create_custom_i2c(pstar_bus_manager_t*        manager,
                                              const char*                 i2c_bus_name,
                                              i2c_port_t                  port,
                                              uint8_t                     addr,
                                              gpio_num_t                  sda_pin,
                                              gpio_num_t                  scl_pin,
                                              uint32_t                    i2c_freq_hz,
                                              uint8_t                     rows,
                                              uint8_t                     cols,
                                              bool                        initial_backlight_on,
                                              pstar_hd44780_hal_handle_t* out_handle);

/**
 * @brief Registers the pins used by the HD44780 component with the pin validator using KConfig values.
 *
 * Reads the configured interface mode from Kconfig. If Parallel, registers RS, E, D4-D7 pins.
 * If I2C, registers SDA and SCL pins.
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if the
 * pin validator and HD44780 components are enabled via Kconfig.
 *
 * @return esp_err_t ESP_OK on success (or if validator/component is disabled), or an error code on failure.
 */
esp_err_t pstar_hd44780_register_kconfig_pins(void);

/**
 * @brief Registers custom parallel pins used by the HD44780 component with the pin validator.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. Only performs registration if the
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
esp_err_t pstar_hd44780_register_custom_parallel_pins(int rs_pin,
                                                      int e_pin,
                                                      int d4_pin,
                                                      int d5_pin,
                                                      int d6_pin,
                                                      int d7_pin);

/**
 * @brief Registers custom I2C pins used by the HD44780 component with the pin validator.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. Only performs registration if the
 * pin validator component is enabled via Kconfig.
 *
 * @param[in] sda_pin GPIO pin number for SDA to register.
 * @param[in] scl_pin GPIO pin number for SCL to register.
 * @return esp_err_t ESP_OK on success (or if validator is disabled), or an error code on failure.
 */
esp_err_t pstar_hd44780_register_custom_i2c_pins(int sda_pin, int scl_pin);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_HD44780_HAL_H */