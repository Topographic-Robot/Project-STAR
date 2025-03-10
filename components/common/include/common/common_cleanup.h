/* components/common/include/common/common_cleanup.h */

#ifndef TOPOROBO_COMMON_CLEANUP_H
#define TOPOROBO_COMMON_CLEANUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Constants ******************************************************************/

extern const char* const common_cleanup_tag; /**< Tag for logging common cleanup messages */

/* Public Functions ***********************************************************/

/**
 * @brief Common function to clean up GPIO pins
 *
 * Resets GPIO pins to input mode with no pull-up/down resistors.
 *
 * @param[in] pin_bit_mask Bit mask of GPIO pins to reset
 * @param[in] tag          Tag for logging
 *
 * @return 
 * - ESP_OK if GPIO pins were reset successfully
 * - Error code from gpio_config if reset failed
 */
esp_err_t common_cleanup_gpio(uint64_t pin_bit_mask, const char* const tag);

/**
 * @brief Common function to clean up a mutex
 *
 * Takes the mutex, gives it back, and then deletes it.
 * Sets the mutex pointer to NULL to prevent use after free.
 *
 * @param[in,out] mutex Pointer to the mutex pointer to clean up
 * @param[in]     tag   Tag for logging
 *
 * @return 
 * - ESP_OK if mutex was cleaned up successfully
 * - ESP_ERR_INVALID_ARG if mutex pointer is NULL
 * - ESP_ERR_TIMEOUT if mutex could not be taken
 */
esp_err_t common_cleanup_mutex(SemaphoreHandle_t* mutex, const char* const tag);

/**
 * @brief Common function to clean up I2C resources
 *
 * Cleans up I2C resources and resets GPIO pins.
 *
 * @param[in] i2c_bus I2C port number to clean up
 * @param[in] scl_io  GPIO pin number for SCL
 * @param[in] sda_io  GPIO pin number for SDA
 * @param[in] tag     Tag for logging
 *
 * @return 
 * - ESP_OK if I2C resources were cleaned up successfully
 * - Error code from bus_manager_i2c_cleanup or common_cleanup_gpio if cleanup failed
 */
esp_err_t common_cleanup_i2c(int i2c_bus, int scl_io, int sda_io, const char* const tag);

/**
 * @brief Common function to clean up SPI resources
 *
 * Cleans up SPI resources and resets GPIO pins.
 *
 * @param[in] host    SPI host to clean up
 * @param[in] mosi_io GPIO pin number for MOSI
 * @param[in] miso_io GPIO pin number for MISO
 * @param[in] sclk_io GPIO pin number for SCLK
 * @param[in] tag     Tag for logging
 *
 * @return 
 * - ESP_OK if SPI resources were cleaned up successfully
 * - Error code from bus_manager_spi_cleanup or common_cleanup_gpio if cleanup failed
 */
esp_err_t common_cleanup_spi(int host, int mosi_io, int miso_io, int sclk_io, const char* const tag);

/**
 * @brief Common function to clean up UART resources
 *
 * Cleans up UART resources and resets GPIO pins.
 *
 * @param[in] uart_num UART port number to clean up
 * @param[in] tx_io    GPIO pin number for TX
 * @param[in] rx_io    GPIO pin number for RX
 * @param[in] tag      Tag for logging
 *
 * @return 
 * - ESP_OK if UART resources were cleaned up successfully
 * - Error code from bus_manager_uart_cleanup or common_cleanup_gpio if cleanup failed
 */
esp_err_t common_cleanup_uart(int uart_num, int tx_io, int rx_io, const char* const tag);

/**
 * @brief Common function to handle multiple cleanup operations
 *
 * Executes multiple cleanup functions and aggregates their results.
 * Continues executing all functions even if some fail.
 *
 * @param[in] tag           Tag for logging
 * @param[in] cleanup_name  Name of the cleanup operation for logging
 * @param[in] cleanup_funcs Array of cleanup functions to execute
 * @param[in] num_funcs     Number of functions in the array
 *
 * @return 
 * - ESP_OK if all cleanup functions succeeded
 * - ESP_FAIL if any cleanup function failed
 */
esp_err_t common_cleanup_multiple(const char* const tag, 
                                 const char* const cleanup_name,
                                 esp_err_t (*cleanup_funcs[])(void),
                                 int num_funcs);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_COMMON_CLEANUP_H */ 