/* components/common/include/common_cleanup.h */

#ifndef TOPOROBO_COMMON_CLEANUP_H
#define TOPOROBO_COMMON_CLEANUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"

/**
 * @brief Cleans up an I2C bus.
 *
 * @param i2c_bus The I2C bus number to clean up.
 * @param tag The tag to use for logging.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t common_cleanup_i2c(i2c_port_t i2c_bus, const char* tag);

/**
 * @brief Cleans up GPIO pins with the specified parameters.
 *
 * @param pin_mask The mask of GPIO pins to clean up.
 * @param tag The tag to use for logging.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t common_cleanup_gpio(uint64_t pin_mask, const char* tag);

/**
 * @brief Cleans up a UART with the specified parameters.
 *
 * @param uart_num The UART number to clean up.
 * @param tag The tag to use for logging.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t common_cleanup_uart(uart_port_t uart_num, const char* tag);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_COMMON_CLEANUP_H */ 