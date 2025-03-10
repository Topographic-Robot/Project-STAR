/* components/common/include/common_setup.h */

#ifndef TOPOROBO_COMMON_SETUP_H
#define TOPOROBO_COMMON_SETUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"

/**
 * @brief Sets up an I2C bus with the specified parameters.
 *
 * @param i2c_bus The I2C bus number to set up.
 * @param scl_io The GPIO pin to use for the SCL line.
 * @param sda_io The GPIO pin to use for the SDA line.
 * @param freq_hz The frequency to use for the I2C bus in Hz.
 * @param tag The tag to use for logging.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t common_setup_i2c(i2c_port_t i2c_bus, 
                          uint8_t scl_io, 
                          uint8_t sda_io, 
                          uint32_t freq_hz, 
                          const char* tag);

/**
 * @brief Sets up GPIO pins with the specified parameters.
 *
 * @param pin_mask The mask of GPIO pins to set up.
 * @param tag The tag to use for logging.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t common_setup_gpio(uint64_t pin_mask, const char* tag);

/**
 * @brief Sets up a UART with the specified parameters.
 *
 * @param uart_num The UART number to set up.
 * @param tx_io The GPIO pin to use for the TX line.
 * @param rx_io The GPIO pin to use for the RX line.
 * @param baud_rate The baud rate to use for the UART.
 * @param tag The tag to use for logging.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t common_setup_uart(uart_port_t uart_num, 
                           uint8_t tx_io, 
                           uint8_t rx_io, 
                           uint32_t baud_rate, 
                           const char* tag);

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

#endif /* TOPOROBO_COMMON_SETUP_H */ 