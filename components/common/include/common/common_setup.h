/* components/common/include/common/common_setup.h */

#ifndef TOPOROBO_COMMON_SETUP_H
#define TOPOROBO_COMMON_SETUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Constants ******************************************************************/

extern const char* const common_setup_tag; /**< Tag for logging common setup messages */

/* Public Functions ***********************************************************/

/**
 * @brief Common function to set up GPIO pins
 *
 * Configures GPIO pins with the specified mode and pull-up/down settings.
 *
 * @param[in] pin_bit_mask  Bit mask of GPIO pins to configure
 * @param[in] mode          GPIO mode (input, output, etc.)
 * @param[in] pull_up_en    Whether to enable pull-up resistors
 * @param[in] pull_down_en  Whether to enable pull-down resistors
 * @param[in] intr_type     Interrupt type for the pins
 * @param[in] tag           Tag for logging
 *
 * @return 
 * - ESP_OK if GPIO pins were configured successfully
 * - Error code from gpio_config if configuration failed
 */
esp_err_t common_setup_gpio(uint64_t pin_bit_mask, 
                           gpio_mode_t mode,
                           gpio_pullup_t pull_up_en,
                           gpio_pulldown_t pull_down_en,
                           gpio_int_type_t intr_type,
                           const char* const tag);

/**
 * @brief Common function to set up a mutex
 *
 * Creates a mutex and initializes it.
 *
 * @param[out] mutex Pointer to the mutex pointer to initialize
 * @param[in]  tag   Tag for logging
 *
 * @return 
 * - ESP_OK if mutex was created successfully
 * - ESP_ERR_INVALID_ARG if mutex pointer is NULL
 * - ESP_ERR_NO_MEM if mutex could not be created due to memory constraints
 */
esp_err_t common_setup_mutex(SemaphoreHandle_t* mutex, const char* const tag);

/**
 * @brief Common function to set up I2C resources
 *
 * Sets up I2C resources and configures GPIO pins.
 *
 * @param[in] i2c_bus  I2C port number to set up
 * @param[in] scl_io   GPIO pin number for SCL
 * @param[in] sda_io   GPIO pin number for SDA
 * @param[in] freq_hz  Frequency in Hz for the I2C bus
 * @param[in] tag      Tag for logging
 *
 * @return 
 * - ESP_OK if I2C resources were set up successfully
 * - Error code from bus_manager_i2c_init or common_setup_gpio if setup failed
 */
esp_err_t common_setup_i2c(int i2c_bus, int scl_io, int sda_io, uint32_t freq_hz, const char* const tag);

/**
 * @brief Common function to set up SPI resources
 *
 * Sets up SPI resources and configures GPIO pins.
 *
 * @param[in] host     SPI host to set up
 * @param[in] mosi_io  GPIO pin number for MOSI
 * @param[in] miso_io  GPIO pin number for MISO
 * @param[in] sclk_io  GPIO pin number for SCLK
 * @param[in] dma_chan DMA channel to use (1 or 2), or 0 for no DMA
 * @param[in] tag      Tag for logging
 *
 * @return 
 * - ESP_OK if SPI resources were set up successfully
 * - Error code from bus_manager_spi_init if setup failed
 */
esp_err_t common_setup_spi(int host, int mosi_io, int miso_io, int sclk_io, int dma_chan, const char* const tag);

/**
 * @brief Common function to set up UART resources
 *
 * Sets up UART resources and configures GPIO pins.
 *
 * @param[in] uart_num        UART port number to set up
 * @param[in] tx_io           GPIO pin number for TX
 * @param[in] rx_io           GPIO pin number for RX
 * @param[in] baud_rate       Communication baud rate
 * @param[in] rx_buffer_size  Size of the RX buffer in bytes
 * @param[in] tx_buffer_size  Size of the TX buffer in bytes
 * @param[in] tag             Tag for logging
 *
 * @return 
 * - ESP_OK if UART resources were set up successfully
 * - Error code from bus_manager_uart_init if setup failed
 */
esp_err_t common_setup_uart(int uart_num, int tx_io, int rx_io, uint32_t baud_rate, 
                           size_t rx_buffer_size, size_t tx_buffer_size, const char* const tag);

/**
 * @brief Common function to handle multiple setup operations
 *
 * Executes multiple setup functions and aggregates their results.
 * Continues executing all functions even if some fail.
 *
 * @param[in] tag          Tag for logging
 * @param[in] setup_name   Name of the setup operation for logging
 * @param[in] setup_funcs  Array of setup functions to execute
 * @param[in] num_funcs    Number of functions in the array
 *
 * @return 
 * - ESP_OK if all setup functions succeeded
 * - ESP_FAIL if any setup function failed
 */
esp_err_t common_setup_multiple(const char* const tag, 
                               const char* const setup_name,
                               esp_err_t (*setup_funcs[])(void),
                               int num_funcs);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_COMMON_SETUP_H */ 