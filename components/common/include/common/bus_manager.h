/* components/common/include/common/bus_manager.h */

#ifndef TOPOROBO_BUS_MANAGER_H
#define TOPOROBO_BUS_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"
#include "driver/uart.h"

/* Constants ******************************************************************/

extern const char* const bus_manager_tag; /**< Tag for logging bus manager messages */

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the I2C bus with the specified parameters.
 *
 * Configures and initializes an I2C bus with the given parameters. This function
 * should be called during system initialization before any I2C devices are used.
 *
 * @param[in] scl_io  GPIO pin number for the SCL (clock) line.
 * @param[in] sda_io  GPIO pin number for the SDA (data) line.
 * @param[in] freq_hz Frequency in Hz for the I2C bus.
 * @param[in] i2c_bus I2C port number to initialize.
 *
 * @return 
 * - ESP_OK if the I2C bus was initialized successfully.
 * - ESP_FAIL or other error code if initialization failed.
 */
esp_err_t bus_manager_i2c_init(uint8_t    scl_io, 
                               uint8_t    sda_io, 
                               uint32_t   freq_hz, 
                               i2c_port_t i2c_bus);

/**
 * @brief Deinitializes the specified I2C bus.
 *
 * Releases resources associated with the I2C bus. This function should be called
 * during system shutdown after all I2C devices have been deinitialized.
 *
 * @param[in] i2c_bus I2C port number to deinitialize.
 *
 * @return 
 * - ESP_OK if the I2C bus was deinitialized successfully.
 * - ESP_FAIL or other error code if deinitialization failed.
 */
esp_err_t bus_manager_i2c_deinit(i2c_port_t i2c_bus);

/**
 * @brief Initializes the SPI bus with the specified parameters.
 *
 * Configures and initializes an SPI bus with the given parameters. This function
 * should be called during system initialization before any SPI devices are used.
 *
 * @param[in] mosi_io GPIO pin number for MOSI (Master Out Slave In).
 * @param[in] miso_io GPIO pin number for MISO (Master In Slave Out).
 * @param[in] sclk_io GPIO pin number for SCLK (Serial Clock).
 * @param[in] host    SPI host to initialize.
 * @param[in] dma_chan DMA channel to use (1 or 2), or 0 for no DMA.
 *
 * @return 
 * - ESP_OK if the SPI bus was initialized successfully.
 * - ESP_FAIL or other error code if initialization failed.
 */
esp_err_t bus_manager_spi_init(int               mosi_io, 
                               int               miso_io, 
                               int               sclk_io, 
                               spi_host_device_t host, 
                               int               dma_chan);

/**
 * @brief Deinitializes the specified SPI bus.
 *
 * Releases resources associated with the SPI bus. This function should be called
 * during system shutdown after all SPI devices have been deinitialized.
 *
 * @param[in] host SPI host to deinitialize.
 *
 * @return 
 * - ESP_OK if the SPI bus was deinitialized successfully.
 * - ESP_FAIL or other error code if deinitialization failed.
 */
esp_err_t bus_manager_spi_deinit(spi_host_device_t host);

/**
 * @brief Initializes the UART interface with specified parameters.
 *
 * Configures the UART interface with the specified TX and RX pins, baud rate,
 * and UART port number. Sets the UART communication parameters and installs the driver.
 *
 * @param[in] tx_io          Pin number for the TX (transmit) line.
 * @param[in] rx_io          Pin number for the RX (receive) line.
 * @param[in] baud_rate      Communication baud rate (e.g., 9600, 115200).
 * @param[in] uart_num       UART port number to configure.
 * @param[in] rx_buffer_size Size of the RX buffer in bytes.
 * @param[in] tx_buffer_size Size of the TX buffer in bytes.
 *
 * @return 
 * - ESP_OK if the UART port was initialized successfully.
 * - ESP_FAIL or other error code if initialization failed.
 */
esp_err_t bus_manager_uart_init(uint8_t     tx_io, 
                                uint8_t     rx_io, 
                                uint32_t    baud_rate,
                                uart_port_t uart_num,
                                size_t      rx_buffer_size,
                                size_t      tx_buffer_size);

/**
 * @brief Deinitializes the specified UART port.
 *
 * Releases resources associated with the UART port. This function should be called
 * during system shutdown after all UART communication has completed.
 *
 * @param[in] uart_num UART port number to deinitialize.
 *
 * @return 
 * - ESP_OK if the UART port was deinitialized successfully.
 * - ESP_FAIL or other error code if deinitialization failed.
 */
esp_err_t bus_manager_uart_deinit(uart_port_t uart_num);

/**
 * @brief Deinitializes all communication buses.
 *
 * Releases resources associated with all I2C, SPI, and UART buses. This function should
 * be called during system shutdown after all devices have been deinitialized.
 *
 * @return 
 * - ESP_OK if all buses were deinitialized successfully.
 * - ESP_FAIL if any bus failed to deinitialize.
 */
esp_err_t bus_manager_deinit_all(void);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_BUS_MANAGER_H */ 