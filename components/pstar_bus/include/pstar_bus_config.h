/* components/pstar_bus/include/bus_config.h */

#ifndef PSTAR_BUS_CONFIG_H
#define PSTAR_BUS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "pstar_bus_types.h"

/* Public Functions ***********************************************************/

/**
 * @brief Create a new I2C bus configuration.
 * 
 * @param[in] name    Name of the bus.
 * @param[in] port    I2C port number.
 * @param[in] address 7-bit I2C device address.
 * @param[in] mode    Operation mode.
 * @return pstar_bus_config_t* Pointer to the created configuration, or NULL on failure.
 */
pstar_bus_config_t* pstar_bus_config_create_i2c(const char*         name, 
                                                i2c_port_t          port, 
                                                uint8_t             address, 
                                                pstar_common_mode_t mode);

/**
 * @brief Create a new SPI bus configuration.
 * 
 * @param[in] name  Name of the bus.
 * @param[in] host  SPI host device.
 * @param[in] mode  Operation mode.
 * @return pstar_bus_config_t* Pointer to the created configuration, or NULL on failure.
 */
pstar_bus_config_t* pstar_bus_config_create_spi(const char*         name, 
                                                spi_host_device_t   host, 
                                                pstar_common_mode_t mode);

/**
 * @brief Create a new UART bus configuration.
 * 
 * @param[in] name           Name of the bus.
 * @param[in] port           UART port number.
 * @param[in] rx_buffer_size Size of the RX buffer.
 * @param[in] tx_buffer_size Size of the TX buffer.
 * @param[in] mode           Operation mode.
 * @return pstar_bus_config_t* Pointer to the created configuration, or NULL on failure.
 */
pstar_bus_config_t* pstar_bus_config_create_uart(const char*         name, 
                                                 uart_port_t         port,
                                                 size_t              rx_buffer_size, 
                                                 size_t              tx_buffer_size,
                                                 pstar_common_mode_t mode);

/**
 * @brief Create a new GPIO bus configuration.
 * 
 * @param[in] name  Name of the bus.
 * @param[in] mode  Operation mode.
 * @return pstar_bus_config_t* Pointer to the created configuration, or NULL on failure.
 */
pstar_bus_config_t* pstar_bus_config_create_gpio(const char*         name, 
                                                 pstar_common_mode_t mode);

/**
 * @brief Create a new SDIO bus configuration.
 * 
 * @param[in] name       Name of the bus.
 * @param[in] host_flags SDIO host configuration flags.
 * @param[in] slot       SDIO slot.
 * @param[in] mode       Operation mode.
 * @return pstar_bus_config_t* Pointer to the created configuration, or NULL on failure.
 */
pstar_bus_config_t* pstar_bus_config_create_sdio(const char*         name, 
                                                 uint32_t            host_flags,
                                                 uint8_t             slot,
                                                 pstar_common_mode_t mode);

/**
 * @brief Destroy a bus configuration and free all resources.
 * 
 * @param[in] config Pointer to the bus configuration to destroy.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_config_destroy(pstar_bus_config_t* config);

/**
 * @brief Initialize a bus configuration.
 * 
 * @param[in] config Pointer to the bus configuration to initialize.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_config_init(pstar_bus_config_t* config);

/**
 * @brief Deinitialize a bus configuration.
 * 
 * @param[in] config Pointer to the bus configuration to deinitialize.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_config_deinit(pstar_bus_config_t* config);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_CONFIG_H */
