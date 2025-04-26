/* components/pstar_bus/include/pstar_bus_config.h */

#ifndef PSTAR_COMPONENT_BUS_CONFIG_H
#define PSTAR_COMPONENT_BUS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_types.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h" /* Added for SPI */

#include "esp_err.h"

/* --- Forward declaration needed for the function signature --- */
struct pstar_bus_manager;

/* --- Public Functions --- */

/**
 * @brief Create a new I2C bus/device configuration.
 *
 * Configures parameters for a specific I2C device communication.
 * The underlying I2C driver/port will be initialized when pstar_bus_config_init is called.
 *
 * @param[in] name    Unique name for this bus/device instance (e.g., "I2C_SensorA"). Must be non-NULL.
 * @param[in] port    I2C port number (I2C_NUM_0 or I2C_NUM_1).
 * @param[in] address 7-bit I2C device address.
 * @param[in] sda_pin GPIO number for SDA line.
 * @param[in] scl_pin GPIO number for SCL line.
 * @param[in] clk_speed Clock speed in Hz (e.g., 100000 for 100kHz).
 * @return pstar_bus_config_t* Pointer to the created configuration, or NULL on failure. Must be destroyed via pstar_bus_config_destroy.
 */
pstar_bus_config_t* pstar_bus_config_create_i2c(const char* name,
                                                i2c_port_t  port,
                                                uint8_t     address,
                                                gpio_num_t  sda_pin,
                                                gpio_num_t  scl_pin,
                                                uint32_t    clk_speed);

/**
 * @brief Create a new SPI device configuration.
 *
 * Configures parameters for a specific SPI device communication.
 * The underlying SPI bus driver will be initialized when pstar_bus_config_init is called
 * for the first device on a given SPI host.
 *
 * @param[in] name        Unique name for this SPI device instance (e.g., "SPI_LCD"). Must be non-NULL.
 * @param[in] host        SPI host device (e.g., SPI2_HOST, SPI3_HOST).
 * @param[in] posi_pin    GPIO number for POSI (Primary Out Secondary In) line.
 * @param[in] piso_pin    GPIO number for PISO (Primary In Secondary Out) line (-1 if not used).
 * @param[in] sclk_pin    GPIO number for SCLK line.
 * @param[in] dma_chan    SPI DMA channel to use (SPI_DMA_CH_AUTO, SPI_DMA_CH1, SPI_DMA_CH2, or 0 if DMA disabled).
 * @param[in] dev_cfg     Pointer to the ESP-IDF SPI device interface configuration structure.
 *                        This structure contains CS pin, clock speed, mode, queue size, callbacks, etc.
 *                        The bus manager does NOT take ownership of this pointer or its contents;
 *                        it copies the relevant data. The caller must ensure dev_cfg is valid.
 * @return pstar_bus_config_t* Pointer to the created configuration, or NULL on failure. Must be destroyed via pstar_bus_config_destroy.
 */
pstar_bus_config_t*
pstar_bus_config_create_spi_device(const char*                          name,
                                   spi_host_device_t                    host,
                                   gpio_num_t                           posi_pin,
                                   gpio_num_t                           piso_pin,
                                   gpio_num_t                           sclk_pin,
                                   int                                  dma_chan,
                                   const spi_device_interface_config_t* dev_cfg);

/**
 * @brief Destroy a bus configuration and free all associated resources.
 *
 * This function will first attempt to deinitialize the bus/device driver if it was initialized
 * by this configuration (e.g., i2c_driver_delete, spi_bus_remove_device).
 * For SPI, it will also attempt to free the underlying SPI bus if this was the last device on it.
 * Then, it frees the memory allocated for the configuration structure itself.
 *
 * @param[in] config Pointer to the bus configuration to destroy. This pointer will be invalid after the call.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if config is NULL, or an error code from the underlying deinitialization process.
 */
esp_err_t pstar_bus_config_destroy(pstar_bus_config_t* config);

/**
 * @brief Initialize the bus/device associated with this configuration.
 *
 * For I2C: Performs i2c_param_config, i2c_driver_install.
 * For SPI: Performs spi_bus_initialize (if not already done for this host) and spi_bus_add_device.
 *          Requires the bus manager to track host initialization status.
 *
 * @param[in] config Pointer to the bus configuration to initialize.
 * @param[in] manager Pointer to the bus manager (needed for SPI host tracking). Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if config or manager is NULL, ESP_ERR_INVALID_STATE if already initialized, or an error code from the underlying driver initialization.
 */
esp_err_t pstar_bus_config_init(pstar_bus_config_t* config, struct pstar_bus_manager* manager);

/**
 * @brief Deinitialize the bus/device associated with this configuration.
 *
 * For I2C: Performs i2c_driver_delete.
 * For SPI: Performs spi_bus_remove_device. Does NOT automatically free the SPI bus;
 *          that happens during pstar_bus_config_destroy if it's the last device.
 *
 * @param[in] config Pointer to the bus configuration to deinitialize.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if config is NULL, ESP_ERR_INVALID_STATE if not initialized, or an error code from the underlying driver deinitialization.
 */
esp_err_t pstar_bus_config_deinit(pstar_bus_config_t* config);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_BUS_CONFIG_H */