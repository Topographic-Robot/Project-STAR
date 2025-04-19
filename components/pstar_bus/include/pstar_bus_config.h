/* components/pstar_bus/include/pstar_bus_config.h */

#ifndef PSTAR_COMPONENT_BUS_CONFIG_H
#define PSTAR_COMPONENT_BUS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_types.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_err.h"

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
 * @brief Destroy a bus configuration and free all associated resources.
 *
 * This function will first attempt to deinitialize the I2C driver if it was initialized
 * by this configuration. Then, it frees the memory allocated for the configuration
 * structure itself.
 *
 * @param[in] config Pointer to the bus configuration to destroy. This pointer will be invalid after the call.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if config is NULL, or an error code from the underlying deinitialization process.
 */
esp_err_t pstar_bus_config_destroy(pstar_bus_config_t* config);

/**
 * @brief Initialize the I2C bus/device associated with this configuration.
 *
 * This performs the necessary I2C driver initialization (i2c_param_config, i2c_driver_install).
 *
 * @param[in] config Pointer to the bus configuration to initialize.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if config is NULL, ESP_ERR_INVALID_STATE if already initialized, or an error code from the underlying driver initialization.
 */
esp_err_t pstar_bus_config_init(pstar_bus_config_t* config);

/**
 * @brief Deinitialize the I2C bus/device associated with this configuration.
 *
 * This performs the necessary I2C driver deinitialization (i2c_driver_delete).
 *
 * @param[in] config Pointer to the bus configuration to deinitialize.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if config is NULL, ESP_ERR_INVALID_STATE if not initialized, or an error code from the underlying driver deinitialization.
 */
esp_err_t pstar_bus_config_deinit(pstar_bus_config_t* config);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_BUS_CONFIG_H */
