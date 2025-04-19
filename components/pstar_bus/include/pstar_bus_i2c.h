/* components/pstar_bus/include/pstar_bus_i2c.h */

#ifndef PSTAR_COMPONENT_BUS_I2C_H
#define PSTAR_COMPONENT_BUS_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_types.h"

#include "driver/i2c.h"

#include <stdint.h>

#include "esp_err.h"

/* --- Public Functions --- */

/**
 * @brief Initialize default I2C operations function pointers within a config structure.
 *        Typically called internally by pstar_bus_config_create_i2c.
 *        Assigns default implementations for write, read, write_command, and read_raw.
 *
 * @param[out] ops Pointer to I2C operations structure to initialize. Must not be NULL.
 */
void pstar_bus_i2c_init_default_ops(pstar_i2c_ops_t* ops);

/**
 * @brief Write data to an I2C device specified by a bus configuration, preceded by a register address.
 *
 * Finds the bus configuration by name and calls its write operation.
 *
 * @param[in]  manager       Pointer to the initialized bus manager. Must not be NULL.
 * @param[in]  name          Name of the I2C bus/device configuration. Must not be NULL.
 * @param[in]  data          Pointer to the data buffer to write. Must not be NULL.
 * @param[in]  len           Number of bytes to write from the data buffer. Must be > 0.
 * @param[in]  reg_addr      The register address to write before writing data.
 * @param[out] bytes_written Pointer to store the number of data bytes actually written (excluding register address). Can be NULL if not needed.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if params invalid, ESP_ERR_NOT_FOUND if bus not found, ESP_ERR_INVALID_STATE if bus not initialized, or an error from the driver (e.g., timeout, NACK).
 */
esp_err_t pstar_bus_i2c_write(const pstar_bus_manager_t* manager,
                              const char*                name,
                              const uint8_t*             data,
                              size_t                     len,
                              uint8_t                    reg_addr,
                              size_t*                    bytes_written);

/**
 * @brief Read data from an I2C device specified by a bus configuration, after writing a register address.
 *
 * Finds the bus configuration by name and calls its read operation.
 *
 * @param[in]  manager    Pointer to the initialized bus manager. Must not be NULL.
 * @param[in]  name       Name of the I2C bus/device configuration. Must not be NULL.
 * @param[out] data       Pointer to the buffer where read data will be stored. Must not be NULL.
 * @param[in]  len        Number of bytes to read into the data buffer. Must be > 0.
 * @param[in]  reg_addr   The register address to write before initiating the read.
 * @param[out] bytes_read Pointer to store the number of data bytes actually read. Can be NULL if not needed.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if params invalid, ESP_ERR_NOT_FOUND if bus not found, ESP_ERR_INVALID_STATE if bus not initialized, or an error from the driver (e.g., timeout, NACK).
 */
esp_err_t pstar_bus_i2c_read(const pstar_bus_manager_t* manager,
                             const char*                name,
                             uint8_t*                   data,
                             size_t                     len,
                             uint8_t                    reg_addr,
                             size_t*                    bytes_read);

/**
 * @brief Write a single command byte to an I2C device specified by a bus configuration.
 *        No register address or additional data is sent after the command.
 *
 * Finds the bus configuration by name and calls its write_command operation.
 *
 * @param[in] manager Pointer to the initialized bus manager. Must not be NULL.
 * @param[in] name    Name of the I2C bus/device configuration. Must not be NULL.
 * @param[in] command The command byte to write.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if params invalid, ESP_ERR_NOT_FOUND if bus not found, ESP_ERR_INVALID_STATE if bus not initialized, or an error from the driver.
 */
esp_err_t
pstar_bus_i2c_write_command(const pstar_bus_manager_t* manager, const char* name, uint8_t command);

/**
 * @brief Read raw bytes from an I2C device specified by a bus configuration.
 *        Does not send a register address before reading.
 *
 * Finds the bus configuration by name and calls its read_raw operation.
 *
 * @param[in]  manager    Pointer to the initialized bus manager. Must not be NULL.
 * @param[in]  name       Name of the I2C bus/device configuration. Must not be NULL.
 * @param[out] data       Pointer to the buffer where read data will be stored. Must not be NULL.
 * @param[in]  len        Number of bytes to read into the data buffer. Must be > 0.
 * @param[out] bytes_read Pointer to store the number of data bytes actually read. Can be NULL if not needed.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if params invalid, ESP_ERR_NOT_FOUND if bus not found, ESP_ERR_INVALID_STATE if bus not initialized, or an error from the driver.
 */
esp_err_t pstar_bus_i2c_read_raw(const pstar_bus_manager_t* manager,
                                 const char*                name,
                                 uint8_t*                   data,
                                 size_t                     len,
                                 size_t*                    bytes_read);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_BUS_I2C_H */
