/* components/pstar_bus/include/pstar_bus_function_types.h */

#ifndef PSTAR_COMPONENT_BUS_FUNCTION_TYPES_H
#define PSTAR_COMPONENT_BUS_FUNCTION_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_common_types.h"
#include "pstar_bus_event_types.h"

#include "driver/i2c.h"
#include "driver/spi_master.h" /* Added for SPI */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* --- Callback Function Types --- */
/* Note: Callbacks return void. They should handle errors internally and avoid blocking. */

/* I2C Callback Function Types */
typedef void (*pstar_i2c_transfer_complete_cb_t)(const pstar_bus_event_t* event, void* user_ctx);
typedef void (*pstar_i2c_error_cb_t)(const pstar_bus_event_t* event,
                                     esp_err_t                error,
                                     void*                    user_ctx);

/* SPI Callback Function Types */
typedef void (*pstar_spi_transfer_complete_cb_t)(const pstar_bus_event_t* event, void* user_ctx);
typedef void (*pstar_spi_error_cb_t)(const pstar_bus_event_t* event,
                                     esp_err_t                error,
                                     void*                    user_ctx);

/* --- Operation Function Types --- */

/* I2C Operation Function Types */

/**
 * @brief Function pointer type for writing data preceded by a register address.
 * @param config Pointer to the initialized I2C bus configuration.
 * @param data Pointer to the data buffer to write.
 * @param len Number of bytes to write from the data buffer. Must be > 0.
 * @param reg_addr The register address to write before the data.
 * @param bytes_written Optional pointer to store the number of data bytes written (excluding reg_addr).
 * @return ESP_OK on success, error code otherwise.
 */
typedef esp_err_t (*pstar_i2c_write_fn_t)(const pstar_bus_config_t* config,
                                          const uint8_t*            data,
                                          size_t                    len,
                                          uint8_t                   reg_addr,
                                          size_t*                   bytes_written);

/**
 * @brief Function pointer type for reading data after writing a register address.
 * @param config Pointer to the initialized I2C bus configuration.
 * @param data Pointer to the buffer where read data will be stored.
 * @param len Number of bytes to read into the data buffer. Must be > 0.
 * @param reg_addr The register address to write before initiating the read.
 * @param bytes_read Optional pointer to store the number of data bytes actually read.
 * @return ESP_OK on success, error code otherwise.
 */
typedef esp_err_t (*pstar_i2c_read_fn_t)(const pstar_bus_config_t* config,
                                         uint8_t*                  data,
                                         size_t                    len,
                                         uint8_t                   reg_addr,
                                         size_t*                   bytes_read);

/**
 * @brief Function pointer type for writing a single command byte (no register address, no data).
 *        Useful for devices like BH1750.
 * @param config Pointer to the initialized I2C bus configuration.
 * @param command The command byte to write.
 * @return ESP_OK on success, error code otherwise.
 */
typedef esp_err_t (*pstar_i2c_write_command_fn_t)(const pstar_bus_config_t* config,
                                                  uint8_t                   command);

/**
 * @brief Function pointer type for reading raw bytes without sending a register address first.
 *        Useful for devices like BH1750 after a command has been sent.
 * @param config Pointer to the initialized I2C bus configuration.
 * @param data Pointer to the buffer where read data will be stored.
 * @param len Number of bytes to read into the data buffer. Must be > 0.
 * @param bytes_read Optional pointer to store the number of data bytes actually read.
 * @return ESP_OK on success, error code otherwise.
 */
typedef esp_err_t (*pstar_i2c_read_raw_fn_t)(const pstar_bus_config_t* config,
                                             uint8_t*                  data,
                                             size_t                    len,
                                             size_t*                   bytes_read);

/* SPI Operation Function Types */

/**
 * @brief Function pointer type for transmitting SPI data (command or general data).
 * @param config Pointer to the initialized SPI device configuration.
 * @param tx_buffer Pointer to the data buffer to transmit.
 * @param len Number of bytes to transmit.
 * @param user_flags User-defined flags (e.g., to control DC pin via callback).
 * @return ESP_OK on success, error code otherwise.
 */
typedef esp_err_t (*pstar_spi_transmit_fn_t)(const pstar_bus_config_t* config,
                                             const void*               tx_buffer,
                                             size_t                    len,
                                             uint32_t                  user_flags);

/**
 * @brief Function pointer type for receiving SPI data.
 * @param config Pointer to the initialized SPI device configuration.
 * @param rx_buffer Pointer to the buffer where received data will be stored.
 * @param len Number of bytes to receive.
 * @param user_flags User-defined flags.
 * @return ESP_OK on success, error code otherwise.
 */
typedef esp_err_t (*pstar_spi_receive_fn_t)(const pstar_bus_config_t* config,
                                            void*                     rx_buffer,
                                            size_t                    len,
                                            uint32_t                  user_flags);

/**
 * @brief Function pointer type for full-duplex SPI transfer.
 * @param config Pointer to the initialized SPI device configuration.
 * @param tx_buffer Pointer to the data buffer to transmit.
 * @param rx_buffer Pointer to the buffer where received data will be stored.
 * @param len Number of bytes to transfer.
 * @param user_flags User-defined flags.
 * @return ESP_OK on success, error code otherwise.
 */
typedef esp_err_t (*pstar_spi_transfer_fn_t)(const pstar_bus_config_t* config,
                                             const void*               tx_buffer,
                                             void*                     rx_buffer,
                                             size_t                    len,
                                             uint32_t                  user_flags);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_BUS_FUNCTION_TYPES_H */
