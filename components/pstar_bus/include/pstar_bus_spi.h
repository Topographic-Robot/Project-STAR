/* components/pstar_bus/include/pstar_bus_spi.h */

#ifndef PSTAR_COMPONENT_BUS_SPI_H
#define PSTAR_COMPONENT_BUS_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_types.h"

#include "driver/spi_master.h"

#include <stdint.h>

#include "esp_err.h"

/* --- PISO/POSI Explanation --- */
/* PISO: Primary In Secondary Out - Data line where the Primary device receives data from the Secondary device. */
/* POSI: Primary Out Secondary In - Data line where the Primary device sends data to the Secondary device. */

/* --- Public Functions --- */

/**
 * @brief Initialize default SPI operations function pointers within a config structure.
 *        Typically called internally by pstar_bus_config_create_spi_device.
 *        Assigns default implementations for transmit, receive, and transfer.
 *
 * @param[out] ops Pointer to SPI operations structure to initialize. Must not be NULL.
 */
void pstar_bus_spi_init_default_ops(pstar_spi_ops_t* ops);

/**
 * @brief Transmit data to an SPI device specified by a bus configuration.
 *
 * Finds the bus configuration by name and calls its transmit operation.
 * This function typically uses queued transactions for performance.
 *
 * @param[in] manager    Pointer to the initialized bus manager. Must not be NULL.
 * @param[in] name       Name of the SPI device configuration. Must not be NULL.
 * @param[in] tx_buffer  Pointer to the data buffer to transmit. Must not be NULL.
 * @param[in] len        Number of bytes to transmit from the buffer.
 * @param[in] user_flags User-defined flags passed to the underlying transaction
 *                       (e.g., SPI_TRANS_MODE_CMD, SPI_TRANS_MODE_DATA for DC pin control).
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if params invalid, ESP_ERR_NOT_FOUND if bus not found, ESP_ERR_INVALID_STATE if bus not initialized, or an error from the driver.
 */
esp_err_t pstar_bus_spi_transmit(const pstar_bus_manager_t* manager,
                                 const char*                name,
                                 const void*                tx_buffer,
                                 size_t                     len,
                                 uint32_t                   user_flags);

/**
 * @brief Receive data from an SPI device specified by a bus configuration.
 *
 * Finds the bus configuration by name and calls its receive operation.
 * This function typically uses queued transactions for performance.
 *
 * @param[in] manager    Pointer to the initialized bus manager. Must not be NULL.
 * @param[in] name       Name of the SPI device configuration. Must not be NULL.
 * @param[out] rx_buffer Pointer to the buffer where received data will be stored. Must not be NULL.
 * @param[in] len        Number of bytes to receive into the buffer.
 * @param[in] user_flags User-defined flags passed to the underlying transaction.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if params invalid, ESP_ERR_NOT_FOUND if bus not found, ESP_ERR_INVALID_STATE if bus not initialized, or an error from the driver.
 */
esp_err_t pstar_bus_spi_receive(const pstar_bus_manager_t* manager,
                                const char*                name,
                                void*                      rx_buffer,
                                size_t                     len,
                                uint32_t                   user_flags);

/**
 * @brief Perform a full-duplex transfer with an SPI device specified by a bus configuration.
 *
 * Finds the bus configuration by name and calls its transfer operation.
 * This function typically uses queued transactions for performance.
 *
 * @param[in] manager    Pointer to the initialized bus manager. Must not be NULL.
 * @param[in] name       Name of the SPI device configuration. Must not be NULL.
 * @param[in] tx_buffer  Pointer to the data buffer to transmit. Must not be NULL.
 * @param[out] rx_buffer Pointer to the buffer where received data will be stored. Must not be NULL.
 * @param[in] len        Number of bytes to transfer (both transmit and receive).
 * @param[in] user_flags User-defined flags passed to the underlying transaction.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if params invalid, ESP_ERR_NOT_FOUND if bus not found, ESP_ERR_INVALID_STATE if bus not initialized, or an error from the driver.
 */
esp_err_t pstar_bus_spi_transfer(const pstar_bus_manager_t* manager,
                                 const char*                name,
                                 const void*                tx_buffer,
                                 void*                      rx_buffer,
                                 size_t                     len,
                                 uint32_t                   user_flags);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_BUS_SPI_H */