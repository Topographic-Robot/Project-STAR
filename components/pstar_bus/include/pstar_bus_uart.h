/* components/pstar_bus/include/pstar_bus_uart.h */

#ifndef PSTAR_BUS_UART_H
#define PSTAR_BUS_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_types.h"

#include "driver/uart.h"

#include <stdint.h>

#include "esp_err.h"

/* Forward Declarations *******************************************************/

typedef struct pstar_bus_manager pstar_bus_manager_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initialize default UART operations.
 *
 * @param[out] ops Pointer to UART operations structure to initialize.
 */
void pstar_bus_uart_init_default_ops(pstar_uart_ops_t* ops);

/**
 * @brief Write data to a UART device.
 *
 * @param[in]  manager       Pointer to the bus manager.
 * @param[in]  name          Name of the UART bus.
 * @param[in]  data          Data to write.
 * @param[in]  len           Length of data to write.
 * @param[out] bytes_written Pointer to store the number of bytes written
 *                           (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_uart_write(const pstar_bus_manager_t* manager,
                               const char*                name,
                               const uint8_t*             data,
                               size_t                     len,
                               size_t*                    bytes_written);

/**
 * @brief Read data from a UART device.
 *
 * @param[in]  manager    Pointer to the bus manager.
 * @param[in]  name       Name of the UART bus.
 * @param[out] data       Buffer to read data into.
 * @param[in]  len        Length of data to read.
 * @param[out] bytes_read Pointer to store the number of bytes read
 *                        (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_uart_read(const pstar_bus_manager_t* manager,
                              const char*                name,
                              uint8_t*                   data,
                              size_t                     len,
                              size_t*                    bytes_read);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_UART_H */
