/* components/pstar_bus/include/bus_sdio.h */

#ifndef PSTAR_BUS_SDIO_H
#define PSTAR_BUS_SDIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp_err.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "pstar_bus_types.h"

/* Forward Declarations *******************************************************/

typedef struct pstar_bus_manager pstar_bus_manager_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initialize default SDIO operations.
 *
 * @param[out] ops Pointer to SDIO operations structure to initialize.
 */
void pstar_bus_sdio_init_default_ops(pstar_sdio_ops_t* ops);

/**
 * @brief Write data to an SDIO device.
 *
 * @param[in]  manager       Pointer to the bus manager.
 * @param[in]  name          Name of the SDIO bus.
 * @param[in]  data          Data to write.
 * @param[in]  len           Length of data to write.
 * @param[in]  offset        Offset to write at.
 * @param[out] bytes_written Pointer to store the number of bytes written (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_sdio_write(const pstar_bus_manager_t* manager,
                               const char*                name,
                               const uint8_t*             data, 
                               size_t                     len,
                               size_t                     offset,
                               size_t*                    bytes_written);

/**
 * @brief Read data from an SDIO device.
 *
 * @param[in]  manager    Pointer to the bus manager.
 * @param[in]  name       Name of the SDIO bus.
 * @param[out] data       Buffer to read data into.
 * @param[in]  len        Length of data to read.
 * @param[in]  offset     Offset to read from.
 * @param[out] bytes_read Pointer to store the number of bytes read (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_sdio_read(const pstar_bus_manager_t* manager,
                              const char*                name,
                              uint8_t*                   data, 
                              size_t                     len,
                              size_t                     offset,
                              size_t*                    bytes_read);

/**
 * @brief Perform an IOCTL operation on an SDIO device.
 *
 * @param[in] manager Pointer to the bus manager.
 * @param[in] name    Name of the SDIO bus.
 * @param[in] cmd     IOCTL command.
 * @param[in] arg     Argument for the command.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_sdio_ioctl(const pstar_bus_manager_t* manager,
                               const char*                name,
                               int                        cmd,
                               void*                      arg);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_SDIO_H */
