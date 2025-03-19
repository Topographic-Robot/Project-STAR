/* components/pstar_bus/include/bus_i2c.h */

#ifndef PSTAR_BUS_I2C_H
#define PSTAR_BUS_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "pstar_bus_types.h"

/* Forward Declarations *******************************************************/

typedef struct pstar_bus_manager pstar_bus_manager_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initialize default I2C operations.
 *
 * @param[out] ops  Pointer to I2C operations structure to initialize.
 */
void pstar_bus_i2c_init_default_ops(pstar_i2c_ops_t* ops);

/**
 * @brief Write data to an I2C device.
 *
 * @param[in]  manager       Pointer to the bus manager.
 * @param[in]  name          Name of the I2C bus.
 * @param[in]  data          Data to write.
 * @param[in]  len           Length of data to write.
 * @param[in]  reg_addr      Register address to write to.
 * @param[out] bytes_written Pointer to store the number of bytes written (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_i2c_write(const pstar_bus_manager_t* manager,
                              const char*                name,
                              const uint8_t*             data, 
                              size_t                     len, 
                              uint8_t                    reg_addr, 
                              size_t*                    bytes_written);

/**
 * @brief Read data from an I2C device.
 *
 * @param[in]  manager    Pointer to the bus manager.
 * @param[in]  name       Name of the I2C bus.
 * @param[out] data       Buffer to read data into.
 * @param[in]  len        Length of data to read.
 * @param[in]  reg_addr   Register address to read from.
 * @param[out] bytes_read Pointer to store the number of bytes read (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_i2c_read(const pstar_bus_manager_t* manager,
                             const char*                name,
                             uint8_t*                   data, 
                             size_t                     len, 
                             uint8_t                    reg_addr, 
                             size_t*                    bytes_read);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_I2C_H */
