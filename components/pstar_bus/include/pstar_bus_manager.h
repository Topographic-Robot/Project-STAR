/* components/pstar_bus/include/bus_manager.h */

#ifndef PSTAR_BUS_MANAGER_H
#define PSTAR_BUS_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "pstar_bus_manager_types.h"

/* Public Functions ***********************************************************/

/**
 * @brief Initialize a bus manager.
 * 
 * @param[out] manager Pointer to a bus manager structure.
 * @param[in]  tag     Tag for logging (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_manager_init(pstar_bus_manager_t* manager, const char* tag);

/**
 * @brief Add a new bus configuration to the manager.
 * 
 * @param[in] manager Pointer to the bus manager.
 * @param[in] config  Pointer to the bus configuration to add.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_manager_add_bus(pstar_bus_manager_t* manager, 
                                    pstar_bus_config_t*  config);

/**
 * @brief Find a bus configuration by name.
 * 
 * @param[in] manager Pointer to the bus manager.
 * @param[in] name    Name of the bus to find.
 * @return pstar_bus_config_t* Pointer to the found configuration, 
 *                             or NULL if not found.
 */
pstar_bus_config_t* pstar_bus_manager_find_bus(const pstar_bus_manager_t* manager, 
                                               const char*                name);

/**
 * @brief Remove a bus configuration from the manager.
 * 
 * @param[in] manager Pointer to the bus manager.
 * @param[in] name    Name of the bus to remove.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_manager_remove_bus(pstar_bus_manager_t* manager, 
                                       const char*          name);

/**
 * @brief Deinitialize a bus manager and free all resources.
 * 
 * @param[in] manager Pointer to the bus manager.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_manager_deinit(pstar_bus_manager_t* manager);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_MANAGER_H */
