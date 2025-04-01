/* components/pstar_bus/include/pstar_bus_manager_fwd.h */

#ifndef PSTAR_BUS_MANAGER_FWD_H
#define PSTAR_BUS_MANAGER_FWD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/* Forward Declarations *******************************************************/

/**
 * @brief Forward declaration of pstar_bus_manager_t structure
 *
 * This forward declaration allows other modules to reference the bus manager
 * without including the full definition, helping to avoid circular dependencies.
 */
typedef struct pstar_bus_manager pstar_bus_manager_t;

/**
 * @brief Forward declaration of pstar_bus_config_t structure
 */
typedef struct pstar_bus_config pstar_bus_config_t;

/**
 * @brief Function declarations for bus manager operations
 *
 * These declarations allow code to reference the bus manager functions
 * without including the full implementation, helping to avoid circular dependencies.
 */
esp_err_t pstar_bus_manager_init(pstar_bus_manager_t* manager, const char* tag);
esp_err_t pstar_bus_manager_add_bus(pstar_bus_manager_t* manager, pstar_bus_config_t* config);
pstar_bus_config_t* pstar_bus_manager_find_bus(const pstar_bus_manager_t* manager,
                                               const char*                name);
esp_err_t           pstar_bus_manager_remove_bus(pstar_bus_manager_t* manager, const char* name);
esp_err_t           pstar_bus_manager_deinit(pstar_bus_manager_t* manager);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_MANAGER_FWD_H */
