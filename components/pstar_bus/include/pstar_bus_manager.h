/* components/pstar_bus/include/pstar_bus_manager.h */

#ifndef PSTAR_COMPONENT_BUS_MANAGER_H
#define PSTAR_COMPONENT_BUS_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_manager_types.h"

#include "esp_err.h"

/* --- Public Functions --- */

/**
 * @brief Initialize an I2C bus manager instance.
 *
 * Allocates resources (like a mutex) for the manager. Must be called before
 * any other manager functions.
 *
 * @param[out] manager Pointer to a bus manager structure to initialize. Must not be NULL.
 * @param[in]  tag     Optional tag string for logging associated with this manager instance. If NULL or empty, a default tag is used.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if manager is NULL, ESP_ERR_NO_MEM if mutex creation fails.
 */
esp_err_t pstar_bus_manager_init(pstar_bus_manager_t* manager, const char* tag);

/**
 * @brief Add a new I2C bus/device configuration to the manager.
 *
 * Adds a previously created configuration (via pstar_bus_config_create_i2c)
 * to the manager's internal list. The manager takes ownership conceptually,
 * but the configuration still needs to be destroyed eventually (typically via
 * pstar_bus_manager_remove_bus or pstar_bus_manager_deinit).
 * The configuration must have a unique name.
 * This function is thread-safe.
 *
 * @param[in] manager Pointer to the initialized bus manager. Must not be NULL.
 * @param[in] config  Pointer to the I2C bus configuration structure to add. Must not be NULL and must have a valid name.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if manager or config is invalid, ESP_ERR_INVALID_STATE if a bus with the same name already exists or mutex error, ESP_ERR_TIMEOUT if mutex times out.
 */
esp_err_t pstar_bus_manager_add_bus(pstar_bus_manager_t* manager, pstar_bus_config_t* config);

/**
 * @brief Find an I2C bus configuration by its unique name.
 *
 * Searches the manager's list for a configuration matching the given name.
 * This function is thread-safe.
 *
 * @param[in] manager Pointer to the initialized bus manager. Can be NULL (returns NULL).
 * @param[in] name    The unique name of the bus configuration to find. Can be NULL (returns NULL).
 * @return pstar_bus_config_t* Pointer to the found configuration structure, or NULL if not found or an error occurs (e.g., mutex timeout). The returned pointer is still managed by the bus manager.
 */
pstar_bus_config_t* pstar_bus_manager_find_bus(const pstar_bus_manager_t* manager,
                                               const char*                name);

/**
 * @brief Remove an I2C bus configuration from the manager and destroy it.
 *
 * Finds the configuration by name, removes it from the manager's list,
 * deinitializes the associated I2C driver (if initialized), and frees the
 * configuration structure memory (via pstar_bus_config_destroy).
 * This function is thread-safe (acquires mutex initially, releases before deinit/destroy).
 *
 * @param[in] manager Pointer to the initialized bus manager. Must not be NULL.
 * @param[in] name    The unique name of the bus configuration to remove and destroy. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if manager or name is invalid, ESP_ERR_NOT_FOUND if bus not found, ESP_ERR_TIMEOUT if mutex times out, or an error code from the underlying deinitialization/destroy process.
 */
esp_err_t pstar_bus_manager_remove_bus(pstar_bus_manager_t* manager, const char* name);

/**
 * @brief Deinitialize the bus manager and all managed I2C bus configurations.
 *
 * Iterates through all I2C bus configurations currently managed, deinitializes
 * each one (if initialized), destroys each configuration structure (freeing memory),
 * and finally releases the manager's resources (mutex).
 * After this call, the manager and all its previously managed configurations are invalid.
 *
 * @param[in] manager Pointer to the bus manager to deinitialize. Must not be NULL.
 * @return esp_err_t ESP_OK if all deinitializations and cleanup were successful, ESP_ERR_INVALID_ARG if manager is NULL, or the first error code encountered during the deinitialization/destroy process of any bus.
 */
esp_err_t pstar_bus_manager_deinit(pstar_bus_manager_t* manager);

/**
 * @brief Iterates through all I2C buses/devices managed by the manager and calls a
 *        user-provided function to register each configured pin (SDA, SCL).
 *
 * This allows decoupling the bus manager from a specific pin validation/tracking system.
 * The user provides a callback function that conforms to `pin_registration_function_t`.
 * This function will be called for the SDA and SCL pins associated with each I2C
 * configuration in the manager.
 * This function is thread-safe (acquires mutex during iteration).
 *
 * @param[in] manager  Pointer to the initialized bus manager. Must not be NULL.
 * @param[in] reg_func The user's pin registration callback function. Must not be NULL.
 * @param[in] user_ctx A user context pointer to be passed to the registration function.
 * @return esp_err_t ESP_OK if iteration completed (doesn't guarantee all registrations succeeded, check reg_func's return), ESP_ERR_INVALID_ARG if manager or reg_func is NULL, ESP_ERR_TIMEOUT if mutex times out. Returns the first error reported by the callback.
 */
esp_err_t pstar_bus_manager_register_all_pins(const pstar_bus_manager_t*  manager,
                                              pin_registration_function_t reg_func,
                                              void*                       user_ctx);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_BUS_MANAGER_H */
