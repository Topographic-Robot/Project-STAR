/* main/include/system_error_handler.h */

#ifndef TOPOROBO_SYSTEM_ERROR_HANDLER_H
#define TOPOROBO_SYSTEM_ERROR_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "error_handler.h"

/* Constants ******************************************************************/

extern const char* const system_error_tag; /**< Tag for system error handler */

/* Globals ********************************************************************/

extern error_handler_t g_system_error_handler; /**< Global system error handler */

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the system-level error handler
 * 
 * This function initializes the global system error handler with appropriate
 * recovery strategies and callbacks. It should be called after error_handler_system_init()
 * but before any other component initialization.
 * 
 * @return ESP_OK if initialization succeeded, error code otherwise
 */
esp_err_t system_error_handler_init(void);

/**
 * @brief Callback function for handling system-level errors
 * 
 * This function is called when a system-level error occurs. It logs the error
 * and takes appropriate action based on the error severity.
 * 
 * @param error_info Information about the error
 * @param context    Context pointer (unused)
 * @return ESP_OK if handling succeeded, error code otherwise
 */
esp_err_t system_error_callback(error_info_t* error_info, void* context);

/**
 * @brief Callback function for handling permanent failures
 * 
 * This function is called when a component has permanently failed. It logs the
 * failure and initiates a system restart if necessary.
 * 
 * @param error_info Information about the error
 * @param context    Context pointer (unused)
 * @return ESP_OK if handling succeeded, error code otherwise
 */
esp_err_t system_failure_callback(error_info_t* error_info, void* context);

/**
 * @brief Recovery function for system-level errors
 * 
 * This function implements the recovery strategy for system-level errors.
 * It may restart components, reconfigure hardware, or take other actions
 * to recover from errors.
 * 
 * @param handler Pointer to the error handler
 * @param context Context pointer (unused)
 * @return ESP_OK if recovery succeeded, error code otherwise
 */
esp_err_t system_recovery_function(error_handler_t* handler, void* context);

/**
 * @brief Cleans up the system-level error handler
 * 
 * This function cleans up the global system error handler. It should be called
 * before error_handler_system_cleanup().
 * 
 * @return ESP_OK if cleanup succeeded, error code otherwise
 */
esp_err_t system_error_handler_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_SYSTEM_ERROR_HANDLER_H */ 