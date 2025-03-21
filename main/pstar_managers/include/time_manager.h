/* main/pstar_managers/include/time_manager.h */

#ifndef PSTAR_TIME_MANAGER_H
#define PSTAR_TIME_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include <stdbool.h>

/* Macros *********************************************************************/

#define TIME_STRFTIME_BUFFER_SIZE (64) /**< Size of buffer for formatted time strings */

/* Public Functions ***********************************************************/

/**
 * @brief Checks if the system time has been properly initialized.
 *
 * The system time is considered initialized if either:
 * - NTP synchronization was successful
 * - A default time was manually set
 *
 * @return true if time is initialized, false otherwise
 */
bool time_manager_is_initialized(void);

/**
 * @brief Formats the current date and time as a string.
 *
 * Retrieves the current system time and formats it as `YYYY-MM-DD HH:MM:SS`.
 * The system time is synchronized with NTP if WiFi is available, otherwise
 * it uses the default time set during initialization.
 *
 * @return A dynamically allocated string containing the formatted timestamp.
 *         The caller is responsible for freeing this memory using free().
 *         Returns NULL if memory allocation fails.
 *
 * @note 
 * - The returned string must be freed by the caller to avoid memory leaks.
 * - The system time must be properly initialized before calling this function.
 */
char* time_manager_get_timestamp(void);

/**
 * @brief Initializes the SNTP service and synchronizes the ESP32's system time.
 *
 * Sets up the Simple Network Time Protocol (SNTP) service to retrieve the 
 * current date and time from an NTP server. If synchronization with the NTP 
 * server fails (e.g., due to lack of Wi-Fi connectivity), the system falls 
 * back to manually setting the time to a predefined default value ("beginning 
 * of time"). The synchronized system time is critical for logging and 
 * timestamping operations across the application.
 *
 * @return
 * - ESP_OK   if the system time is successfully synchronized with an NTP server.
 * - ESP_FAIL if synchronization fails and the default fallback time is set.
 *
 * @note
 * - Call this function once during the system initialization phase.
 * - Ensure Wi-Fi connectivity is established before invoking this function 
 *   for successful NTP synchronization.
 */
esp_err_t time_manager_init(void);

/**
 * @brief Cleans up resources used by the time manager module.
 *
 * Stops the SNTP service and performs cleanup of resources allocated during
 * time manager initialization. This function should be called during system
 * shutdown to ensure proper cleanup of the time synchronization subsystem.
 *
 * @return
 * - ESP_OK   if all resources are successfully cleaned up.
 * - ESP_FAIL if any cleanup operation fails.
 *
 * @note
 * - Call this function during the system shutdown phase.
 * - This function should be called after all other components that depend on
 *   time services have been stopped.
 */
esp_err_t time_manager_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_TIME_MANAGER_H */
