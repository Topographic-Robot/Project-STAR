/* components/pstar_managers/include/pstar_time_manager.h */

#ifndef PSTAR_TIME_MANAGER_H
#define PSTAR_TIME_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "sdkconfig.h"

/* Public Functions ***********************************************************/

/**
 * @brief Checks if the system time has been properly initialized (either via NTP or default).
 *
 * The system time is considered initialized if either:
 * - NTP synchronization was successful
 * - A default time was manually set
 *
 * @return true if time is initialized, false otherwise
 */
bool time_manager_is_initialized(void);

/**
 * @brief Formats the current date and time as a string into a provided buffer.
 *
 * Retrieves the current system time and formats it as `YYYY-MM-DD HH:MM:SS`
 * into the given buffer. The system time is synchronized with NTP if WiFi is
 * available and sync was successful, otherwise it uses the default time set
 * during initialization.
 *
 * @param[out] buffer Buffer to store the formatted timestamp string.
 * @param[in]  size   Size of the provided buffer. Should be at least 20 bytes.
 * @return
 *  - ESP_OK on success.
 *  - ESP_ERR_INVALID_STATE if the time manager is not initialized.
 *  - ESP_ERR_INVALID_ARG if buffer is NULL or size is too small.
 *  - ESP_FAIL if formatting fails internally.
 *
 * @note
 * - The system time must be properly initialized before calling this function.
 * - The buffer should be at least 20 bytes to hold "YYYY-MM-DD HH:MM:SS\0".
 */
esp_err_t time_manager_get_timestamp(char* buffer, size_t size);

/**
 * @brief Initializes the SNTP service and synchronizes the ESP32's system time.
 *
 * Sets up the Simple Network Time Protocol (SNTP) service to retrieve the
 * current date and time from an NTP server. This function starts a background
 * task to handle synchronization. If synchronization with the NTP
 * server fails (e.g., due to lack of Wi-Fi connectivity within the timeout),
 * the system falls back to manually setting the time to a predefined default
 * value ("beginning of time"). The synchronized system time is critical for
 * logging and timestamping operations across the application.
 *
 * @return
 * - ESP_OK   if initialization tasks are successfully started.
 * - ESP_FAIL if task creation or event group creation fails.
 * - ESP_ERR_NO_MEM if memory allocation fails.
 *
 * @note
 * - Call this function once during the system initialization phase.
 * - Actual time synchronization happens asynchronously in the background.
 * - Use time_manager_is_initialized() to check if synchronization has completed
 *   or the fallback time has been set.
 */
esp_err_t time_manager_init(void);

/**
 * @brief Cleans up resources used by the time manager module.
 *
 * Stops the SNTP service, deletes the background task (if running), and
 * performs cleanup of resources allocated during time manager initialization
 * (event group). This function should be called during system shutdown.
 *
 * @return
 * - ESP_OK   if all resources are successfully cleaned up.
 * - ESP_FAIL if any cleanup operation fails (currently always returns ESP_OK).
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
