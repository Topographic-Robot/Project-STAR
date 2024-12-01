/* main/include/managers/include/time_manager.h */

#ifndef TOPOROBO_TIME_MANAGER_H
#define TOPOROBO_TIME_MANAGER_H

#include "esp_err.h"

/* Macros *********************************************************************/

#define max_file_path_length (64)  /* Maximum file path length */
#define max_data_length      (256) /* Maximum data length per write */

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the SNTP service and synchronizes the ESP32's system time.
 *
 * This function sets up SNTP (Simple Network Time Protocol) to retrieve the
 * current date and time from an NTP server. If synchronization with the NTP
 * server fails (e.g., no Wi-Fi), it falls back to manually setting the time
 * to a default value ("beginning of time").
 *
 * The synchronized time is used for logging and timestamping operations
 * throughout the application.
 *
 * @return
 * - ESP_OK if time synchronization is successful.
 * - ESP_FAIL if synchronization fails and the fallback is used.
 *
 * @note Call this function once during system initialization.
 */
esp_err_t time_manager_initialize(void);

#endif /* TOPOROBO_TIME_MANAGER_H */

