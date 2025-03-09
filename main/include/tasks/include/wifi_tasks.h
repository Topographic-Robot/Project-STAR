/* main/include/tasks/include/wifi_tasks.h */

#ifndef TOPOROBO_WIFI_TASKS_H
#define TOPOROBO_WIFI_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

/* NOTE: check wifi_credentials.txt is included, but wifi_credentials.h isn't
 * copy wifi_credentials.txt to wifi_credentials.h and replace the values */
#include <stdbool.h>
#include "wifi_credentials.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

/* Constants ******************************************************************/

/* The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT (BIT0) /**< Wifi is Connected */
#define WIFI_FAIL_BIT      (BIT1) /**< Wifi failed to connect */

extern const char* const wifi_tag;                /**< Tag for logging */
extern const uint8_t     wifi_max_retry;          /**< The max number of times to try and connect to the station */
extern const uint8_t     wifi_ssid_max_len;       /**< The max length for wifi's SSID defined by esp */
extern const uint8_t     wifi_pass_max_len;       /**< The max length for wifi's password defined by esp */
extern const uint32_t    wifi_connect_timeout_ms; /**< Timeout for WiFi connection in milliseconds */

/* Structs ********************************************************************/

/**
 * @brief Structure to hold configuration for WiFi task.
 *
 * Represents the WiFi task's configuration, including its priority,
 * stack size, and enablement flag.
 */
typedef struct {
  UBaseType_t priority;    /**< Priority of the WiFi task for scheduling purposes. */
  uint32_t    stack_depth; /**< Stack depth allocated for the WiFi task, in words. */
  bool        enabled;     /**< Flag indicating if the WiFi task is enabled (true) or disabled (false). */
} wifi_task_config_t;

/* Public Functions ***********************************************************/

/**
 * @brief Checks if the system has a valid IP address and can reach the internet.
 *
 * Verifies the current network status by checking if the WiFi station (STA)
 * interface has a valid IP address assigned. This function can be used to
 * confirm internet connectivity before attempting network operations.
 *
 * @return 
 * - ESP_OK   if the network is up and a valid IP address is assigned.
 * - ESP_FAIL if the interface does not exist or no IP address is assigned.
 *
 * @note 
 * - Ensure that WiFi is initialized and connected to an Access Point (AP) 
 *   before calling this function.
 * - This function does not test actual internet connectivity, only the 
 *   presence of a valid IP address.
 */
esp_err_t wifi_check_connection(void);

/**
 * @brief Initializes the WiFi in station mode and connects to the specified Access Point (AP).
 *
 * Sets up the ESP32's WiFi system to operate in station (STA) mode and attempts
 * to connect to a preconfigured Access Point (AP). The initialization process
 * includes setting up the required event handlers, configuring the network
 * interface, and managing the connection lifecycle. Upon successful connection,
 * the system acquires an IP address for further network operations.
 *
 * @return 
 * - ESP_OK   if the WiFi is initialized and connected successfully.
 * - ESP_FAIL if any step in the initialization or connection process fails.
 *
 * @note 
 * - Ensure the SSID and password are correctly configured before calling this function.
 * - This function blocks until a connection is established or a failure occurs.
 * - Check the logs for detailed information about the connection status and errors.
 */
esp_err_t wifi_init_sta(void);

/**
 * @brief Starts the WiFi task that handles connection and monitoring.
 *
 * Creates and starts a FreeRTOS task that manages WiFi connection, including:
 * - Initial connection attempt
 * - Time synchronization after connection
 * - Connection monitoring and automatic reconnection
 *
 * @return
 * - ESP_OK if the task was created successfully
 * - ESP_FAIL if task creation failed
 */
esp_err_t wifi_task_start(void);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_WIFI_TASKS_H */
