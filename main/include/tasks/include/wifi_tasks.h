/* main/include/tasks/include/wifi_tasks.h */

#ifndef TOPOROBO_WIFI_TASKS_H
#define TOPOROBO_WIFI_TASKS_H

/* NOTE: check wifi_credentials.txt is included, but wifi_credentials.h isn't
 * copy wifi_credentials.txt to wifi_credentials.h and replace the values */
#include "wifi_credentials.h"
#include <esp_err.h>

/* Constants ******************************************************************/

/* The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

extern const char    *wifi_tag;          /**< Tag for logging */
extern const uint8_t  wifi_max_retry;    /**< The max number of times to try and connect to the station */
extern const uint8_t  wifi_ssid_max_len; /**< The max length for wifi's SSID defined by esp */
extern const uint8_t  wifi_pass_max_len; /**< The max legnth for wifi's password defined by esp */


/* Public Functions ***********************************************************/

/**
 * @brief Initializes the WiFi in station mode and connects to the specified Access Point (AP).
 * 
 * This function performs the following steps:
 *   1. Creates an event group to manage WiFi events.
 *   2. Initializes the underlying TCP/IP stack.
 *   3. Creates the default event loop required for handling events.
 *   4. Sets up the default WiFi station (STA) network interface.
 *   5. Initializes the WiFi driver with the default configuration.
 *   6. Registers event handlers for WiFi events and IP events to handle
 *      connection status and IP acquisition.
 *   7. Configures the WiFi with the specified SSID and password, including
 *      authentication mode and SAE settings.
 *   8. Sets the WiFi mode to station (STA) mode.
 *   9. Starts the WiFi driver.
 *   10. Waits for the WiFi connection to be established or to fail, and logs
 *       the connection status.
 */
esp_err_t wifi_init_sta(void);

#endif /* TOPOROBO_WIFI_TASKS_H */
