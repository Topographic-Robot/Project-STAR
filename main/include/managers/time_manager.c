/* main/include/managers/time_manager.c */

#include "time_manager.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_netif.h"
#include <time.h>
#include <sys/time.h>
#include "wifi_tasks.h"

/* Constants ******************************************************************/

const char *time_manager_tag = "TIME_MANAGER";

/* Private Functions **********************************************************/

/**
 * @brief Initializes the SNTP service for time synchronization.
 *
 * Configures SNTP to synchronize the system clock with an NTP server. By default, 
 * it uses "pool.ntp.org" and operates in polling mode to periodically refresh the time.
 *
 * @note 
 * - Ensure network connectivity before calling this function.
 * - The system time will remain unsynchronized if SNTP initialization fails.
 */
static void priv_initialize_sntp(void)
{
  ESP_LOGI(time_manager_tag, "- Starting SNTP initialization - configuring time sync");

  /* Set SNTP operating mode (polling mode) */
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);

  /* Set the NTP server */
  esp_sntp_setservername(0, "pool.ntp.org");

  /* Initialize SNTP */
  esp_sntp_init();

  ESP_LOGI(time_manager_tag, "- SNTP initialization complete - awaiting time sync");
}

/**
 * @brief Sets the system time to a default value as a fallback.
 *
 * Sets the system clock to January 1st, 2023, 00:00:00 in cases where Wi-Fi 
 * is unavailable or NTP synchronization fails.
 *
 * @note 
 * - Use this function only as a fallback when time synchronization cannot complete.
 */
static void priv_set_default_time(void)
{
  ESP_LOGW(time_manager_tag, "- Using default time - network sync unavailable");

  struct timeval tv;
  struct tm      tm;

  tm.tm_year  = 2025 - 1900; /* Year since 1900 */
  tm.tm_mon   = 0;           /* January */
  tm.tm_mday  = 1;           /* Day 1 */
  tm.tm_hour  = 0;           /* Midnight */
  tm.tm_min   = 0;           /* Zero minutes */
  tm.tm_sec   = 0;           /* Zero seconds */
  tm.tm_isdst = -1;          /* Not considering daylight saving time */

  tv.tv_sec  = mktime(&tm); /* Convert to seconds since epoch */
  tv.tv_usec = 0;

  settimeofday(&tv, NULL);
  char *time_str = asctime(&tm);
  /* Remove the trailing newline added by asctime */
  time_str[strcspn(time_str, "\n")] = '\0'; 
  
  ESP_LOGI(time_manager_tag, "- Default time set - system date: 2024-01-01 00:00:00");
}

/* Public Functions ***********************************************************/

/* TODO: Make this non-blocking */
esp_err_t time_manager_init(void)
{
  ESP_LOGI(time_manager_tag, "- Starting time manager initialization");
  
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (netif == NULL || !esp_netif_is_netif_up(netif)) {
    ESP_LOGW(time_manager_tag, "- Network unavailable - falling back to default time");
    priv_set_default_time();
    return ESP_OK;
  }
  
  priv_initialize_sntp();

  /* Wait for NTP synchronization */
  time_t    now         = 0;
  struct tm timeinfo    = { 0 };
  int       retry       = 0;
  const int max_retries = 10;
  while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < max_retries) {
    ESP_LOGI(time_manager_tag, "- Waiting for time sync - attempt %u/%u", retry, max_retries);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
  if (retry == max_retries) {
    ESP_LOGW(time_manager_tag, "- Time sync failed - maximum retries reached");
    priv_set_default_time();
    return ESP_OK;
  }
  
  char strftime_buf[64];
  
  time(&now);
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  
  ESP_LOGI(time_manager_tag, "- Time sync successful - current time: %s", strftime_buf);
  ESP_LOGI(time_manager_tag, "- Time manager initialization complete - system clock ready");
  
  return ESP_OK;
}

