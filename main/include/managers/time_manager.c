/* main/include/managers/time_manager.c */

#include "time_manager.h"
#include <time.h>
#include <sys/time.h>
#include "wifi_tasks.h"
#include "esp_sntp.h"
#include "esp_log.h"

/* Globals (Constants) ********************************************************/

const char *time_manager_tag = "TIME_MANAGER";

/* Private Functions **********************************************************/

/**
 * @brief Initializes the SNTP service for time synchronization.
 *
 * Sets up SNTP to poll the NTP server for time updates. The default NTP
 * server is "pool.ntp.org". SNTP operates in polling mode, periodically
 * refreshing the system clock.
 */
static void priv_initialize_sntp(void)
{
  ESP_LOGI(time_manager_tag, "Initializing SNTP");

  /* Set SNTP operating mode (polling mode) */
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);

  /* Set the NTP server */
  esp_sntp_setservername(0, "pool.ntp.org");

  /* Initialize SNTP */
  esp_sntp_init();

  ESP_LOGI(time_manager_tag, "SNTP initialization complete");
}

/**
 * @brief Sets the system time to a default value if network or NTP sync fails.
 *
 * This function provides a fallback time setting in scenarios where
 * the Wi-Fi connection is not available, or NTP synchronization
 * cannot complete. It sets the system clock to January 1st, 2023, 00:00:00.
 */
static void priv_set_default_time(void)
{
  ESP_LOGW(time_manager_tag, "Setting time to default values.");

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
  ESP_LOGI(time_manager_tag, "Time set to default: %s", time_str);
}

/* Public Functions ***********************************************************/

esp_err_t time_manager_init(void)
{
  /* First, check the Wi-Fi connection. If it's unavailable, fallback to default time. */
  if (wifi_check_connection() != ESP_OK) {
    ESP_LOGE(time_manager_tag, "Network not available. Using default time.");
    priv_set_default_time();
    return ESP_FAIL; 
  }

  /* If we have Wi-Fi, proceed with SNTP initialization. */
  priv_initialize_sntp();

  /* Wait for NTP synchronization */
  time_t    now         = 0;
  struct tm timeinfo    = { 0 };
  int       retry       = 0;
  const int max_retries = 10;

  while (timeinfo.tm_year < (2023 - 1900) && ++retry < max_retries) {
    ESP_LOGI(time_manager_tag, "Waiting for system time to be set... (%d/%d)", retry, max_retries);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    time(&now);
    localtime_r(&now, &timeinfo);
  }

  /* If time is still not set after retries, fallback to default. */
  if (timeinfo.tm_year < (2023 - 1900)) {
    ESP_LOGW(time_manager_tag, "NTP sync failed after multiple attempts. Setting default time.");
    priv_set_default_time();
    return ESP_FAIL;
  }

  ESP_LOGI(time_manager_tag, "Time synchronized: %s", asctime(&timeinfo));
  return ESP_OK;
}

