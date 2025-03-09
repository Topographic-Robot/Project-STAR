/* main/include/managers/time_manager.c */

#include "time_manager.h"
#include "esp_sntp.h"
#include "esp_netif.h"
#include <time.h>
#include <sys/time.h>
#include "wifi_tasks.h"
#include "log_handler.h"

/* Constants ******************************************************************/

const char* const time_manager_tag = "Time Manager";

/* Globals (Static) ***********************************************************/

static bool s_time_initialized = false; /*< Flag indicating if time is initialized */

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
  log_info(time_manager_tag, "SNTP Start", "Beginning SNTP service initialization");

  /* Set SNTP operating mode (polling mode) */
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);

  /* Set the NTP server */
  esp_sntp_setservername(0, "pool.ntp.org");

  /* Initialize SNTP */
  esp_sntp_init();

  log_info(time_manager_tag, 
           "SNTP Complete", 
           "SNTP service initialized, awaiting time sync");
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
  log_warn(time_manager_tag, 
           "Default Time", 
           "Using default time due to unavailable network sync");

  struct timeval tv;
  struct tm      tm;

  /* Set all time values to 0 */
  memset(&tm, 0, sizeof(tm));
  tm.tm_mday  = 1;  /* Day must be at least 1 to be valid */
  tm.tm_isdst = -1; /* Not considering daylight saving time */

  /* Set the actual time values to 0 */
  tv.tv_sec  = 0; /* 0 seconds since epoch */
  tv.tv_usec = 0; /* 0 microseconds */

  settimeofday(&tv, NULL);
  
  log_info(time_manager_tag, 
           "Time Set", 
           "System time set to default: 2025-01-01 00:00:00");
  s_time_initialized = true; /* Mark time as initialized even though it's default */
}

/* Public Functions ***********************************************************/

bool time_manager_is_initialized(void)
{
  return s_time_initialized;
}

char *time_manager_get_timestamp(void)
{
  if (!s_time_initialized) {
    return NULL;  /* Don't generate timestamps until time is properly initialized */
  }

  time_t    now = time(NULL);
  struct tm timeinfo;
  char*     buffer;

  /* Allocate memory for the timestamp string (YYYY-MM-DD HH:MM:SS\0 = 20 bytes) */
  buffer = malloc(20);
  if (buffer == NULL) {
    return NULL;  /* Simply return NULL on allocation failure */
  }

  localtime_r(&now, &timeinfo);
  strftime(buffer, 20, "%Y-%m-%d %H:%M:%S", &timeinfo);
  return buffer;
}

/* TODO: Make this non-blocking */
esp_err_t time_manager_init(void)
{
  log_info(time_manager_tag, "Init Start", "Beginning time manager initialization");
  
  esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (netif == NULL || !esp_netif_is_netif_up(netif)) {
    log_warn(time_manager_tag, 
             "Network Error", 
             "Network unavailable, falling back to default time");
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
    log_info(time_manager_tag, 
             "Sync Status", 
             "Waiting for time sync (attempt %u/%u)", 
             retry, 
             max_retries);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
  if (retry == max_retries) {
    log_error(time_manager_tag, 
              "Sync Error", 
              "Time sync failed after maximum retries");
    priv_set_default_time();
    return ESP_OK;
  }
  
  char strftime_buf[TIME_STRFTIME_BUFFER_SIZE];
  
  time(&now);
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  
  log_info(time_manager_tag, 
           "Sync Complete", 
           "Time synchronized successfully: %s", 
           strftime_buf);
  log_info(time_manager_tag, 
           "Init Complete", 
           "Time manager initialization finished");
  
  s_time_initialized = true;  /* Mark time as initialized after successful NTP sync */
  return ESP_OK;
}

