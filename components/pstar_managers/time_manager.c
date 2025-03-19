/* components/pstar_managers/time_manager.c */

#include "time_manager.h"
#include "log_handler.h"
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "esp_err.h"
#include "esp_sntp.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

/* Constants ******************************************************************/

static const char* const time_manager_tag     = "Time Manager";
static const uint32_t    time_sync_bit        = (1 << 0);
static const uint32_t    time_sync_timeout_ms = CONFIG_PSTAR_KCONFIG_TIME_SYNC_TIMEOUT_MS;

/* Globals (Static) ***********************************************************/

static bool               s_time_initialized      = false; /**< Flag indicating if time is initialized */
static EventGroupHandle_t s_time_sync_event_group = NULL;
static TaskHandle_t       s_time_sync_task_handle = NULL;

/* Private Functions **********************************************************/
/**
 * @brief Callback function called when time is synchronized with NTP server
 *
 * @param tv Pointer to timeval structure containing the synchronized time
 */
static void priv_time_sync_notification_cb(struct timeval *tv)
{
  if (s_time_sync_event_group) {
    xEventGroupSetBits(s_time_sync_event_group, time_sync_bit);
  }

  char      strftime_buf[64];
  time_t    now = tv->tv_sec;
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);

  log_info(time_manager_tag,
           "Time Synced",
           "Time synchronized: %s",
           strftime_buf);
}

/**
 * @brief Sets the system time to a default value as a fallback.
 *
 * Sets the system clock to January 1st, 2025, 00:00:00 in cases where Wi-Fi
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

  /* Set to January 1st, 2025 */
  tm.tm_year  = 2025 - 1900;  /* Years since 1900 (2025 - 1900 = 125) */
  tm.tm_mon   = 0;            /* January (0-based month) */
  tm.tm_mday  = 1;            /* Day must be at least 1 to be valid */
  tm.tm_isdst = -1;           /* Not considering daylight saving time */

  /* Convert tm to time_t and set timeval */
  time_t time_value = mktime(&tm);
  tv.tv_sec  = time_value;
  tv.tv_usec = 0; /* 0 microseconds */

  settimeofday(&tv, NULL);

  log_info(time_manager_tag,
           "Time Set",
           "System time set to default: 2025-01-01 00:00:00");
  s_time_initialized = true; /* Mark time as initialized even though it's default */
}

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

  /* Register the time sync notification callback */
  esp_sntp_set_time_sync_notification_cb(priv_time_sync_notification_cb);

  /* Initialize SNTP */
  esp_sntp_init();

  log_info(time_manager_tag,
           "SNTP Complete",
           "SNTP service initialized, awaiting time sync");
}

/**
 * @brief Task that handles time synchronization in the background
 *
 * @param pvParameters Task parameters (unused)
 */
static void priv_time_sync_task(void *pvParameters)
{
  log_info(time_manager_tag, "Sync Task", "Time synchronization task started");

  esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (netif == NULL || !esp_netif_is_netif_up(netif)) {
    log_warn(time_manager_tag,
             "Network Error",
             "Network unavailable, falling back to default time");
    priv_set_default_time();
    vTaskDelete(NULL);
    return;
  }

  priv_initialize_sntp();

  /* Wait for time sync or timeout */
  EventBits_t bits = xEventGroupWaitBits(s_time_sync_event_group,
                                         time_sync_bit,
                                         pdFALSE,
                                         pdFALSE,
                                         pdMS_TO_TICKS(time_sync_timeout_ms));

  if ((bits & time_sync_bit) == 0) {
    log_error(time_manager_tag,
              "Sync Error",
              "Time sync failed after timeout");
    priv_set_default_time();
  } else {
    log_info(time_manager_tag,
             "Init Complete",
             "Time manager initialization finished");
    s_time_initialized = true;
  }

  s_time_sync_task_handle = NULL;
  vTaskDelete(NULL);
}

/* Public Functions ***********************************************************/

bool time_manager_is_initialized(void)
{
  return s_time_initialized;
}

char* time_manager_get_timestamp(void)
{
  if (!s_time_initialized) {
    return NULL; /* Don't generate timestamps until time is properly initialized */
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

esp_err_t time_manager_init(void)
{
  log_info(time_manager_tag, "Init Start", "Beginning time manager initialization");

  /* Create event group for synchronization */
  s_time_sync_event_group = xEventGroupCreate();
  if (s_time_sync_event_group == NULL) {
    log_error(time_manager_tag,
              "Init Error",
              "Failed to create event group");
    return ESP_FAIL;
  }

  /* Start time synchronization task */
  BaseType_t task_created = xTaskCreate(priv_time_sync_task,
                                        "time_sync_task",
                                        4096,
                                        NULL,
                                        5,
                                        &s_time_sync_task_handle);

  if (task_created != pdPASS) {
    log_error(time_manager_tag,
              "Init Error",
              "Failed to create time sync task");
    vEventGroupDelete(s_time_sync_event_group);
    s_time_sync_event_group = NULL;
    return ESP_FAIL;
  }

  log_info(time_manager_tag,
           "Init Progress",
           "Time synchronization started in background");

  return ESP_OK;
}

esp_err_t time_manager_cleanup(void)
{
  log_info(time_manager_tag,
           "Cleanup Start",
           "Beginning time manager cleanup");

  /* Delete the time sync task if it's still running */
  if (s_time_sync_task_handle != NULL) {
    vTaskDelete(s_time_sync_task_handle);
    s_time_sync_task_handle = NULL;
  }

  /* Delete the event group */
  if (s_time_sync_event_group != NULL) {
    vEventGroupDelete(s_time_sync_event_group);
    s_time_sync_event_group = NULL;
  }

  /* Stop the SNTP service if it was initialized */
  if (s_time_initialized) {
    /* Stop the SNTP service */
    esp_sntp_stop();

    log_info(time_manager_tag,
             "SNTP Stop",
             "SNTP service stopped successfully");

    /* Reset the initialization flag */
    s_time_initialized = false;
  } else {
    log_info(time_manager_tag,
             "Cleanup Skip",
             "Time manager was not initialized, nothing to clean up");
  }

  log_info(time_manager_tag,
           "Cleanup Complete",
           "Time manager cleanup completed successfully");

  return ESP_OK;
}
