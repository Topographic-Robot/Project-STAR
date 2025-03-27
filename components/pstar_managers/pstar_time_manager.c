/* components/pstar_managers/time_manager.c */

#include "pstar_time_manager.h"
#include "pstar_log_handler.h"
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h> /* For memset */
#include "esp_system.h"
#include "esp_err.h"
#include "esp_sntp.h"
#include "esp_netif.h"
#include "esp_event.h" /* For event loop used by SNTP/Netif */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <stdatomic.h> /* For atomic bool */
#include "sdkconfig.h" /* For Kconfig values */

/* Constants ******************************************************************/

/* Fix: Use TAG consistently */
static const char* TAG = "Time Manager"; /* Renamed TIME_MANAGER_TAG */
/* #define TIME_SYNC_BIT    (1 << 0) /* Use Kconfig value */ */
#define TIME_SYNC_BIT    (CONFIG_PSTAR_KCONFIG_TIME_SYNC_BIT)


/* Globals (Static) ***********************************************************/

static _Atomic bool       s_time_initialized      = false; /**< Flag indicating if time is initialized */
static EventGroupHandle_t s_time_sync_event_group = NULL;
static TaskHandle_t       s_time_sync_task_handle = NULL;
static bool               s_sntp_initialized      = false; /* Track SNTP service state */

/* Private Functions **********************************************************/
/**
 * @brief Callback function called when time is synchronized with NTP server
 *
 * @param tv Pointer to timeval structure containing the synchronized time
 */
static void priv_time_sync_notification_cb(struct timeval *tv)
{
  /* Mark time as initialized once synced */
  atomic_store(&s_time_initialized, true);

  if (s_time_sync_event_group) {
    xEventGroupSetBits(s_time_sync_event_group, TIME_SYNC_BIT);
  } else { /* Use log_warn */
      /* Event group might be deleted during cleanup race condition */
      log_warn(TAG, "Sync CB Warning", "Event group NULL in sync callback");
  }

  char      strftime_buf[CONFIG_PSTAR_KCONFIG_TIME_STRFTIME_BUFFER_SIZE]; /* Use Kconfig size */
  time_t    now = tv->tv_sec;
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);

  /* Fix: Use TAG */
  log_info(TAG, /* Use log_info */
           "Time Synced",
           "Time synchronized via NTP: %s",
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
  /* Avoid setting default time if already initialized (e.g., by NTP) */
  if (atomic_load(&s_time_initialized)) {
      return;
  }

  log_warn(TAG, /* Use log_warn */
           "Default Time Set", /* Changed log message slightly */
           "Setting default time (2025-01-01) due to unavailable network sync");

  struct timeval tv;
  struct tm      tm;

  /* Set all time values to 0 */
  memset(&tm, 0, sizeof(tm));

  /* Set to January 1st, 2025 */
  tm.tm_year  = 2025 - 1900;  /* Years since 1900 */
  tm.tm_mon   = 0;            /* January (0-based month) */
  tm.tm_mday  = 1;            /* Day */
  /* tm.tm_hour = 0; /* Already 0 from memset */ */
  /* tm.tm_min = 0; */
  /* tm.tm_sec = 0; */
  tm.tm_isdst = -1;           /* Let system determine DST if applicable (usually not for embedded) */

  /* Convert tm to time_t and set timeval */
  time_t time_value = mktime(&tm);
  tv.tv_sec = time_value;
  tv.tv_usec = 0;

  /* Use settimeofday to set system time */
  if (settimeofday(&tv, NULL) != 0) {
      log_error(TAG, "Default Time Error", "Failed to set default system time!"); /* Use log_error */
      /* Cannot mark as initialized if setting failed */
  } else {
      log_info(TAG, "Default Time Set", "System time set to default: 2025-01-01 00:00:00"); /* Use log_info */
      /* Mark time as initialized even though it's default */
      atomic_store(&s_time_initialized, true);
  }
}

/**
 * @brief Initializes the SNTP service for time synchronization.
 *
 * Configures SNTP to synchronize the system clock with an NTP server.
 *
 * @note
 * - Ensure network connectivity before calling this function.
 * - The system time will remain unsynchronized if SNTP initialization fails.
 */
static void priv_initialize_sntp(void)
{
  /* Check if already initialized */
  if (s_sntp_initialized) {
      log_info(TAG, "SNTP Info", "SNTP service already initialized."); /* Use log_info */
      /* Optionally restart it? */
      /* esp_sntp_stop(); /* Stop first if re-init is desired */ */
      /* esp_sntp_init(); */
      return;
  }

  log_info(TAG, /* Use log_info */
           "SNTP Start",
           "Initializing SNTP service (Server: %s)",
           CONFIG_PSTAR_KCONFIG_TIME_NTP_SERVER);

  /* Set SNTP operating mode (polling mode) */
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);

  /* Set the NTP server from Kconfig */
  esp_sntp_setservername(0, CONFIG_PSTAR_KCONFIG_TIME_NTP_SERVER);

  /* Register the time sync notification callback */
  sntp_set_time_sync_notification_cb(priv_time_sync_notification_cb); /* Use correct function name */

  /* Initialize SNTP */
  esp_sntp_init();
  s_sntp_initialized = true;

  log_info(TAG, /* Use log_info */
           "SNTP Initialized", /* Changed log message */
           "SNTP service initialized, awaiting first time sync");
}

/**
 * @brief Task that handles time synchronization in the background
 *
 * @param pvParameters Task parameters (unused)
 */
static void priv_time_sync_task(void *pvParameters)
{
  log_info(TAG, "Sync Task Started", "Time synchronization task running"); /* Use log_info */

  bool network_ready = false;
  /* Fix: Use event loop or wait for IP address instead of simple check */
  /* Get default interface (can be STA or ETH depending on config) */
  esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"); /* Assume default STA for now */
  if (!netif) {
      netif = esp_netif_get_handle_from_ifkey("ETH_DEF"); /* Try Ethernet if STA not found */
  }


  /* Wait for network connection with timeout */
  TickType_t start_wait = xTaskGetTickCount();
  TickType_t max_wait_ticks = pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_TIME_SYNC_TIMEOUT_MS); /* Use Kconfig timeout */

  log_info(TAG, "Network Check", "Waiting up to %lu ms for network connection...", CONFIG_PSTAR_KCONFIG_TIME_SYNC_TIMEOUT_MS); /* Use log_info */

  while(xTaskGetTickCount() - start_wait < max_wait_ticks) {
       if (netif != NULL) {
           esp_netif_ip_info_t ip_info;
           /* Check if IP is acquired (for STA/ETH) */
           if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
               network_ready = true;
               log_info(TAG, "Network Ready", "Network interface is up with IP address."); /* Use log_info */
               break;
           }
       } else {
            /* Maybe netif handle wasn't found yet, try getting it again */
            netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            if (!netif) netif = esp_netif_get_handle_from_ifkey("ETH_DEF");
       }
       vTaskDelay(pdMS_TO_TICKS(500)); /* Check every 500ms */
  }


  if (!network_ready) {
    log_warn(TAG, /* Use log_warn */
             "Network Error",
             "Network unavailable after timeout (%lu ms). Setting default time.",
              CONFIG_PSTAR_KCONFIG_TIME_SYNC_TIMEOUT_MS);
    priv_set_default_time();
    s_time_sync_task_handle = NULL; /* Clear handle before deleting self */
    vTaskDelete(NULL);
    return;
  }

  /* Network is ready, initialize SNTP */
  priv_initialize_sntp();

  /* Wait for time sync notification or timeout */
  /* Use a slightly shorter wait here as SNTP itself might take time */
  /* Use Kconfig timeout directly, callback will set the bit when done. */
  TickType_t sntp_wait_ticks = pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_TIME_SYNC_TIMEOUT_MS);
  EventBits_t bits = 0;

  if (s_time_sync_event_group != NULL) {
        bits = xEventGroupWaitBits(s_time_sync_event_group,
                                   TIME_SYNC_BIT,
                                   pdTRUE, /* Clear the bit on exit */
                                   pdFALSE, /* Wait for just this bit */
                                   sntp_wait_ticks);
  } else {
      log_error(TAG, "Sync Task Error", "Event group is NULL, cannot wait for sync bit."); /* Use log_error */
      /* Fallback to default time if we can't wait */
      priv_set_default_time();
      s_time_sync_task_handle = NULL; /* Clear handle before deleting self */
      vTaskDelete(NULL);
      return;
  }


  if ((bits & TIME_SYNC_BIT) == 0) {
    log_error(TAG, /* Use log_error */
              "Sync Timeout", /* Changed log message */
              "Time sync via NTP failed after timeout (%lu ms)", CONFIG_PSTAR_KCONFIG_TIME_SYNC_TIMEOUT_MS);
    /* Set default time as fallback only if not already initialized by a late callback */
    priv_set_default_time();
  } else {
    /* Sync successful (callback already set s_time_initialized) */
    log_info(TAG, /* Use log_info */
             "Sync Success", /* Changed log message */
             "Time synchronization successful.");
  }

  log_info(TAG, "Sync Task Finished", "Time synchronization task finished."); /* Use log_info */
  s_time_sync_task_handle = NULL; /* Clear handle before deleting self */
  vTaskDelete(NULL);
}

/* Public Functions ***********************************************************/

bool time_manager_is_initialized(void)
{
  /* Fix: Use atomic load */
  return atomic_load(&s_time_initialized);
}

/* Fix: Changed signature and implementation */
esp_err_t time_manager_get_timestamp(char* buffer, size_t size)
{
  if (!atomic_load(&s_time_initialized)) {
    /* log_warn(TAG, "Timestamp Warning", "Time not initialized yet"); /* Avoid logging within get_timestamp? */ */
    return ESP_ERR_INVALID_STATE;
  }
  if (buffer == NULL || size < 20) { /* Need at least 20 for "YYYY-MM-DD HH:MM:SS\0" */
     return ESP_ERR_INVALID_ARG;
  }


  time_t    now = time(NULL);
  struct tm timeinfo;

  localtime_r(&now, &timeinfo); /* Use thread-safe version */

  /* Format directly into the provided buffer */
  size_t written = strftime(buffer, size, "%Y-%m-%d %H:%M:%S", &timeinfo);

  if (written == 0) {
      /* Formatting failed or buffer too small */
      buffer[0] = '\0'; /* Ensure null termination on error */
      /* log_error(TAG, "Timestamp Format Error", "strftime failed or buffer too small (size: %zu)", size); /* Avoid logging here */ */
      return ESP_FAIL;
  }

  return ESP_OK;
}


esp_err_t time_manager_init(void)
{
  /* Fix: Check if already initialized (by atomic flag) */
   if (atomic_load(&s_time_initialized) || s_time_sync_task_handle != NULL) {
       log_warn(TAG, "Init Warning", "Time manager already initialized or init task running."); /* Use log_warn */
       return ESP_OK;
   }


  log_info(TAG, /* Use log_info */
           "Init Start",
           "Beginning time manager initialization");

  /* Create event group for synchronization */
  s_time_sync_event_group = xEventGroupCreate();
  if (s_time_sync_event_group == NULL) {
    log_error(TAG, /* Use log_error */
              "Init Error",
              "Failed to create event group");
    return ESP_ERR_NO_MEM; /* Fix: Return correct error code */
  }

  /* Start time synchronization task */
  BaseType_t task_created = xTaskCreate(priv_time_sync_task,
                                        "time_sync_task",
                                        4096, /* Kconfig? Hardcoded for now */
                                        NULL,
                                        5,    /* Kconfig? Hardcoded for now */
                                        &s_time_sync_task_handle);

  if (task_created != pdPASS) {
    log_error(TAG, /* Use log_error */
              "Init Error",
              "Failed to create time sync task");
    vEventGroupDelete(s_time_sync_event_group);
    s_time_sync_event_group = NULL;
    s_time_sync_task_handle = NULL; /* Ensure handle is NULL on failure */
    return ESP_FAIL; /* Or ESP_ERR_NO_MEM if task creation failed due to memory */
  }

  log_info(TAG, /* Use log_info */
           "Init In Progress", /* Changed log message */
           "Time synchronization started in background task");

  return ESP_OK;
}

esp_err_t time_manager_cleanup(void)
{
  log_info(TAG, /* Use log_info */
           "Cleanup Start",
           "Beginning time manager cleanup");

  /* Delete the time sync task if it's still running */
  if (s_time_sync_task_handle != NULL) {
    log_info(TAG, "Cleanup", "Deleting time sync task..."); /* Use log_info */
    vTaskDelete(s_time_sync_task_handle);
    s_time_sync_task_handle = NULL;
  }

  /* Delete the event group */
  if (s_time_sync_event_group != NULL) {
    vEventGroupDelete(s_time_sync_event_group);
    s_time_sync_event_group = NULL;
  }

  /* Stop the SNTP service if it was initialized */
  if (s_sntp_initialized) {
    esp_sntp_stop();
    s_sntp_initialized = false; /* Mark as stopped */
    log_info(TAG, "SNTP Stop", "SNTP service stopped successfully"); /* Use log_info */
  }

  /* Reset the initialization flag */
  /* Keep initialized true if default time was set? Or reset always? Resetting seems safer. */
  atomic_store(&s_time_initialized, false);

  log_info(TAG, /* Use log_info */
           "Cleanup Complete",
           "Time manager cleanup completed successfully");

  return ESP_OK; /* Cleanup usually shouldn't fail critically */
}
