/* components/pstar_managers/pstar_time_manager.c */

#include "pstar_time_manager.h"

#include "pstar_log_handler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "sdkconfig.h"

/* Constants ******************************************************************/

static const char* TAG = "Time Manager";

#define TIME_SYNC_BIT (1 << 0) /**< Bit for time sync success event group */
#define IP_EVENT_BIT (1 << 1)  /**< Bit for network interface ready event */

/* Globals (Static) ***********************************************************/

static _Atomic bool       s_time_initialized = false; /**< Flag indicating if time is initialized */
static EventGroupHandle_t s_time_sync_event_group = NULL;
static TaskHandle_t       s_time_sync_task_handle = NULL;
static bool               s_sntp_initialized      = false; /**< Track SNTP service state */
static bool               s_network_ready         = false; /**< Track network state */

/* Private Functions **********************************************************/

/**
 * @brief Callback function called when time is synchronized with NTP server
 *
 * @param tv Pointer to timeval structure containing the synchronized time
 */
static void priv_time_sync_notification_cb(struct timeval* tv)
{
  /* Mark time as initialized once synced */
  atomic_store(&s_time_initialized, true);

  if (s_time_sync_event_group) {
    xEventGroupSetBits(s_time_sync_event_group, TIME_SYNC_BIT);
  } else {
    /* Event group might be deleted during cleanup race condition */
    log_warn(TAG, "Sync CB Warning", "Event group NULL in sync callback");
  }

  char      strftime_buf[CONFIG_PSTAR_KCONFIG_TIME_STRFTIME_BUFFER_SIZE];
  time_t    now = tv->tv_sec;
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);

  log_info(TAG, "Time Synced", "Time synchronized via NTP: %s", strftime_buf);
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

  log_warn(TAG,
           "Default Time Set",
           "Setting default time (2025-01-01) due to unavailable network sync or NTP failure");

  struct timeval tv;
  struct tm      tm;

  /* Set all time values to 0 */
  memset(&tm, 0, sizeof(tm));

  /* Set to January 1st, 2025 */
  tm.tm_year  = 2025 - 1900; /* Years since 1900 */
  tm.tm_mon   = 0;           /* January (0-based month) */
  tm.tm_mday  = 1;           /* Day */
  tm.tm_isdst = -1;          /* Let system determine DST if applicable (usually not for embedded) */

  /* Convert tm to time_t and set timeval */
  time_t time_value = mktime(&tm);
  tv.tv_sec         = time_value;
  tv.tv_usec        = 0;

  /* Use settimeofday to set system time */
  if (settimeofday(&tv, NULL) != 0) {
    log_error(TAG, "Default Time Error", "Failed to set default system time!");
    /* Cannot mark as initialized if setting failed */
  } else {
    log_info(TAG, "Default Time Set", "System time set to default: 2025-01-01 00:00:00");
    /* Mark time as initialized even though it's default */
    atomic_store(&s_time_initialized, true);
    /* Set the sync bit as well, to unblock the task */
    if (s_time_sync_event_group) {
      xEventGroupSetBits(s_time_sync_event_group, TIME_SYNC_BIT);
    }
  }
}

/**
 * @brief Initializes the SNTP service for time synchronization.
 *
 * Configures SNTP to synchronize the system clock with an NTP server.
 * Should only be called when the network is ready.
 *
 * @note
 * - Ensure network connectivity before calling this function.
 * - The system time will remain unsynchronized if SNTP initialization fails.
 */
static void priv_initialize_sntp(void)
{
  /* Check if already initialized */
  if (s_sntp_initialized) {
    log_info(TAG, "SNTP Info", "SNTP service already initialized.");
    /* Optionally restart if needed, but generally not necessary unless config changes */
    // esp_sntp_stop();
    // esp_sntp_init();
    return;
  }

  log_info(TAG,
           "SNTP Start",
           "Initializing SNTP service (Server: %s)",
           CONFIG_PSTAR_KCONFIG_TIME_NTP_SERVER);

  /* Set SNTP operating mode (polling mode) */
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);

  /* Set the NTP server from Kconfig */
  esp_sntp_setservername(0, CONFIG_PSTAR_KCONFIG_TIME_NTP_SERVER);

  /* Register the time sync notification callback */
  sntp_set_time_sync_notification_cb(priv_time_sync_notification_cb);

  /* Initialize SNTP */
  esp_sntp_init();
  s_sntp_initialized = true;

  log_info(TAG, "SNTP Initialized", "SNTP service initialized, awaiting first time sync");
}

/**
 * @brief Event handler for IP stack events (specifically GOT_IP)
 */
static void
ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
  if (event_base == IP_EVENT) {
    if (event_id == IP_EVENT_STA_GOT_IP || event_id == IP_EVENT_ETH_GOT_IP) {
      ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
      log_info(TAG, "Network Ready", "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
      s_network_ready = true;
      /* Signal the time sync task that network is ready */
      if (s_time_sync_event_group) {
        xEventGroupSetBits(s_time_sync_event_group, IP_EVENT_BIT);
      }
      /* Initialize SNTP now that we have an IP */
      if (!s_sntp_initialized) {
        priv_initialize_sntp();
      }
      /* Unregister handler after first IP event */
      esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, ip_event_handler);
      esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, ip_event_handler);
    }
  }
}

/**
 * @brief Task that waits for network and SNTP sync
 *
 * @param pvParameters Task parameters (unused)
 */
static void priv_time_sync_task(void* pvParameters)
{
  log_info(TAG, "Sync Task Started", "Time synchronization task running");

  EventBits_t uxBits;
  TickType_t  network_wait_ticks = pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_TIME_SYNC_TIMEOUT_MS);
  TickType_t  sntp_wait_ticks    = pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_TIME_SYNC_TIMEOUT_MS);

  /* Wait for the IP event bit to be set by the event handler */
  if (s_time_sync_event_group != NULL) {
    log_info(TAG,
             "Network Wait",
             "Waiting up to %u ms for network connection (IP event)...",
             CONFIG_PSTAR_KCONFIG_TIME_SYNC_TIMEOUT_MS);
    uxBits = xEventGroupWaitBits(s_time_sync_event_group,
                                 IP_EVENT_BIT,
                                 pdFALSE, // Don't clear the bit
                                 pdFALSE, // Wait for just this bit
                                 network_wait_ticks);
  } else {
    log_error(TAG, "Sync Task Error", "Event group is NULL, cannot wait for IP event.");
    priv_set_default_time(); // Fallback if group is bad
    s_time_sync_task_handle = NULL;
    vTaskDelete(NULL);
    return;
  }

  if ((uxBits & IP_EVENT_BIT) == 0) {
    /* IP event never occurred within timeout */
    log_error(TAG,
              "Network Error",
              "Network connection (IP event) timed out after %u ms. Setting default time.",
              CONFIG_PSTAR_KCONFIG_TIME_SYNC_TIMEOUT_MS);
    priv_set_default_time();
    s_time_sync_task_handle = NULL;
    vTaskDelete(NULL);
    return;
  }

  /* Network is ready (IP_EVENT_BIT was set), SNTP should have been initialized by handler.
     Now wait for the SNTP sync notification bit */
  if (s_time_sync_event_group != NULL) {
    log_info(TAG,
             "SNTP Wait",
             "Network ready. Waiting up to %u ms for SNTP synchronization...",
             CONFIG_PSTAR_KCONFIG_TIME_SYNC_TIMEOUT_MS);
    uxBits = xEventGroupWaitBits(s_time_sync_event_group,
                                 TIME_SYNC_BIT,
                                 pdTRUE,  /* Clear the sync bit on exit */
                                 pdFALSE, /* Wait for just this bit */
                                 sntp_wait_ticks);
  } else {
    log_error(TAG, "Sync Task Error", "Event group is NULL, cannot wait for SNTP sync bit.");
    priv_set_default_time(); // Fallback if group is bad
    s_time_sync_task_handle = NULL;
    vTaskDelete(NULL);
    return;
  }

  if ((uxBits & TIME_SYNC_BIT) == 0) {
    /* SNTP sync timed out */
    log_error(TAG,
              "Sync Timeout",
              "Time sync via NTP failed after timeout (%u ms)",
              CONFIG_PSTAR_KCONFIG_TIME_SYNC_TIMEOUT_MS);
    /* Set default time as fallback only if not already initialized by a late callback */
    priv_set_default_time();
  } else {
    /* Sync successful (callback already set s_time_initialized) */
    log_info(TAG, "Sync Success", "Time synchronization successful.");
  }

  log_info(TAG, "Sync Task Finished", "Time synchronization task finished.");
  s_time_sync_task_handle = NULL; /* Clear handle before deleting self */
  vTaskDelete(NULL);
}

/* Public Functions ***********************************************************/

bool time_manager_is_initialized(void)
{
  return atomic_load(&s_time_initialized);
}

esp_err_t time_manager_get_timestamp(char* buffer, size_t size)
{
  if (!atomic_load(&s_time_initialized)) {
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
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t time_manager_init(void)
{
  /* Check if already initialized (by atomic flag or running task) */
  if (atomic_load(&s_time_initialized) || s_time_sync_task_handle != NULL) {
    log_warn(TAG, "Init Warning", "Time manager already initialized or init task running.");
    return ESP_OK;
  }

  log_info(TAG, "Init Start", "Beginning time manager initialization");

  /* Create event group for synchronization */
  s_time_sync_event_group = xEventGroupCreate();
  if (s_time_sync_event_group == NULL) {
    log_error(TAG, "Init Error", "Failed to create event group");
    return ESP_ERR_NO_MEM;
  }

  /* Register IP event handlers */
  esp_err_t reg_err =
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL);
  if (reg_err != ESP_OK) {
    log_error(TAG,
              "Init Error",
              "Failed to register STA IP event handler: %s",
              esp_err_to_name(reg_err));
    vEventGroupDelete(s_time_sync_event_group);
    s_time_sync_event_group = NULL;
    return reg_err;
  }
  reg_err = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &ip_event_handler, NULL);
  if (reg_err != ESP_OK) {
    log_error(TAG,
              "Init Error",
              "Failed to register ETH IP event handler: %s",
              esp_err_to_name(reg_err));
    esp_event_handler_unregister(IP_EVENT,
                                 IP_EVENT_STA_GOT_IP,
                                 ip_event_handler); // Unregister STA handler
    vEventGroupDelete(s_time_sync_event_group);
    s_time_sync_event_group = NULL;
    return reg_err;
  }

  /* Start time synchronization task using Kconfig values for stack size and priority */
  BaseType_t task_created = xTaskCreate(priv_time_sync_task,
                                        "time_sync_task",
                                        CONFIG_PSTAR_KCONFIG_TIME_SYNC_TASK_STACK_SIZE,
                                        NULL,
                                        CONFIG_PSTAR_KCONFIG_TIME_SYNC_TASK_PRIORITY,
                                        &s_time_sync_task_handle);

  if (task_created != pdPASS) {
    log_error(TAG, "Init Error", "Failed to create time sync task");
    /* Unregister event handlers on task creation failure */
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, ip_event_handler);
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, ip_event_handler);
    vEventGroupDelete(s_time_sync_event_group);
    s_time_sync_event_group = NULL;
    s_time_sync_task_handle = NULL; /* Ensure handle is NULL on failure */
    return ESP_FAIL;
  }

  log_info(TAG,
           "Init In Progress",
           "Time synchronization task started, waiting for network event...");

  return ESP_OK;
}

esp_err_t time_manager_cleanup(void)
{
  log_info(TAG, "Cleanup Start", "Beginning time manager cleanup");

  /* Unregister event handlers */
  esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, ip_event_handler);
  esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP, ip_event_handler);

  /* Delete the time sync task if it's still running */
  if (s_time_sync_task_handle != NULL) {
    log_info(TAG, "Cleanup", "Deleting time sync task...");
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
    s_sntp_initialized = false;
    log_info(TAG, "SNTP Stop", "SNTP service stopped successfully");
  }

  /* Reset flags */
  atomic_store(&s_time_initialized, false);
  s_network_ready = false;

  log_info(TAG, "Cleanup Complete", "Time manager cleanup completed successfully");

  return ESP_OK;
}
