/* components/pstar_logger/log_handler.c */

#include "pstar_log_handler.h"
#include "sdkconfig.h"
#include "esp_system.h" /* For esp_log_write */
#include "esp_log.h"
#if CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
#include "pstar_time_manager.h"
#endif
#include "pstar_log_macros.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <stdatomic.h>

// Include storage header regardless of SD enable status, as log_init/cleanup call storage functions
#include "pstar_log_storage.h"

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
// These are only needed if SD is enabled
#include "pstar_sd_card_hal.h"
#include "pstar_file_write_manager.h"
#endif

/* Constants ******************************************************************/

static const char *TAG = "Log Handler";

/* Global Variables ***********************************************************/

_Atomic uint64_t g_log_sequence_number = 0; /* Initialize sequence counter */

/* Globals (Static) ***********************************************************/

// Tracks if the full logger (including storage) is initialized and ready.
// Console logging works even if this is false.
static _Atomic bool s_logger_fully_initialized = false;

/* Private Function Prototypes ************************************************/

static void priv_get_task_info(char* buffer, size_t size);

/* Public Functions ***********************************************************/

bool log_is_fully_initialized(void) {
    return atomic_load(&s_logger_fully_initialized);
}

void log_write_va(esp_log_level_t   level,
                  const char* const tag,
                  const char* const short_msg,
                  const char* const detailed_msg,
                  va_list           args)
{
  // Basic validation
  if (!tag || !short_msg || !detailed_msg) {
    // Use ESP_LOGE directly for critical argument errors
    ESP_LOGE("Log Handler Core", "Invalid arguments passed to log_write_va (tag=%p, short=%p, detailed=%p)", tag, short_msg, detailed_msg);
    return;
  }

  /* Format the detailed message with provided va_list */
  char formatted_msg[CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH];
  int  result = vsnprintf(formatted_msg,
                          sizeof(formatted_msg),
                          detailed_msg,
                          args);

  /* Check for formatting errors or truncation */
  if (result < 0) {
    // Use ESP_LOGE directly for internal formatting errors
    ESP_LOGE(tag, "Error formatting detailed message: vsnprintf returned %d", result);
    strlcpy(formatted_msg, "ERROR FORMATTING DETAILED MESSAGE", sizeof(formatted_msg));
  } else if ((size_t)result >= sizeof(formatted_msg)) {
    // Use ESP_LOGW for truncation, it might not be critical
    ESP_LOGW(tag, "Detailed message truncated from %d to %zu bytes", result, sizeof(formatted_msg) - 1);
    formatted_msg[sizeof(formatted_msg) - 1] = '\0'; // Ensure null-termination
  }

  /* Get task information */
  char task_info[CONFIG_PSTAR_KCONFIG_LOGGING_TASK_NAME_LENGTH + 10] = {0}; // Slightly larger buffer
#if CONFIG_PSTAR_KCONFIG_LOGGING_INCLUDE_TASK_INFO
    priv_get_task_info(task_info, sizeof(task_info));
#endif

  /* Get timestamp if enabled and time manager is ready */
  char timestamp_buf[CONFIG_PSTAR_KCONFIG_LOGGING_TIMESTAMP_BUFFER_SIZE] = {0};
  bool timestamp_ok = false;
#if CONFIG_PSTAR_KCONFIG_LOGGING_INCLUDE_TIMESTAMP && CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
  /* Check time manager initialization atomically or via function call */
  if (time_manager_is_initialized()) {
    esp_err_t time_err = time_manager_get_timestamp(timestamp_buf, sizeof(timestamp_buf));
    if (time_err == ESP_OK) {
      timestamp_ok = true;
    }
    /* else: Continue without timestamp, get_timestamp logs its own errors */
  }
#endif

  /* Get sequence number */
  uint64_t seq_num = atomic_fetch_add(&g_log_sequence_number, 1);

  /* Create the complete log message with the required format */
  char complete_msg[PSTAR_LOGGING_MAX_FORMATTED_ENTRY_LENGTH];
  int written = 0;

  // Construct the log prefix based on available information
  char prefix_buf[sizeof(timestamp_buf) + sizeof(task_info) + 25] = {0}; // Buffer for "[timestamp][seq][task]"
  char* p = prefix_buf;
  size_t remaining = sizeof(prefix_buf);

  if (timestamp_ok) {
    int w = snprintf(p, remaining, "[%s]", timestamp_buf);
    if (w > 0 && (size_t)w < remaining) { p += w; remaining -= w; } else { timestamp_ok = false; /* Formatting failed */ }
  }
  int w_seq = snprintf(p, remaining, "[%llu]", seq_num);
  if (w_seq > 0 && (size_t)w_seq < remaining) { p += w_seq; remaining -= w_seq; }
#if CONFIG_PSTAR_KCONFIG_LOGGING_INCLUDE_TASK_INFO
  int w_task = snprintf(p, remaining, "%s", task_info); // task_info already includes brackets
  if (w_task > 0 && (size_t)w_task < remaining) { p += w_task; }
#endif
  // Add a space after the prefix if it's not empty
  if (p != prefix_buf && remaining > 1) {
      *p++ = ' ';
      *p = '\0'; // Null-terminate prefix
      remaining--;
  } else {
      prefix_buf[0] = '\0'; // Empty prefix
  }


  /* Combine prefix, short message, separator, and formatted detailed message */
   written = snprintf(complete_msg,
                       sizeof(complete_msg),
                       "%s%s%s%s",
                       prefix_buf,
                       short_msg,
                       CONFIG_PSTAR_KCONFIG_LOGGING_SEPARATOR,
                       formatted_msg);


  /* Check for truncation in the complete message */
  if (written < 0) {
    // Use ESP_LOGE directly for internal formatting errors
    ESP_LOGE(tag, "Error formatting complete message: snprintf returned %d", written);
    strlcpy(complete_msg, "ERROR FORMATTING COMPLETE MESSAGE", sizeof(complete_msg));
  } else if ((size_t)written >= sizeof(complete_msg)) {
    // Use ESP_LOGW for truncation
    ESP_LOGW(tag, "Complete message truncated from %d to %zu bytes", written, sizeof(complete_msg) - 1);
    complete_msg[sizeof(complete_msg) - 1] = '\0'; // Ensure null-termination
   }

  /* --- Always Log to Console (using underlying ESP log) --- */
#if CONFIG_PSTAR_KCONFIG_LOGGING_CONSOLE_ENABLED
  /* Log only if the level is enabled in menuconfig for this tag */
  if (esp_log_level_get(tag) >= level) {
    // Use ESP's built-in log function which handles UART output
    esp_log_write(level, tag, "%s\n", complete_msg);
  }
#endif

  /* --- Log to Storage (ONLY if SD Card support is ENABLED and logger is fully initialized) --- */
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  // Check if the *full* logger (including storage) is ready
  if (atomic_load(&s_logger_fully_initialized)) {
    // Pass the already fully formatted message to storage component.
    // log_storage_write handles buffering internally based on SD availability.
    esp_err_t storage_err = log_storage_write(level, complete_msg);
    if (storage_err != ESP_OK && storage_err != ESP_ERR_TIMEOUT) { // Don't spam logs on timeout
        // If storage write fails, log an error to console (which should still work)
        ESP_LOGE(tag, "Failed to write log to storage: %s", esp_err_to_name(storage_err));
    }
  }
  // *** FIX: Removed the #else block that called log_storage_write when SD was disabled ***
#endif
}

void log_write(esp_log_level_t   level,
               const char* const tag,
               const char* const short_msg,
               const char* const detailed_msg,
               ...)
{
  va_list args;
  va_start(args, detailed_msg);
  log_write_va(level, tag, short_msg, detailed_msg, args);
  va_end(args);
}

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
esp_err_t log_init(file_write_manager_t* file_manager,
                   sd_card_hal_t*        sd_card)
#else
esp_err_t log_init(void* file_manager, // Parameters ignored
                   void* sd_card)
#endif
{
  // Use ESP_LOGI for initial messages as full logger might not be ready
  ESP_LOGI(TAG, "log_init called...");

  // If already fully initialized, just return success
  if (atomic_load(&s_logger_fully_initialized)) {
    ESP_LOGI(TAG, "Log handler already fully initialized.");
    return ESP_OK;
  }

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  // SD Logging is enabled in Kconfig

  // Check if arguments for full init are provided
  if (file_manager == NULL && sd_card == NULL) {
      // Minimal init requested (or called before SD HAL is ready)
      // Console logging already works. No storage init possible yet.
      ESP_LOGI(TAG, "Performing minimal log handler initialization (SD storage deferred).");
      // Call storage init with NULLs for minimal setup (mutex)
      log_storage_init(NULL, NULL);
      // Do NOT set s_logger_fully_initialized = true yet.
      return ESP_OK;
  } else if (file_manager == NULL || sd_card == NULL) {
      // Invalid combination - need both for full SD init
      ESP_LOGE(TAG, "Full log init failed: Both file_manager and sd_card must be provided when SD logging is enabled.");
      return ESP_ERR_INVALID_ARG;
  } else {
      // Both pointers provided, attempt full initialization including storage
      ESP_LOGI(TAG, "Attempting full log handler initialization (including SD storage)...");

      // Initialize log storage
      esp_err_t ret = log_storage_init(file_manager, sd_card);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize log storage: %s", esp_err_to_name(ret));
        // Do not mark as fully initialized
        return ret; // Propagate storage init error
      }

      ESP_LOGI(TAG, "Log storage initialized successfully.");
      // Mark logger as fully initialized ONLY after storage init succeeds
      atomic_store(&s_logger_fully_initialized, true);
      log_info(TAG, "Init Complete", "Full log handler initialized successfully (Console + SD Card)");
      return ESP_OK;
  }

#else
  // SD Logging is disabled in Kconfig
  ESP_LOGI(TAG, "Performing log handler initialization (SD Card Disabled).");
  // Minimal init IS the full init in this case.
  // Initialize storage with NULL pointers (it will handle this minimally)
  log_storage_init(NULL, NULL);
  atomic_store(&s_logger_fully_initialized, true);
  log_info(TAG, "Init Complete", "Log handler initialized successfully (Console Only)");
  return ESP_OK;
#endif
}

esp_err_t log_flush(void)
{
  // Check if full logger (including storage) is initialized
  if (!atomic_load(&s_logger_fully_initialized)) {
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
    // Only warn if SD logging is expected but not ready
    log_warn(TAG, "Flush Skip", "Log flush called before full logger initialization.");
    return ESP_ERR_INVALID_STATE;
#else
    // If SD is disabled, flushing is a no-op, return OK.
    log_info(TAG, "Flush Skip", "SD card logging is disabled, skipping flush.");
    return ESP_OK;
#endif
  }

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  // If SD logging is enabled and fully initialized, call storage flush
  log_info(TAG, "Flush Start", "Attempting to flush log storage...");
  esp_err_t ret = log_storage_flush(); // log_storage_flush handles its own init/mutex checks
  if (ret != ESP_OK) {
    log_error(TAG, "Flush Error", "Failed to flush log storage: %s", esp_err_to_name(ret));
  } else {
    log_info(TAG, "Flush Success", "Log storage flushed successfully.");
  }
  return ret;
#else
  // Should not be reached if SD disabled due to check above, but included for completeness.
  log_info(TAG, "Flush Skip", "SD card logging is disabled, skipping flush.");
  return ESP_OK;
#endif
}

esp_err_t log_cleanup(void)
{
  // Use atomic exchange to check and set the flag in one operation
  bool was_initialized = atomic_exchange(&s_logger_fully_initialized, false);

  // If logger wasn't fully initialized, there's less to clean up
  if (!was_initialized) {
    // Use ESP_LOGW as our own logger might be partially down or never fully up
    ESP_LOGW(TAG, "Log cleanup called but logger was not fully initialized.");
    // Still cleanup storage in case minimal storage init (mutex) happened
    log_storage_cleanup();
    atomic_store(&g_log_sequence_number, 0); // Reset sequence number
    return ESP_OK; // Not an error to cleanup if not fully initialized
  }

  // Use ESP_LOGx for cleanup messages now, as our logger is marked uninitialized
  ESP_LOGI(TAG, "Cleanup Start - Beginning log handler cleanup");

  esp_err_t final_ret = ESP_OK;

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  // Attempt a final flush if storage was initialized
  if (log_storage_is_initialized()) { // Check storage init status
      ESP_LOGI(TAG, "Cleanup - Attempting final log flush...");
      esp_err_t flush_ret = log_storage_flush();
      // Ignore INVALID_STATE as storage cleanup might have already run partially
      if (flush_ret != ESP_OK && flush_ret != ESP_ERR_INVALID_STATE && flush_ret != ESP_ERR_TIMEOUT) {
          ESP_LOGW(TAG, "Cleanup Warning - Final log flush failed: %s", esp_err_to_name(flush_ret));
          final_ret = flush_ret; // Record the first error
      }
  }
#endif

  // Clean up log storage (handles its own internal state)
  ESP_LOGI(TAG, "Cleanup - Cleaning up log storage component...");
  esp_err_t storage_cleanup_ret = log_storage_cleanup();
  if (storage_cleanup_ret != ESP_OK && storage_cleanup_ret != ESP_ERR_TIMEOUT) {
     ESP_LOGW(TAG, "Cleanup Warning - Log storage cleanup failed: %s", esp_err_to_name(storage_cleanup_ret));
     if (final_ret == ESP_OK) {
         final_ret = storage_cleanup_ret; // Record error if none previously
     }
  }

  // Reset sequence number
  atomic_store(&g_log_sequence_number, 0);

  ESP_LOGI(TAG,
           "Cleanup Complete - Log handler cleanup %s",
           (final_ret == ESP_OK) ? "successful" : "completed with warnings/errors");

  return final_ret;
}

/* Private Functions **********************************************************/

/**
 * @brief Get the current task information formatted as a string
 *
 * @param[out] buffer Buffer to store the task info string
 * @param[in]  size   Size of the buffer
 */
static void priv_get_task_info(char* buffer, size_t size)
{
  if (!buffer || size == 0) {
    return;
  }
  buffer[0] = '\0'; // Ensure null-terminated initially

  /* Check if running inside scheduler */
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    if (current_task) {
      char task_name[CONFIG_PSTAR_KCONFIG_LOGGING_TASK_NAME_LENGTH];
      const char* name_ptr = pcTaskGetName(current_task);
      if (name_ptr != NULL) {
        strlcpy(task_name, name_ptr, sizeof(task_name));
      } else {
        strlcpy(task_name, "???", sizeof(task_name)); // Indicate unknown name
      }

      int written = snprintf(buffer, size, "[%s:%p]", task_name, (void*)current_task);
      if (written < 0 || (size_t)written >= size) {
        if (size > 0) buffer[size - 1] = '\0'; // Ensure null termination if truncated
      }
    } else {
      // Should not happen if scheduler running, but handle defensively
      int written = snprintf(buffer, size, "[NoTask]");
      if (written < 0 || (size_t)written >= size) {
         if (size > 0) buffer[size - 1] = '\0';
      }
    }
  } else {
    // If no task context (e.g., running before scheduler starts)
    int written = snprintf(buffer, size, "[PreOS]");
     if (written < 0 || (size_t)written >= size) {
         if (size > 0) buffer[size - 1] = '\0';
    }
  }
}