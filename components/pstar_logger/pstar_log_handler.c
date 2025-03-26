/* components/pstar_logger/log_handler.c */

#include "pstar_log_handler.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_log.h"
#include "pstar_time_manager.h"
#include "pstar_log_macros.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
#include "pstar_log_storage.h"
#include "pstar_sd_card_hal.h" 
#include "pstar_file_write_manager.h"
#endif

/* Constants ******************************************************************/

#define LOG_HANDLER_TAG ("Log Handler")

/* Global Variables ***********************************************************/

_Atomic uint64_t g_log_sequence_number = 0; /* Initialize sequence counter */

/* Globals (Static) ***********************************************************/

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
static file_write_manager_t* s_file_manager = NULL;
static sd_card_hal_t*        s_sd_card      = NULL;
#endif
static bool                  s_logger_initialized = false; /* Track initialization state */

/* Private Function Prototypes ************************************************/

static void priv_get_task_info(char* buffer, size_t size);

/* Public Functions ***********************************************************/

void log_write_va(esp_log_level_t   level, 
                  const char* const tag,
                  const char* const short_msg, 
                  const char* const detailed_msg,
                  va_list           args) 
{
  if (!tag || !short_msg || !detailed_msg) {
    ESP_LOGE(tag ? tag : "LOG", "Invalid arguments: tag, short_msg, or detailed_msg is NULL");
    return;
  }

  /* Format the detailed message with provided va_list */
  char formatted_msg[CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH];
  int  result = vsnprintf(formatted_msg, sizeof(formatted_msg), detailed_msg, args);
  
  /* Check for formatting errors or truncation */
  if (result < 0) {
    ESP_LOGE(tag, "Error formatting message: vsnprintf returned %d", result);
    strlcpy(formatted_msg, "ERROR FORMATTING MESSAGE", sizeof(formatted_msg));
  } else if ((size_t)result >= sizeof(formatted_msg)) {
    /* Message was truncated, add a warning note */
    ESP_LOGW(tag, "Message truncated from %d to %zu bytes", 
             result, sizeof(formatted_msg) - 1);
    
    /* Ensure null-termination */
    formatted_msg[sizeof(formatted_msg) - 1] = '\0';
  }

  /* Get task information */
  char task_info[CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH / 4];
  priv_get_task_info(task_info, sizeof(task_info));

  /* Create the complete log message with the required format */
  char     complete_msg[CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH];
  char*    timestamp = NULL;
  uint64_t seq_num   = atomic_fetch_add(&g_log_sequence_number, 1);

  /* Only try to get timestamp if time manager is initialized and timestamp inclusion is enabled */
#if CONFIG_PSTAR_KCONFIG_LOGGING_INCLUDE_TIMESTAMP
  if (time_manager_is_initialized()) {
    timestamp = time_manager_get_timestamp();
  }
#endif

  int written;
  if (timestamp != NULL) {
    /* Include timestamp, sequence number, and task info */
    written = snprintf(complete_msg, 
                       sizeof(complete_msg), 
                       "[%s][%llu]%s %s%s%s",
                       timestamp,
                       seq_num,
                       CONFIG_PSTAR_KCONFIG_LOGGING_INCLUDE_TASK_INFO ? task_info : "",
                       short_msg,
                       CONFIG_PSTAR_KCONFIG_LOGGING_SEPARATOR,
                       formatted_msg);
    free(timestamp);
  } else {
    /* Skip timestamp but include sequence number and task info */
    written = snprintf(complete_msg, 
                       sizeof(complete_msg), 
                       "[%llu]%s %s%s%s",
                       seq_num,
                       CONFIG_PSTAR_KCONFIG_LOGGING_INCLUDE_TASK_INFO ? task_info : "",
                       short_msg,
                       CONFIG_PSTAR_KCONFIG_LOGGING_SEPARATOR,
                       formatted_msg);
  }
  
  /* Check for truncation in the complete message */
  if (written < 0) {
    ESP_LOGE(tag, "Error formatting complete message: snprintf returned %d", written);
    strlcpy(complete_msg, "ERROR FORMATTING COMPLETE MESSAGE", sizeof(complete_msg));
  } else if ((size_t)written >= sizeof(complete_msg)) {
    /* Complete message was truncated */
    ESP_LOGW(tag, "Complete message truncated from %d to %zu bytes", 
             written, sizeof(complete_msg) - 1);
    
    /* Ensure null-termination */
    complete_msg[sizeof(complete_msg) - 1] = '\0';
  }

  /* Log using ESP's logging system - always do this, even if logger isn't initialized yet */
  switch (level) {
    case ESP_LOG_ERROR:
      ESP_LOGE(tag, "%s", complete_msg);
      break;
    case ESP_LOG_WARN:
      ESP_LOGW(tag, "%s", complete_msg);
      break;
    case ESP_LOG_INFO:
      ESP_LOGI(tag, "%s", complete_msg);
      break;
    case ESP_LOG_DEBUG:
      ESP_LOGD(tag, "%s", complete_msg);
      break;
    case ESP_LOG_VERBOSE:
      ESP_LOGV(tag, "%s", complete_msg);
      break;
    default:
      ESP_LOGI(tag, "%s", complete_msg);
      break;
  }
  
  /* If SD card logging is enabled AND logger is initialized, also write to storage */
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  if (s_logger_initialized) {
    log_storage_write(level, complete_msg);
  }
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
esp_err_t log_init(void* file_manager, 
                   void* sd_card)
#endif
{
  /* If already initialized, just return success */
  if (s_logger_initialized) {
    ESP_LOGI(LOG_HANDLER_TAG, "Log handler already initialized");
    return ESP_OK;
  }
  
  /* Use ESP logger until we're fully initialized */
  ESP_LOGI(LOG_HANDLER_TAG, "Initializing log handler");
  
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  /* Check that both file_manager and sd_card are provided */
  if (file_manager == NULL || sd_card == NULL) {
    ESP_LOGE(LOG_HANDLER_TAG, "file_manager and sd_card must be provided when log_to_sd is true");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Store the references */
  s_file_manager = file_manager;
  s_sd_card = sd_card;
  
  /* Initialize log storage */
  esp_err_t ret = log_storage_init(file_manager, sd_card);
  if (ret != ESP_OK) {
    ESP_LOGE(LOG_HANDLER_TAG, "Failed to initialize log storage: %s", esp_err_to_name(ret));
    s_file_manager = NULL;
    s_sd_card = NULL;
    return ret;
  }
  
  ESP_LOGI(LOG_HANDLER_TAG, "Log storage initialized successfully");
#endif
  
  /* Mark logger as initialized */
  s_logger_initialized = true;
  
  /* Now that we're initialized, we can use our own logger */
  log_info(LOG_HANDLER_TAG, "Init Complete", "Log handler initialized successfully");
  return ESP_OK;
}

void log_set_sd_logging(bool enabled)
{
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  /* Only needed for runtime control when Kconfig already enables SD logging */
  if (!CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED) {
    log_warn(LOG_HANDLER_TAG, 
             "SD Config Error", 
             "Cannot enable SD card logging: disabled in Kconfig");
    return;
  }

  /* Only allow enabling if the file manager and SD card were provided during init */
  if (enabled && (s_file_manager == NULL || s_sd_card == NULL)) {
    log_error(LOG_HANDLER_TAG, 
              "SD Config Error", 
              "Cannot enable SD card logging: missing file_manager or sd_card");
    return;
  }
  
  log_info(LOG_HANDLER_TAG, 
           "SD Config", 
           "SD card logging %s", 
           enabled ? "enabled" : "disabled");
#else
  log_warn(LOG_HANDLER_TAG, 
           "SD Config Error", 
           "Cannot enable SD card logging: disabled in Kconfig");
#endif
}

esp_err_t log_flush(void)
{
  /* If logger isn't initialized, there's nothing to flush */
  if (!s_logger_initialized) {
    ESP_LOGW(LOG_HANDLER_TAG, "Log flush called before initialization");
    return ESP_ERR_INVALID_STATE;
  }
  
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  esp_err_t ret = log_storage_flush();
  if (ret != ESP_OK) {
    log_error(LOG_HANDLER_TAG, 
              "Flush Error", 
              "Failed to flush log storage: %s", 
              esp_err_to_name(ret));
  }
  
  return ret;
#else
  log_info(LOG_HANDLER_TAG, "Flush Skip", "SD card logging is disabled, skipping flush");
  return ESP_OK;
#endif
}

esp_err_t log_cleanup(void)
{
  /* If logger isn't initialized, there's nothing to clean up */
  if (!s_logger_initialized) {
    ESP_LOGW(LOG_HANDLER_TAG, "Log cleanup called before initialization");
    return ESP_ERR_INVALID_STATE;
  }
  
  log_info(LOG_HANDLER_TAG, "Cleanup Start", "Beginning log handler cleanup");

  esp_err_t ret = ESP_OK;

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  /* Flush any remaining logs */
  esp_err_t temp_ret = log_flush();
  if (temp_ret != ESP_OK) {
    log_warn(LOG_HANDLER_TAG, 
             "Flush Warning", 
             "Failed to flush logs during cleanup: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }

  /* Clean up log storage */
  temp_ret = log_storage_cleanup();
  if (temp_ret != ESP_OK) {
    log_warn(LOG_HANDLER_TAG, 
             "Storage Warning", 
             "Failed to clean up log storage: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }
#endif

  /* Reset sequence number */
  atomic_store(&g_log_sequence_number, 0);
  
  /* Clear references */
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  s_file_manager = NULL;
  s_sd_card = NULL;
#endif
  
  /* Mark logger as uninitialized */
  s_logger_initialized = false;

  /* We need to use ESP logger here since we've marked our logger as uninitialized */
  ESP_LOGI(LOG_HANDLER_TAG, 
           "Log handler cleanup %s", 
           (ret == ESP_OK) ? "successful" : "completed with warnings");

  return ret;
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

  TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
  if (current_task) {
    char task_name[CONFIG_PSTAR_KCONFIG_LOGGING_TASK_NAME_LENGTH];
    /* Get task name, truncate if too long */
    strlcpy(task_name, pcTaskGetName(current_task), sizeof(task_name));
    int written = snprintf(buffer, size, "[%s:%p]", task_name, (void*)current_task);
    
    /* Check for truncation */
    if (written < 0 || (size_t)written >= size) {
      /* Ensure null termination if truncated */
      buffer[size - 1] = '\0';
    }
  } else {
    /* If no task context (e.g. running from ISR), indicate that */
    int written = snprintf(buffer, size, "[ISR]");
    
    /* Check for truncation */
    if (written < 0 || (size_t)written >= size) {
      /* Ensure null termination if truncated */
      buffer[size - 1] = '\0';
    }
  }
}