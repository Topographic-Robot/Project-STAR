/* components/common/log_handler.c */

#include "log_handler.h"
#include "esp_system.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include "time_manager.h"

/* Constants ******************************************************************/

const char *log_tag = "LOG_HANDLER";

/* Global Variables *********************************************************/

_Atomic uint64_t g_log_sequence_number = 0; /* Initialize sequence counter */

/* Private Functions *******************************************************/

/**
 * @brief Get the current task information formatted as a string
 * 
 * @param buffer Buffer to store the task info string
 * @param size Size of the buffer
 */
static void priv_get_task_info(char *buffer, size_t size) 
{
  if (!buffer || size == 0) {
    return;
  }

  TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
  if (current_task) {
    char task_name[LOG_TASK_NAME_LENGTH];
    /* Get task name, truncate if too long */
    strlcpy(task_name, pcTaskGetName(current_task), sizeof(task_name));
    snprintf(buffer, size, "[%s:%p]", task_name, (void*)current_task);
  } else {
    /* If no task context (e.g. running from ISR), indicate that */
    snprintf(buffer, size, "[ISR]");
  }
}

/* Public Functions ********************************************************/

void log_write_va(esp_log_level_t level, const char *tag,
                  const char *short_msg, const char *detailed_msg,
                  va_list args) 
{
  if (!tag || !short_msg || !detailed_msg) {
    ESP_LOGE(tag, "Invalid arguments: tag, short_msg, or detailed_msg is NULL");
    return;
  }

  /* Format the detailed message with provided va_list */
  char formatted_msg[LOG_MAX_MESSAGE_LENGTH];
  vsnprintf(formatted_msg, sizeof(formatted_msg), detailed_msg, args);

  /* Get task information */
  char task_info[LOG_MAX_MESSAGE_LENGTH / 4];
  priv_get_task_info(task_info, sizeof(task_info));

  /* Create the complete log message with the required format */
  char complete_msg[LOG_MAX_MESSAGE_LENGTH * 2];
  char *timestamp = time_manager_get_timestamp();
  uint64_t seq_num = atomic_fetch_add(&g_log_sequence_number, 1);

  if (timestamp != NULL) {
    /* Include timestamp, sequence number, and task info */
    snprintf(complete_msg, sizeof(complete_msg), "[%s][%llu]%s%s%s%s%s%s",
             timestamp,
             seq_num,
             task_info,
             short_msg,
             LOG_SEPARATOR,
             formatted_msg,
             LOG_SEPARATOR,
             tag);
    free(timestamp);
  } else {
    /* Skip timestamp but include sequence number and task info */
    snprintf(complete_msg, sizeof(complete_msg), "[%llu]%s%s%s%s%s%s",
             seq_num,
             task_info,
             short_msg,
             LOG_SEPARATOR,
             formatted_msg,
             LOG_SEPARATOR,
             tag);
  }

  /* Log using ESP's logging system */
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
}

void log_write(esp_log_level_t level, const char *tag,
               const char *short_msg, const char *detailed_msg, ...)
{
  va_list args;
  va_start(args, detailed_msg);
  log_write_va(level, tag, short_msg, detailed_msg, args);
  va_end(args);
}