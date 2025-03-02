/* components/common/log_handler.c */

#include "log_handler.h"
#include "log_storage.h"
#include "esp_system.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* Constants ******************************************************************/

const char *log_tag = "LOG_HANDLER";

/* Globals (Static) ***********************************************************/

static bool s_log_to_sd_enabled = false;

/* Public Functions ***********************************************************/

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

  /* Create the complete log message with the required format */
  char complete_msg[LOG_MAX_MESSAGE_LENGTH * 2];
  snprintf(complete_msg, sizeof(complete_msg), "%s%s%s%s%s",
           short_msg,
           LOG_SEPARATOR,
           formatted_msg,
           LOG_SEPARATOR,
           tag);

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
  
  /* If SD card logging is enabled, also write to storage */
  if (s_log_to_sd_enabled) {
    log_storage_write(level, complete_msg);
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

esp_err_t log_init(bool log_to_sd)
{
  log_info(log_tag, "Init Start", "Initializing log handler");
  
  s_log_to_sd_enabled = log_to_sd;
  
  if (log_to_sd) {
    esp_err_t ret = log_storage_init();
    if (ret != ESP_OK) {
      log_error(log_tag, "Storage Error", "Failed to initialize log storage: %s", esp_err_to_name(ret));
      s_log_to_sd_enabled = false;
      return ret;
    }
    log_info(log_tag, "Storage Ready", "Log storage initialized successfully");
  }
  
  log_info(log_tag, "Init Complete", "Log handler initialized successfully");
  return ESP_OK;
}

void log_set_sd_logging(bool enabled)
{
  s_log_to_sd_enabled = enabled;
  log_info(log_tag, "SD Config", "SD card logging %s", enabled ? "enabled" : "disabled");
}

esp_err_t log_flush(void)
{
  if (!s_log_to_sd_enabled) {
    log_info(log_tag, "Flush Skip", "SD card logging is disabled, skipping flush");
    return ESP_OK;
  }
  
  esp_err_t ret = log_storage_flush();
  if (ret != ESP_OK) {
    log_error(log_tag, "Flush Error", "Failed to flush log storage: %s", esp_err_to_name(ret));
  }
  
  return ret;
}