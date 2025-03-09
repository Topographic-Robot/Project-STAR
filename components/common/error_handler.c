/* components/common/error_handler.c */

#include "error_handler.h"
#include "log_handler.h"

/* Constants ******************************************************************/

const char* const error_handler_tag = "Error Handler";

/* Public Functions ***********************************************************/

void error_handler_init(error_handler_t* const handler, 
                        const char* const      tag,
                        uint8_t                max_retries, 
                        uint32_t               initial_interval,
                        uint32_t               max_interval, 
                        esp_err_t            (*reset_func)(void* context),
                        void*                  context, 
                        uint32_t               initial_backoff_interval,
                        uint32_t               max_backoff_interval)
{
  if (!handler) {
    log_error(tag, "Init Error", "Handler pointer is NULL");
    return;
  }

  log_info(tag, 
           "Init Start", 
           "Initializing error handler with max_retries=%u", 
           max_retries);

  handler->retry_count              = 0;
  handler->max_retries              = max_retries;
  handler->retry_interval           = initial_interval;
  handler->initial_interval         = initial_interval;
  handler->initial_backoff_interval = initial_backoff_interval;
  handler->max_interval             = max_interval;
  handler->max_backoff_interval     = max_backoff_interval;
  handler->last_attempt_ticks       = 0;
  handler->last_status              = ESP_OK;
  handler->in_error_state           = false;
  handler->tag                      = tag;
  handler->reset_func               = reset_func;
  handler->context                  = context;

  log_info(tag, "Init Complete", "Error handler initialized successfully");
}

esp_err_t error_handler_record_status(error_handler_t* const handler, 
                                      esp_err_t              status)
{
  if (!handler) {
    return ESP_ERR_INVALID_ARG;
  }

  handler->last_status = status;

  /* If this is a success status, reset error state */
  if (status == ESP_OK) {
    if (handler->in_error_state) {
      log_info(handler->tag, "Recovery", "Component recovered from error state");
    }
    handler->in_error_state = false;
    handler->retry_count    = 0;
    handler->retry_interval = handler->initial_interval;
    return ESP_OK;
  }

  /* Handle error status */
  handler->in_error_state = true;

  TickType_t now_ticks = xTaskGetTickCount();
  if (now_ticks - handler->last_attempt_ticks >= handler->retry_interval) {
    handler->last_attempt_ticks = now_ticks;
    handler->retry_count++;

    /* Check if we've exceeded max retries */
    if (handler->retry_count >= handler->max_retries) {
      log_error(handler->tag, 
                "Retry Error", 
                "Maximum retry attempts (%u) exceeded", 
                handler->max_retries);
      
      /* Update retry interval with exponential backoff */
      handler->retry_interval = (handler->retry_interval * 2 <= handler->max_interval) ?
                                handler->retry_interval * 2 :
                                handler->max_interval;
      handler->retry_count    = 0;
      return ESP_ERR_INVALID_STATE;
    }

    /* Call component-specific reset function if set */
    esp_err_t ret = ESP_OK;
    if (handler->reset_func) {
      log_info(handler->tag, 
               "Reset Start", 
               "Attempting component reset (attempt %u/%u)", 
               handler->retry_count, 
               handler->max_retries);
      ret = handler->reset_func(handler->context);
      if (ret == ESP_OK) {
        /* If reset was successful, restore initial state */
        handler->retry_interval = handler->initial_interval;
        handler->last_status    = ESP_OK;
        handler->in_error_state = false;
        handler->retry_count    = 0;
        log_info(handler->tag, "Reset Complete", "Component reset successful");
      } else {
        log_error(handler->tag, "Reset Error", "Component reset failed");
      }
    }
    return ret;
  } else {
    log_warn(handler->tag, 
             "Retry Delay", 
             "Next retry available in %u ticks",
             (unsigned int)(handler->retry_interval - (now_ticks - handler->last_attempt_ticks)));
    return ESP_ERR_INVALID_STATE;
  }
}

esp_err_t error_handler_reset(error_handler_t* const handler)
{
  if (!handler) {
    /* handler is NULL, so we need to use "ERROR_HANDLER" as the tag */
    log_error(error_handler_tag, "Reset Error", "Handler pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = ESP_OK;
  if (handler->reset_func) {
    ret = handler->reset_func(handler->context);
  }

  if (ret == ESP_OK) {
    handler->retry_interval = handler->initial_interval;
    handler->last_status    = ESP_OK;
    handler->in_error_state = false;
    handler->retry_count    = 0;
    log_info(handler->tag, "Reset Complete", "Error handler reset successful");
  }

  return ret;
}
