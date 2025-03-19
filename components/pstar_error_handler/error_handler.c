/* components/pstar_error_handler/error_handler.c */

#include "error_handler.h"
#include "log_handler.h"
#include <string.h>

/* Constants ******************************************************************/

static const char* const error_handler_tag = "Error Handler";
  
/* Public Functions ***********************************************************/

void error_handler_init(error_handler_t* handler, 
                        uint32_t         max_retries,
                        uint32_t         base_retry_delay,
                        uint32_t         max_retry_delay,
                        esp_err_t      (*reset_fn)(void* context),
                        void*            reset_context)
{
  if (handler == NULL) {
    return;
  }
  
  if (handler->mutex == NULL) {
    handler->mutex = xSemaphoreCreateMutex();
    if (handler->mutex == NULL) {
      log_error(error_handler_tag, 
                "Mutex Error", 
                "Failed to create mutex during initialization");
      return;
    }
  }
  
  if (xSemaphoreTake(handler->mutex, portMAX_DELAY) == pdTRUE) {
    handler->error_count         = 0;
    handler->max_retries         = max_retries;
    handler->current_retry       = 0;
    handler->base_retry_delay    = base_retry_delay;
    handler->max_retry_delay     = max_retry_delay;
    handler->current_retry_delay = base_retry_delay;
    handler->last_error          = ESP_OK;
    handler->in_error_state      = false;
    handler->reset_fn            = reset_fn;
    handler->reset_context       = reset_context;
    xSemaphoreGive(handler->mutex);
  }
}

esp_err_t error_handler_record_error(error_handler_t* handler, 
                                     esp_err_t        error, 
                                     const char*      description,
                                     const char*      file, 
                                     int              line)
{
  if (handler == NULL) {
    return error;
  }
  
  if (xSemaphoreTake(handler->mutex, portMAX_DELAY) == pdTRUE) {
    log_error(error_handler_tag, 
              "Error Recorded", 
              "Desc: %s | Code: %d | File: %s | Line: %d | Retry: %lu", 
              description, 
              error, 
              file, 
              line, 
              handler->current_retry);
    
    handler->error_count++;
    handler->last_error = error;
    
    if (!handler->in_error_state) {
      handler->in_error_state      = true;
      handler->current_retry       = 0;
      handler->current_retry_delay = handler->base_retry_delay;
    } else {
      handler->current_retry++;
      uint32_t new_delay = handler->current_retry_delay * 2;
      handler->current_retry_delay = (new_delay > handler->max_retry_delay) 
                                     ? handler->max_retry_delay : new_delay;
    }
    
    if (handler->current_retry < handler->max_retries) {
      if (handler->reset_fn != NULL) {
        esp_err_t reset_result = handler->reset_fn(handler->reset_context);
        if (reset_result == ESP_OK) {
          error_handler_reset_state(handler);
          xSemaphoreGive(handler->mutex);
          return ESP_OK;
        } else {
          log_error(error_handler_tag, 
                    "Reset Failed", 
                    "Reset function failed with code: %d on retry %lu", 
                    reset_result, 
                    handler->current_retry);
        }
      }
    } else {
      log_error(error_handler_tag, 
                "Max Retries", 
                "Max retries exceeded (%lu attempts). No further attempts will be made.", 
                handler->max_retries);
    }
    xSemaphoreGive(handler->mutex);
    return error;
  }
  return error;
}

bool error_handler_can_retry(error_handler_t* handler)
{
  bool can_retry = false;
  if (handler == NULL) {
    return false;
  }
  
  if (xSemaphoreTake(handler->mutex, portMAX_DELAY) == pdTRUE) {
    can_retry = (handler->in_error_state && (handler->current_retry < handler->max_retries));
    xSemaphoreGive(handler->mutex);
  }
  return can_retry;
}

void error_handler_reset_state(error_handler_t* handler)
{
  if (handler == NULL) {
    return;
  }
  
  if (xSemaphoreTake(handler->mutex, portMAX_DELAY) == pdTRUE) {
    handler->error_count         = 0;
    handler->current_retry       = 0;
    handler->current_retry_delay = handler->base_retry_delay;
    handler->last_error          = ESP_OK;
    handler->in_error_state      = false;
    xSemaphoreGive(handler->mutex);
  }
}
