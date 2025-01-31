/* components/common/error_handler.c */

#include "error_handler.h"
#include "esp_log.h"

/* Public Functions ***********************************************************/

void error_handler_init(error_handler_t *handler, const char *tag,
                       uint8_t max_retries, uint32_t initial_interval,
                       uint32_t max_interval, esp_err_t (*reset_func)(void *context),
                       void *context, uint32_t initial_backoff_interval,
                       uint32_t max_backoff_interval)
{
  if (!handler) {
    ESP_LOGE(tag, "Invalid handler pointer");
    return;
  }

  ESP_LOGI(tag, "Starting error handler initialization");

  handler->retry_count              = 0;
  handler->max_retries              = max_retries;
  handler->retry_interval           = initial_interval;
  handler->initial_interval         = initial_interval;
  handler->initial_backoff_interval = initial_backoff_interval;
  handler->max_interval             = max_interval;
  handler->max_backoff_interval     = max_backoff_interval;
  handler->last_attempt_ticks       = 0;
  handler->last_error               = ESP_OK;
  handler->in_error_state           = false;
  handler->tag                      = tag;
  handler->reset_func               = reset_func;
  handler->context                  = context;

  ESP_LOGI(tag, "Error handler initialization complete");
}

esp_err_t error_handler_record_error(error_handler_t *handler, esp_err_t error)
{
  if (!handler) {
    return ESP_ERR_INVALID_ARG;
  }

  handler->last_error     = error;
  handler->in_error_state = true;

  TickType_t now_ticks = xTaskGetTickCount();
  if (now_ticks - handler->last_attempt_ticks >= handler->retry_interval) {
    handler->last_attempt_ticks = now_ticks;
    handler->retry_count++;

    /* Check if we've exceeded max retries */
    if (handler->retry_count >= handler->max_retries) {
      ESP_LOGE(handler->tag, "Max retries exceeded, increasing backoff interval");
      
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
      ESP_LOGI(handler->tag, "Calling component reset function");
      ret = handler->reset_func(handler->context);
      if (ret == ESP_OK) {
        /* If reset was successful, restore initial state */
        handler->retry_interval = handler->initial_interval;
        handler->last_error     = ESP_OK;
        handler->in_error_state = false;
        handler->retry_count    = 0;
        ESP_LOGI(handler->tag, "Reset successful");
      } else {
        ESP_LOGE(handler->tag, "Reset failed: %s", esp_err_to_name(ret));
      }
    }
    return ret;
  } else {
    ESP_LOGW(handler->tag, "Reset attempted too soon, waiting %u more ticks",
             (unsigned int)(handler->retry_interval - (now_ticks - handler->last_attempt_ticks)));
    return ESP_ERR_INVALID_STATE;
  }
}

esp_err_t error_handler_reset(error_handler_t *handler)
{
  if (!handler) {
    ESP_LOGE("ERROR_HANDLER", "Invalid handler pointer");
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = ESP_OK;
  if (handler->reset_func) {
    ret = handler->reset_func(handler->context);
  }

  if (ret == ESP_OK) {
    handler->retry_interval = handler->initial_interval;
    handler->last_error     = ESP_OK;
    handler->in_error_state = false;
    handler->retry_count    = 0;
    ESP_LOGI(handler->tag, "Error handler reset complete");
  }

  return ret;
}
