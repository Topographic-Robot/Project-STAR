/* components/common/include/error_handler.h */

#ifndef TOPOROBO_ERROR_HANDLER_H
#define TOPOROBO_ERROR_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/* Common error handling structure */
typedef struct {
  uint8_t    retry_count;            /**< Number of consecutive reinitialization attempts */
  uint32_t   retry_interval;         /**< Current interval between reinitialization attempts, in ticks */
  TickType_t last_attempt_ticks;     /**< Tick count of the last reinitialization attempt */
  uint8_t    fail_count;             /**< Number of consecutive failures before triggering reset */
  uint8_t    allowed_fail_attempts;  /**< Maximum number of failures allowed before reset */
  uint8_t    max_retries;            /**< Maximum number of reset retries before increasing interval */
  uint32_t   initial_retry_interval; /**< Initial retry interval in ticks */
  uint32_t   max_backoff_interval;   /**< Maximum backoff interval in ticks */
  const char *tag;                   /**< Logging tag for the component */
} error_handler_t;

/**
 * @brief Initialize the error handler structure with default values
 *
 * @param handler Pointer to error_handler_t structure
 * @param tag Logging tag for the component
 * @param allowed_fail_attempts Number of failures allowed before reset
 * @param max_retries Maximum number of reset retries
 * @param initial_retry_interval Initial retry interval in ticks
 * @param max_backoff_interval Maximum backoff interval in ticks
 */
void error_handler_init(error_handler_t *handler,
                        const char *tag,
                        uint8_t allowed_fail_attempts,
                        uint8_t max_retries,
                        uint32_t initial_retry_interval,
                        uint32_t max_backoff_interval) 
{
  handler->retry_count            = 0;
  handler->retry_interval         = initial_retry_interval;
  handler->last_attempt_ticks     = 0;
  handler->fail_count             = 0;
  handler->allowed_fail_attempts  = allowed_fail_attempts;
  handler->max_retries            = max_retries;
  handler->initial_retry_interval = initial_retry_interval;
  handler->max_backoff_interval   = max_backoff_interval;
  handler->tag                    = tag;
}

/**
 * @brief Handle error recovery with exponential backoff
 *
 * @param handler Pointer to error_handler_t structure
 * @param current_fail_count Current number of consecutive failures
 * @param init_func Function pointer to component initialization function
 * @param init_data Pointer to component data structure
 * @return esp_err_t ESP_OK if reset successful, ESP_FAIL otherwise
 */
esp_err_t error_handler_reset(error_handler_t *handler,
                            uint8_t current_fail_count,
                            esp_err_t (*init_func)(void*),
                            void *init_data) 
{
  if (current_fail_count >= handler->allowed_fail_attempts) {
    TickType_t current_ticks = xTaskGetTickCount();
    
    if ((current_ticks - handler->last_attempt_ticks) > handler->retry_interval) {
      ESP_LOGI(handler->tag, "Attempting to reset component");
      
      esp_err_t ret = init_func(init_data);
      if (ret == ESP_OK) {
        handler->retry_count    = 0;
        handler->retry_interval = handler->initial_retry_interval;
        handler->fail_count     = 0;
        ESP_LOGI(handler->tag, "Component reset successfully");
        return ESP_OK;
      } else {
        handler->retry_count++;
        if (handler->retry_count >= handler->max_retries) {
          handler->retry_count    = 0;
          handler->retry_interval = (handler->retry_interval * 2 > handler->max_backoff_interval) ?
                                    handler->max_backoff_interval :
                                    handler->retry_interval * 2;
        }
        ESP_LOGE(handler->tag, "Component reset failed, retry count: %d, next interval: %lu",
                 handler->retry_count, handler->retry_interval);
      }
      
      handler->last_attempt_ticks = current_ticks;
      return ESP_FAIL;
    }
  }
  return ESP_FAIL;
}

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_ERROR_HANDLER_H */