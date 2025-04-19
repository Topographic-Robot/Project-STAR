/* components/pstar_error_handler/pstar_error_handler.c */

#include "pstar_error_handler.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#define TAG ("PSTAR ERROR HANDLER")

/* --- Functions --- */

/* clang-format off */
esp_err_t error_handler_init(error_handler_t* handler,
                             uint32_t         max_retries,
                             uint32_t         base_retry_delay,
                             uint32_t         max_retry_delay,
                             esp_err_t      (*reset_fn)(void* context),
                             void*            reset_context)
/* clang-format on */
{
  if (handler == NULL) {
    ESP_LOGE(TAG, "Init Error: Handler pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  /* Initialize struct members first */
  memset(handler, 0, sizeof(error_handler_t)); /* Zero out the structure initially */
  handler->max_retries = max_retries;
  handler->base_retry_delay =
    base_retry_delay > 0 ? base_retry_delay : 100; /* Ensure non-zero base delay */
  handler->max_retry_delay     = max_retry_delay > handler->base_retry_delay
                                   ? max_retry_delay
                                   : handler->base_retry_delay; /* Ensure max >= base */
  handler->current_retry_delay = handler->base_retry_delay;
  handler->reset_fn            = reset_fn;
  handler->reset_context       = reset_context;
  handler->last_error          = ESP_OK;
  handler->in_error_state      = false;
  handler->error_count         = 0;
  handler->current_retry       = 0;
  handler->mutex               = NULL; /* Explicitly NULL before creation */

  /* Create mutex last after other fields are set */
  handler->mutex = xSemaphoreCreateMutex();
  if (handler->mutex == NULL) {
    ESP_LOGE(TAG, "Mutex Error: Failed to create mutex during initialization");
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

void error_handler_deinit(error_handler_t* handler)
{
  if (handler != NULL && handler->mutex != NULL) {
    SemaphoreHandle_t temp_mutex = handler->mutex;
    handler->mutex               = NULL; /* Set to NULL first to prevent double-delete */
    vSemaphoreDelete(temp_mutex);
  }
}

/* clang-format off */
esp_err_t error_handler_record_error(error_handler_t* handler,
                                     esp_err_t        error,
                                     const char*      description,
                                     const char*      file,
                                     int              line,
                                     const char*      func)
/* clang-format on */
{
  if (handler == NULL) {
    return error; /* Return original error if handler is invalid */
  }

  /* Check for NULL description, file, func */
  const char* desc     = description ? description : "No description";
  const char* filename = file ? file : "Unknown file";
  const char* funcname = func ? func : "Unknown function";

  if (handler->mutex == NULL) {
    ESP_LOGE(TAG,
             "Mutex Error: Mutex not initialized for error handler used in %s:%d",
             filename,
             line);
    return ESP_ERR_INVALID_STATE; /* Indicate handler state issue */
  }

  if (xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Mutex Timeout: Failed to acquire mutex in %s:%d", filename, line);
    return ESP_ERR_TIMEOUT; /* Indicate mutex failure */
  }

  /* Only increment retry count if we haven't reached the max yet */
  /* This prevents the logged count from exceeding max_retries */
  uint32_t retry_count_to_log = handler->current_retry;
  if (!handler->in_error_state) {
    /* First error occurrence */
    handler->in_error_state      = true;
    handler->current_retry       = 0; /* Start at 0 for the first attempt */
    handler->current_retry_delay = handler->base_retry_delay;
    retry_count_to_log           = 0; /* Log as 0/max */
  } else if (handler->current_retry < handler->max_retries) {
    /* Subsequent error, increment retry if below max */
    handler->current_retry++;
    retry_count_to_log = handler->current_retry; /* Log the incremented value */
    /* Apply exponential backoff */
    uint64_t new_delay_64 =
      (uint64_t)handler->current_retry_delay * 2; /* Use 64-bit intermediate */
    if (new_delay_64 > handler->max_retry_delay) {
      handler->current_retry_delay = handler->max_retry_delay;
    } else if (new_delay_64 <
               handler->base_retry_delay) { /* Prevent underflow/wrap-around if base is high */
      handler->current_retry_delay = handler->base_retry_delay;
    } else {
      handler->current_retry_delay = (uint32_t)new_delay_64;
    }
    ESP_LOGI(TAG,
             "Backoff: Next retry delay: %lu ms",
             (unsigned long int)handler->current_retry_delay);
  } else {
    /* Retries already exhausted, log the max count */
    retry_count_to_log = handler->max_retries;
    /* Keep the max delay */
    handler->current_retry_delay = handler->max_retry_delay;
    ESP_LOGI(TAG,
             "Backoff: Next retry delay: %lu ms",
             (unsigned long int)handler->current_retry_delay);
  }

  /* Construct detailed log message using retry_count_to_log */
  /* Using a stack buffer, assuming total length won't exceed it. Be cautious. */
  char log_buffer[256]; /* Using reasonable buffer size for ESP logging */
  snprintf(log_buffer,
           sizeof(log_buffer),
           "Desc: %s | Code: %d (%s) | Loc: %s:%d (%s) | Retry: %lu/%lu", /* Use %lu */
           desc,
           error,
           esp_err_to_name(error),
           filename,
           line,
           funcname,
           (unsigned long)retry_count_to_log, /* Cast to unsigned long for %lu */
           (unsigned long)handler->max_retries);
  log_buffer[sizeof(log_buffer) - 1] = '\0'; /* Ensure null termination */

  ESP_LOGE(TAG, "Error Recorded: %s", log_buffer); /* Log the detailed message */

  handler->error_count++;
  handler->last_error = error;

  esp_err_t recovery_result = ESP_FAIL; /* Default to fail */

  /* Check if retries are exhausted based on the *actual* current_retry */
  if (handler->current_retry >= handler->max_retries) {
    ESP_LOGE(TAG,
             "Max Retries: Max retries (%lu) exceeded for error %d (%s).",
             (unsigned long int)handler->max_retries,
             error,
             esp_err_to_name(error));
    /* Try reset function if available as a last resort */
    if (handler->reset_fn != NULL) {
      ESP_LOGI(TAG, "Reset Attempt: Attempting reset function after max retries...");
      /* Release mutex before calling potentially blocking reset function */
      xSemaphoreGive(handler->mutex);
      recovery_result = handler->reset_fn(handler->reset_context);
      /* Re-acquire mutex after reset attempt */
      if (xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Mutex Error: Failed to re-acquire mutex after reset attempt!");
        /* Critical state, maybe restart? For now, return timeout error. */
        return ESP_ERR_TIMEOUT;
      }
      if (recovery_result == ESP_OK) {
        ESP_LOGI(TAG, "Reset Success: Reset function succeeded. Resetting error state.");
        /* Reset state only if reset function succeeds */
        handler->error_count         = 0;
        handler->current_retry       = 0;
        handler->current_retry_delay = handler->base_retry_delay;
        handler->last_error          = ESP_OK;
        handler->in_error_state      = false;
        /* Release mutex and return success */
        xSemaphoreGive(handler->mutex);
        return ESP_OK; /* Indicate recovery was successful */
      } else {
        ESP_LOGE(TAG,
                 "Reset Failed: Reset function failed with code: %d (%s)",
                 recovery_result,
                 esp_err_to_name(recovery_result));
      }
    } else {
      ESP_LOGW(TAG, "No Reset: Max retries exceeded and no reset function provided.");
    }
    /* If reset failed or wasn't available, keep error state, release mutex, return original error */
    xSemaphoreGive(handler->mutex);
    return error; /* Indicate max retries hit, recovery failed */
  } else {
    /* Retries not exhausted yet. */
    /* NOTE: Current logic does NOT call reset until max retries are hit. */

    /* Still in error state, retries remain. */
    xSemaphoreGive(handler->mutex);
    return error; /* Return original error code, indicating error persists but retries remain. */
  }
}

bool error_handler_can_retry(error_handler_t* handler)
{
  bool can_retry = false;
  if (handler == NULL) {
    return false;
  }

  if (handler->mutex == NULL) {
    ESP_LOGE(TAG, "Mutex Error: Mutex not initialized for error handler (can_retry)");
    return false;
  }

  if (xSemaphoreTake(handler->mutex, portMAX_DELAY) == pdTRUE) {
    /* Mutex acquired, proceed safely */
    can_retry = (handler->in_error_state && (handler->current_retry < handler->max_retries));
    xSemaphoreGive(handler->mutex);
  } else {
    ESP_LOGE(TAG, "Mutex Timeout: Failed to acquire mutex for can_retry check");
    /* Cannot determine state, assume no retry possible */
    can_retry = false;
  }
  return can_retry;
}

esp_err_t error_handler_reset_state(error_handler_t* handler)
{
  if (handler == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (handler->mutex == NULL) {
    ESP_LOGE(TAG, "Mutex Error: Mutex not initialized for error handler (reset_state)");
    return ESP_ERR_INVALID_STATE; /* Indicate handler state issue */
  }

  if (xSemaphoreTake(handler->mutex, portMAX_DELAY) == pdTRUE) {
    /* Mutex acquired, proceed safely */
    /* Only log if actually resetting from an error state */
    if (handler->in_error_state) {
      ESP_LOGI(TAG, "State Reset: Resetting error handler state.");
    }
    handler->error_count         = 0;
    handler->current_retry       = 0;
    handler->current_retry_delay = handler->base_retry_delay;
    handler->last_error          = ESP_OK;
    handler->in_error_state      = false;
    xSemaphoreGive(handler->mutex);
    return ESP_OK;
  } else {
    ESP_LOGE(TAG, "Mutex Timeout: Failed to acquire mutex for state reset");
    return ESP_ERR_TIMEOUT;
  }
}
