/* components/pstar_error_handler/pstar_error_handler.c */

#include "pstar_error_handler.h"

#include "pstar_log_handler.h"

#include <stdio.h>
#include <string.h>

/* Constants ******************************************************************/

static const char* TAG = "Error Handler";

/* Public Functions ***********************************************************/

esp_err_t error_handler_init(error_handler_t* handler,
                             uint32_t         max_retries,
                             uint32_t         base_retry_delay,
                             uint32_t         max_retry_delay,
                             esp_err_t (*reset_fn)(void* context),
                             void* reset_context)
{
  if (handler == NULL) {
    /* Use log_error instead of ESP_LOGE - this works even before logger initialization */
    log_error(TAG, "Init Error", "Handler pointer is NULL");
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
    log_error(TAG, "Mutex Error", "Failed to create mutex during initialization");
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

void error_handler_deinit(error_handler_t* handler)
{
  if (handler != NULL && handler->mutex != NULL) {
    vSemaphoreDelete(handler->mutex);
    handler->mutex = NULL; /* Mark mutex as deleted */
  }
}

esp_err_t error_handler_record_error(error_handler_t* handler,
                                     esp_err_t        error,
                                     const char*      description,
                                     const char*      file,
                                     int              line,
                                     const char*      func)
{
  if (handler == NULL) {
    return error; /* Return original error if handler is invalid */
  }

  /* Check for NULL description, file, func */
  const char* desc     = description ? description : "No description";
  const char* filename = file ? file : "Unknown file";
  const char* funcname = func ? func : "Unknown function";

  if (handler->mutex == NULL) {
    log_error(TAG,
              "Mutex Error",
              "Mutex not initialized for error handler used in %s:%d",
              filename,
              line);
    return ESP_ERR_INVALID_STATE; /* Indicate handler state issue */
  }

  if (xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(TAG, "Mutex Timeout", "Failed to acquire mutex in %s:%d", filename, line);
    return ESP_ERR_TIMEOUT; /* Indicate mutex failure */
  }

  /* Construct detailed log message */
  /* Using a stack buffer, assuming total length won't exceed it. Be cautious. */
  char
    log_buffer[CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH]; /* Use logger's max msg length */
  snprintf(log_buffer,
           sizeof(log_buffer),
           "Desc: %s | Code: %d (%s) | Loc: %s:%d (%s) | Retry: %lu/%lu",
           desc,
           error,
           esp_err_to_name(error),
           filename,
           line,
           funcname,
           handler->current_retry,
           handler->max_retries);
  log_buffer[sizeof(log_buffer) - 1] = '\0'; /* Ensure null termination */

  log_error(TAG, "Error Recorded", "%s", log_buffer); /* Log the detailed message */

  handler->error_count++;
  handler->last_error = error;

  esp_err_t recovery_result = ESP_FAIL; /* Default to fail */

  if (!handler->in_error_state) {
    /* First error occurrence */
    handler->in_error_state      = true;
    handler->current_retry       = 0;
    handler->current_retry_delay = handler->base_retry_delay;
  } else {
    /* Subsequent error, increment retry */
    handler->current_retry++;
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
    log_info(TAG, "Backoff", "Next retry delay: %lu ms", handler->current_retry_delay);
  }

  /* Attempt reset only if retries are exhausted or if configured differently (e.g., reset on first error) */
  if (handler->current_retry >= handler->max_retries) {
    log_error(TAG,
              "Max Retries",
              "Max retries (%lu) exceeded for error %d (%s).",
              handler->max_retries,
              error,
              esp_err_to_name(error));
    /* Try reset function if available as a last resort */
    if (handler->reset_fn != NULL) {
      log_info(TAG, "Reset Attempt", "Attempting reset function after max retries...");
      /* Release mutex before calling potentially blocking reset function */
      xSemaphoreGive(handler->mutex);
      recovery_result = handler->reset_fn(handler->reset_context);
      /* Re-acquire mutex after reset attempt */
      if (xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
        log_error(TAG, "Mutex Error", "Failed to re-acquire mutex after reset attempt!");
        /* Critical state, maybe restart? For now, return timeout error. */
        return ESP_ERR_TIMEOUT;
      }
      if (recovery_result == ESP_OK) {
        log_info(TAG, "Reset Success", "Reset function succeeded. Resetting error state.");
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
        log_error(TAG,
                  "Reset Failed",
                  "Reset function failed with code: %d (%s)",
                  recovery_result,
                  esp_err_to_name(recovery_result));
      }
    } else {
      log_warn(TAG, "No Reset", "Max retries exceeded and no reset function provided.");
    }
    /* If reset failed or wasn't available, keep error state, release mutex, return original error */
    xSemaphoreGive(handler->mutex);
    return error; /* Indicate max retries hit, recovery failed */
  } else {
    /* Retries not exhausted yet. */
    /* XXX, should we call reset function on every retry attempt here? */
    /* if (handler->reset_fn != NULL) { ... call reset_fn ... } */
    /* Current logic does NOT call reset until max retries are hit. */

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
    log_error(TAG, "Mutex Error", "Mutex not initialized for error handler (can_retry)");
    return false;
  }

  if (xSemaphoreTake(handler->mutex, portMAX_DELAY) == pdTRUE) {
    /* Mutex acquired, proceed safely */
    can_retry = (handler->in_error_state && (handler->current_retry < handler->max_retries));
    xSemaphoreGive(handler->mutex);
  } else {
    log_error(TAG, "Mutex Timeout", "Failed to acquire mutex for can_retry check");
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
    log_error(TAG, "Mutex Error", "Mutex not initialized for error handler (reset_state)");
    return ESP_ERR_INVALID_STATE; /* Indicate handler state issue */
  }

  if (xSemaphoreTake(handler->mutex, portMAX_DELAY) == pdTRUE) {
    /* Mutex acquired, proceed safely */
    /* Only log if actually resetting from an error state */
    if (handler->in_error_state) {
      log_info(TAG, "State Reset", "Resetting error handler state.");
    }
    handler->error_count         = 0;
    handler->current_retry       = 0;
    handler->current_retry_delay = handler->base_retry_delay;
    handler->last_error          = ESP_OK;
    handler->in_error_state      = false;
    xSemaphoreGive(handler->mutex);
    return ESP_OK;
  } else {
    log_error(TAG, "Mutex Timeout", "Failed to acquire mutex for state reset");
    return ESP_ERR_TIMEOUT;
  }
}
