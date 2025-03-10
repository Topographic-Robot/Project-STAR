/* components/common/error_handler.c */

#include "error_handler.h"
#include "log_handler.h"
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include "common/common_cleanup.h"

/* Forward Declarations for Private Functions *********************************/

static uint64_t priv_get_timestamp(void);
static TickType_t priv_safe_tick_diff(TickType_t current_ticks, TickType_t previous_ticks);
static esp_err_t priv_propagate_error(const error_handler_t* handler, const error_info_t* error_info);
static esp_err_t priv_execute_recovery_strategy(error_handler_t* handler);

/* Constants ******************************************************************/

const char* const error_handler_tag = "Error Handler";

/* Static Variables ***********************************************************/

static component_info_t  s_registered_components[MAX_COMPONENTS] = {0};   /**< Registered components */
static uint32_t          s_component_count                       = 0;     /**< Number of registered components */
static bool              s_system_initialized                    = false; /**< System initialized flag */
static SemaphoreHandle_t s_error_handler_mutex                   = NULL;   /**< Mutex for thread safety */

static const char* const s_error_category_strings[k_error_category_count] = {
  "None",
  "Hardware",
  "Communication",
  "Memory",
  "System",
  "Application",
  "Security",
  "Power",
  "Timing",
  "Configuration"
};

static const char* const s_error_severity_strings[k_error_severity_count] = {
  "None",
  "Info",
  "Low",
  "Medium",
  "High",
  "Critical"
};

static const char* const s_recovery_strategy_strings[k_recovery_strategy_count] = {
  "None",
  "Retry",
  "Reset",
  "Alternate",
  "Degrade",
  "Shutdown",
  "Custom"
};

/* Public Functions ***********************************************************/

esp_err_t error_handler_system_init(void)
{
  if (s_system_initialized) {
    log_warn(error_handler_tag, 
             "Init Warning", 
             "Error handling system already initialized");
    return ESP_OK;
  }

  /* Create mutex for thread safety */
  s_error_handler_mutex = xSemaphoreCreateMutex();
  if (s_error_handler_mutex == NULL) {
    log_error(error_handler_tag, 
              "Init Error", 
              "Failed to create error handler mutex");
    return ESP_ERR_NO_MEM;
  }

  /* Initialize component registry */
  memset(s_registered_components, 0, sizeof(s_registered_components));
  s_component_count    = 0;
  s_system_initialized = true;

  log_info(error_handler_tag, 
           "Init Complete", 
           "Error handling system initialized");
  return ESP_OK;
}

void error_handler_init(error_handler_t* const handler, 
                        const char* const      tag,
                        uint8_t                max_retries, 
                        uint32_t               initial_interval,
                        uint32_t               max_interval, 
                        reset_func_t           reset_func,
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

  /* Initialize basic fields */
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
  handler->tag                      = tag ? tag : error_handler_tag;
  handler->reset_func               = reset_func;
  handler->context                  = context;
  handler->propagate_errors         = true;
  handler->mutex                    = xSemaphoreCreateMutex(); /* Create per-handler mutex */

  /* Initialize enhanced fields */
  memset(&handler->last_error, 0, sizeof(error_info_t));
  handler->recovery_strategy        = k_recovery_strategy_retry; /* Default to retry */
  handler->recovery_func            = NULL;
  memset(&handler->recovery_context, 0, sizeof(recovery_context_t));
  handler->error_callback           = NULL;
  handler->callback_context         = NULL;
  handler->error_count              = 0;
  handler->recovery_count           = 0;

  /* Initialize filtering and correlation */
  handler->min_severity_to_handle = k_error_severity_low;  /* Default to handle all but info */
  handler->categories_to_filter   = 0;                     /* No categories filtered by default */
  handler->next_correlation_id    = 1;                     /* Start with ID 1 */
  handler->correlations           = NULL;                  /* Allocate on first use */
  handler->correlation_count      = 0;
  handler->max_correlations       = MAX_CORRELATIONS;

  /* Initialize root cause analysis */
  handler->root_causes            = NULL;                  /* Allocate on first use */
  handler->root_cause_count       = 0;
  handler->max_root_causes        = MAX_ROOT_CAUSES;

  /* Initialize backoff parameters if provided */
  if (initial_backoff_interval > 0) {
    handler->initial_backoff_interval = initial_backoff_interval;
  } else {
    handler->initial_backoff_interval = initial_interval;
  }

  if (max_backoff_interval > 0) {
    handler->max_backoff_interval = max_backoff_interval;
  } else {
    handler->max_backoff_interval = max_interval;
  }

  if (handler->mutex == NULL) {
    log_error(tag, "Init Error", "Failed to create handler mutex");
  }
  
  log_info(tag, "Init Complete", "Error handler initialized successfully");
}

esp_err_t error_handler_record_status(error_handler_t* const handler, 
                                      esp_err_t              status)
{
  if (!handler) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Record Status Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  handler->last_status = status;

  /* If this is a success status, reset error state */
  if (status == ESP_OK) {
    if (handler->in_error_state) {
      log_info(handler->tag, "Recovery", "Component recovered from error state");
      handler->recovery_count++;
    }
    handler->in_error_state = false;
    handler->retry_count    = 0;
    handler->retry_interval = handler->initial_interval;
    
    if (handler->mutex) {
      xSemaphoreGive(handler->mutex);
    }
    return ESP_OK;
  }

  /* Handle error status */
  handler->in_error_state = true;
  handler->error_count++;

  /* Create basic error info for backward compatibility */
  error_info_t error_info = {
    .code        = status,
    .category    = k_error_category_none,
    .severity    = k_error_severity_medium, /* Default to medium severity */
    .file        = NULL,
    .line        = 0,
    .func        = NULL,
    .description = "Error status recorded",
    .timestamp   = priv_get_timestamp(),
    .count       = 1
  };

  /* Store as last error */
  memcpy(&handler->last_error, &error_info, sizeof(error_info_t));

  /* Call error callback if set */
  if (handler->error_callback) {
    handler->error_callback(&error_info, handler->callback_context);
  }

  /* Propagate error if enabled */
  if (handler->propagate_errors) {
    priv_propagate_error(handler, &error_info);
  }

  TickType_t now_ticks     = xTaskGetTickCount();
  TickType_t elapsed_ticks = priv_safe_tick_diff(now_ticks, handler->last_attempt_ticks);
  
  if (elapsed_ticks >= handler->retry_interval) {
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
      
      /* Release mutex */
      if (handler->mutex) {
        xSemaphoreGive(handler->mutex);
      }
      return ESP_ERR_INVALID_STATE;
    }

    /* Execute recovery based on strategy */
    esp_err_t ret = priv_execute_recovery_strategy(handler);
    
    /* Release mutex */
    if (handler->mutex) {
      xSemaphoreGive(handler->mutex);
    }
    return ret;
  } else {
    log_warn(handler->tag, 
             "Retry Delay", 
             "Next retry available in %lu ticks",
             (uint32_t)(handler->retry_interval - elapsed_ticks));
    
    /* Release mutex */
    if (handler->mutex) {
      xSemaphoreGive(handler->mutex);
    }
    return ESP_ERR_INVALID_STATE;
  }
}

esp_err_t error_handler_record_error(error_handler_t* const handler,
                                     esp_err_t              code,
                                     error_category_t       category,
                                     error_severity_t       severity,
                                     const char*            file,
                                     int                    line,
                                     const char*            func,
                                     const char*            desc)
{
  if (!handler) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Validate category and severity */
  if (category >= k_error_category_count || severity >= k_error_severity_count) {
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if this error should be filtered out */
  if (severity < handler->min_severity_to_handle) {
    /* Error is below minimum severity threshold, silently ignore */
    return code;
  }
  
  /* Check if this error category is filtered */
  if ((1 << category) & handler->categories_to_filter) {
    /* Error category is filtered, silently ignore */
    return code;
  }

  /* Create detailed error info */
  error_info_t error_info = {
    .code        = code,
    .category    = category,
    .severity    = severity,
    .file        = file,
    .line        = line,
    .func        = func,
    .description = desc,
    .timestamp   = priv_get_timestamp(),
    .count       = 1
  };

  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Record Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  /* Check if this is the same error as last time */
  if (handler->last_error.code == code && 
      handler->last_error.category == category &&
      handler->last_error.severity == severity) {
    handler->last_error.count++;
  } else {
    /* Store as new last error */
    memcpy(&handler->last_error, &error_info, sizeof(error_info_t));
  }

  /* Log the error with appropriate level based on severity */
  switch (severity) {
    case k_error_severity_info:
      log_info(handler->tag, 
               error_category_to_string(category), 
               "%s [%s:%d in %s]", 
               desc, file, line, func);
      break;
    case k_error_severity_low:
      log_warn(handler->tag, 
               error_category_to_string(category), 
               "%s [%s:%d in %s]", 
               desc, file, line, func);
      break;
    case k_error_severity_medium:
    case k_error_severity_high:
    case k_error_severity_critical:
      log_error(handler->tag, 
                error_category_to_string(category), 
                "%s [%s:%d in %s]", 
                desc, file, line, func);
      break;
    default:
      log_error(handler->tag, 
                "Unknown Severity", 
                "%s [%s:%d in %s]", 
                desc, file, line, func);
      break;
  }

  /* Call error callback if set */
  if (handler->error_callback) {
    handler->error_callback(&error_info, handler->callback_context);
  }

  /* Propagate error if enabled and severity is high enough */
  if (handler->propagate_errors && 
      (severity == k_error_severity_high || 
      severity == k_error_severity_critical)) {
    priv_propagate_error(handler, &error_info);
  }

  /* For critical errors, always start recovery */
  if (severity == k_error_severity_critical) {
    esp_err_t ret = error_handler_start_recovery(handler);
    
    /* Release mutex */
    if (handler->mutex) {
      xSemaphoreGive(handler->mutex);
    }
    return ret;
  }

  /* For non-critical errors, just record the status */
  handler->last_status    = code;
  handler->in_error_state = (code != ESP_OK);
  handler->error_count++;

  /* Release mutex */
  if (handler->mutex) {
    xSemaphoreGive(handler->mutex);
  }

  return code;
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
    
    /* Clear last error */
    memset(&handler->last_error, 0, sizeof(error_info_t));
    
    log_info(handler->tag, "Reset Complete", "Error handler reset successful");
    handler->recovery_count++;
  }

  return ret;
}

esp_err_t error_handler_register_component(const component_info_t* component_info)
{
  if (!s_system_initialized) {
    log_error(error_handler_tag, "Register Error", "Error handling system not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!component_info || !component_info->component_id || !component_info->handler) {
    log_error(error_handler_tag, "Register Error", "Invalid component information");
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = ESP_OK;
  
  /* Take mutex for thread safety */
  if (xSemaphoreTake(s_error_handler_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(error_handler_tag, "Register Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  
  /* Critical section starts */
  
  /* Check if we've reached the maximum number of components */
  if (s_component_count >= MAX_COMPONENTS) {
    log_error(error_handler_tag, 
              "Register Error", 
              "Maximum number of components (%d) reached", 
              MAX_COMPONENTS);
    ret = ESP_ERR_NO_MEM;
    goto exit;
  }

  /* Check if component is already registered */
  for (uint32_t i = 0; i < s_component_count; i++) {
    if (strcmp(s_registered_components[i].component_id, component_info->component_id) == 0) {
      log_warn(error_handler_tag, 
               "Register Warning", 
               "Component '%s' already registered, updating", 
               component_info->component_id);
      
      /* Update existing registration */
      s_registered_components[i] = *component_info;
      ret = ESP_OK;
      goto exit;
    }
  }

  /* Register new component */
  s_registered_components[s_component_count] = *component_info;
  s_component_count++;

  log_info(error_handler_tag, 
           "Register Complete", 
           "Component '%s' registered successfully (total: %lu)", 
           component_info->component_id, 
           (uint32_t)s_component_count);

exit:
  /* Release mutex */
  xSemaphoreGive(s_error_handler_mutex);
  return ret;
}

esp_err_t error_handler_set_callback(error_handler_t* const handler,
                                     error_handler_func_t   callback,
                                     void*                  context)
{
  if (!handler) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Set Callback Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  handler->error_callback   = callback;
  handler->callback_context = context;

  log_info(handler->tag, 
           "Callback Set", 
           "Error callback %s", 
           callback ? "registered" : "cleared");

  /* Release mutex */
  if (handler->mutex) {
    xSemaphoreGive(handler->mutex);
  }

  return ESP_OK;
}

esp_err_t error_handler_set_recovery(error_handler_t* const handler,
                                     recovery_strategy_t     strategy,
                                     recovery_func_t         func,
                                     void*                   context)
{
  if (!handler) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Validate strategy */
  if (strategy >= k_recovery_strategy_count) {
    return ESP_ERR_INVALID_ARG;
  }

  /* For custom strategy, a function must be provided */
  if (strategy == k_recovery_strategy_custom && func == NULL) {
    log_error(handler->tag, 
              "Recovery Error", 
              "Custom recovery strategy requires a function");
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Set Recovery Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  handler->recovery_strategy             = strategy;
  handler->recovery_func                 = func;
  handler->recovery_context.context      = context;
  handler->recovery_context.strategy     = strategy;
  handler->recovery_context.attempt      = 0;
  handler->recovery_context.max_attempts = handler->max_retries;
  handler->recovery_context.in_progress  = false;

  log_info(handler->tag, 
           "Recovery Set", 
           "Recovery strategy set to %s", 
           recovery_strategy_to_string(strategy));

  /* Release mutex */
  if (handler->mutex) {
    xSemaphoreGive(handler->mutex);
  }

  return ESP_OK;
}

esp_err_t error_handler_start_recovery(error_handler_t* const handler)
{
  if (!handler) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!handler->in_error_state) {
    log_info(handler->tag, "Recovery Info", "No recovery needed, component not in error state");
    return ESP_OK;
  }

  if (handler->recovery_context.in_progress) {
    log_warn(handler->tag, "Recovery Warning", "Recovery already in progress");
    return ESP_ERR_INVALID_STATE;
  }

  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Start Recovery Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  /* Initialize recovery context */
  handler->recovery_context.attempt      = 0;
  handler->recovery_context.max_attempts = handler->max_retries;
  handler->recovery_context.timeout      = xTaskGetTickCount() + pdMS_TO_TICKS(5000); /* 5 second timeout */
  handler->recovery_context.in_progress  = true;

  log_info(handler->tag, 
           "Recovery Start", 
           "Starting recovery with strategy: %s", 
           recovery_strategy_to_string(handler->recovery_strategy));

  /* Execute recovery strategy */
  esp_err_t ret = priv_execute_recovery_strategy(handler);

  /* Release mutex */
  if (handler->mutex) {
    xSemaphoreGive(handler->mutex);
  }

  return ret;
}

esp_err_t error_handler_get_status(const error_handler_t* const handler,
                                   error_info_t*                info)
{
  if (!handler || !info) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Get Status Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  /* Copy last error information */
  memcpy(info, &handler->last_error, sizeof(error_info_t));

  /* Release mutex */
  if (handler->mutex) {
    xSemaphoreGive(handler->mutex);
  }

  return ESP_OK;
}

esp_err_t error_handler_process(void)
{
  if (!s_system_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t result = ESP_OK;

  /* Take mutex for thread safety */
  if (xSemaphoreTake(s_error_handler_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(error_handler_tag, "Process Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  /* Process all registered components */
  for (uint32_t i = 0; i < s_component_count; i++) {
    error_handler_t* handler = s_registered_components[i].handler;
    
    if (!handler) {
      continue;
    }

    /* Check if component is in error state and recovery is in progress */
    if (handler->in_error_state && handler->recovery_context.in_progress) {
      /* Check if recovery has timed out */
      if (xTaskGetTickCount() > handler->recovery_context.timeout) {
        log_error(handler->tag, 
                  "Recovery Error", 
                  "Recovery timed out after %u attempts", 
                  (unsigned int)handler->recovery_context.attempt);
        
        handler->recovery_context.in_progress = false;
        result = ESP_ERR_TIMEOUT;
        continue;
      }

      /* Execute recovery strategy */
      esp_err_t ret = priv_execute_recovery_strategy(handler);
      if (ret != ESP_OK) {
        result = ret;
      }
    }
  }

  /* Release mutex */
  xSemaphoreGive(s_error_handler_mutex);

  return result;
}

const char* error_category_to_string(error_category_t category)
{
  if (category >= k_error_category_count) {
    return "Unknown";
  }
  return s_error_category_strings[category];
}

const char* error_severity_to_string(error_severity_t severity)
{
  if (severity >= k_error_severity_count) {
    return "Unknown";
  }
  return s_error_severity_strings[severity];
}

const char* recovery_strategy_to_string(recovery_strategy_t strategy)
{
  if (strategy >= k_recovery_strategy_count) {
    return "Unknown";
  }
  return s_recovery_strategy_strings[strategy];
}

/* New Functions for Error Filtering, Correlation, and Root Cause Analysis */

esp_err_t error_handler_set_filtering(error_handler_t* const handler,
                                      error_severity_t       min_severity,
                                      uint32_t               categories)
{
  if (!handler) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Set Filtering Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  handler->min_severity_to_handle = min_severity;
  handler->categories_to_filter   = categories;

  log_info(handler->tag, 
           "Filtering Setup", 
           "Set minimum severity to %s, filtered categories: 0x%08lx", 
           error_severity_to_string(min_severity), 
           (uint32_t)categories);

  /* Release mutex */
  if (handler->mutex) {
    xSemaphoreGive(handler->mutex);
  }

  return ESP_OK;
}

esp_err_t error_handler_correlate_errors(error_handler_t* const handler,
                                         error_info_t*          error_info,
                                         error_info_t*          related_error)
{
  if (!handler || !error_info || !related_error) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Correlation Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  /* Check if we need to allocate correlation array */
  if (!handler->correlations) {
    handler->correlations = calloc(handler->max_correlations, sizeof(error_correlation_t));
    if (!handler->correlations) {
      log_error(handler->tag, 
                "Correlation Error", 
                "Failed to allocate memory for correlations");
      if (handler->mutex) {
        xSemaphoreGive(handler->mutex);
      }
      return ESP_ERR_NO_MEM;
    }
  }

  /* Check if either error already has a correlation ID */
  uint32_t correlation_id = 0;
  
  if (error_info->correlation_id > 0) {
    correlation_id = error_info->correlation_id;
  } else if (related_error->correlation_id > 0) {
    correlation_id = related_error->correlation_id;
  } else {
    /* Create a new correlation */
    correlation_id = handler->next_correlation_id++;
  }

  /* Find or create correlation entry */
  error_correlation_t* correlation = NULL;
  
  for (uint32_t i = 0; i < handler->correlation_count; i++) {
    if (handler->correlations[i].correlation_id == correlation_id) {
      correlation = &handler->correlations[i];
      break;
    }
  }

  if (!correlation) {
    /* Create new correlation if we have space */
    if (handler->correlation_count >= handler->max_correlations) {
      /* Try to purge old correlations first */
      error_handler_purge_old_correlations(handler, 3600000); /* 1 hour old */
      
      /* Check if we have space now */
      if (handler->correlation_count >= handler->max_correlations) {
        log_error(handler->tag, 
                  "Correlation Error", 
                  "Maximum number of correlations reached");
        if (handler->mutex) {
          xSemaphoreGive(handler->mutex);
        }
        return ESP_ERR_NO_MEM;
      }
    }

    correlation                   = &handler->correlations[handler->correlation_count++];
    correlation->correlation_id   = correlation_id;
    correlation->related_count    = 0;
    correlation->related_errors   = calloc(MAX_RELATED_ERRORS, sizeof(error_info_t));
    correlation->first_occurrence = error_info->timestamp;
    correlation->last_occurrence  = error_info->timestamp;
    correlation->is_root_cause    = false;
    
    if (!correlation->related_errors) {
      log_error(handler->tag, 
                "Correlation Error", 
                "Failed to allocate memory for related errors");
      handler->correlation_count--;
      if (handler->mutex) {
        xSemaphoreGive(handler->mutex);
      }
      return ESP_ERR_NO_MEM;
    }
  }

  /* Update correlation timestamps */
  if (error_info->timestamp < correlation->first_occurrence) {
    correlation->first_occurrence = error_info->timestamp;
  }
  if (error_info->timestamp > correlation->last_occurrence) {
    correlation->last_occurrence = error_info->timestamp;
  }

  /* Improved similarity check for errors */
  bool error_info_added = false;
  bool related_error_added = false;
  
  for (uint32_t i = 0; i < correlation->related_count; i++) {
    /* Check if errors are similar enough to be considered the same */
    bool error_info_similar = (correlation->related_errors[i].code == error_info->code &&
                              correlation->related_errors[i].category == error_info->category);
                              
    bool related_error_similar = (correlation->related_errors[i].code == related_error->code &&
                                 correlation->related_errors[i].category == related_error->category);
    
    /* If file paths are available, check if they're from the same file */
    if (error_info_similar && correlation->related_errors[i].file && error_info->file) {
      /* Extract just the filename without path for comparison */
      const char* existing_filename = strrchr(correlation->related_errors[i].file, '/');
      const char* new_filename      = strrchr(error_info->file, '/');
      
      existing_filename = existing_filename ? existing_filename + 1 : correlation->related_errors[i].file;
      new_filename = new_filename ? new_filename + 1 : error_info->file;
      
      /* If filenames match, consider it the same error */
      if (strcmp(existing_filename, new_filename) == 0) {
        error_info_added = true;
        /* Update count for this error */
        correlation->related_errors[i].count++;
      }
    }
    
    /* Same check for related error */
    if (related_error_similar && correlation->related_errors[i].file && related_error->file) {
      const char* existing_filename = strrchr(correlation->related_errors[i].file, '/');
      const char* new_filename      = strrchr(related_error->file, '/');
      
      existing_filename = existing_filename ? existing_filename + 1 : correlation->related_errors[i].file;
      new_filename = new_filename ? new_filename + 1 : related_error->file;
      
      if (strcmp(existing_filename, new_filename) == 0) {
        related_error_added = true;
        correlation->related_errors[i].count++;
      }
    }
  }

  /* Add errors to correlation if not already present */
  if (!error_info_added && correlation->related_count < MAX_RELATED_ERRORS) {
    memcpy(&correlation->related_errors[correlation->related_count++], 
           error_info, 
           sizeof(error_info_t));
    
    /* Update the correlation ID in the original error */
    error_info->correlation_id = correlation_id;
  }

  if (!related_error_added && correlation->related_count < MAX_RELATED_ERRORS) {
    memcpy(&correlation->related_errors[correlation->related_count++], 
           related_error, 
           sizeof(error_info_t));
    
    /* Update the correlation ID in the related error */
    related_error->correlation_id = correlation_id;
  }

  log_info(handler->tag, 
           "Error Correlation", 
           "Correlated errors with ID %lu, total related: %lu", 
           (uint32_t)correlation_id, 
           (uint32_t)correlation->related_count);

  /* Release mutex */
  if (handler->mutex) {
    xSemaphoreGive(handler->mutex);
  }
  
  return ESP_OK;
}

esp_err_t error_handler_get_correlation(const error_handler_t* const handler,
                                        uint32_t                     correlation_id,
                                        error_correlation_t*         correlation)
{
  if (!handler || !correlation || correlation_id == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!handler->correlations) {
    return ESP_ERR_NOT_FOUND;
  }

  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Get Correlation Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  /* Find correlation by ID */
  for (uint32_t i = 0; i < handler->correlation_count; i++) {
    if (handler->correlations[i].correlation_id == correlation_id) {
      /* Copy correlation data */
      memcpy(correlation, &handler->correlations[i], sizeof(error_correlation_t));
      
      /* Don't copy the pointer, as it might be freed later */
      correlation->related_errors = NULL;
      
      /* Release mutex */
      if (handler->mutex) {
        xSemaphoreGive(handler->mutex);
      }
      
      return ESP_OK;
    }
  }

  /* Release mutex */
  if (handler->mutex) {
    xSemaphoreGive(handler->mutex);
  }

  return ESP_ERR_NOT_FOUND;
}

esp_err_t error_handler_analyze_root_cause(error_handler_t* const handler,
                                           error_info_t*          error_info,
                                           root_cause_info_t*     root_cause)
{
  if (!handler || !error_info || !root_cause) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Analyze Root Cause Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  /* Check if we need to allocate root causes array */
  if (!handler->root_causes) {
    handler->root_causes = calloc(handler->max_root_causes, sizeof(root_cause_info_t));
    if (!handler->root_causes) {
      log_error(handler->tag, 
                "Root Cause Analysis Error", 
                "Failed to allocate memory for root causes");
      if (handler->mutex) {
        xSemaphoreGive(handler->mutex);
      }
      return ESP_ERR_NO_MEM;
    }
  }

  /* Initialize root cause info */
  memset(root_cause, 0, sizeof(root_cause_info_t));
  
  /* Copy the current error as potential root cause */
  memcpy(&root_cause->root_error, error_info, sizeof(error_info_t));
  root_cause->dependent_count  = 0;
  root_cause->dependent_errors = NULL;
  root_cause->confidence       = 0.5f; /* Default confidence */

  /* Analyze based on error category and severity */
  switch (error_info->category) {
    case k_error_category_hardware:
      root_cause->diagnosis  = "Hardware failure detected";
      root_cause->confidence = 0.8f;
      break;
      
    case k_error_category_communication:
      root_cause->diagnosis  = "Communication failure, check connections";
      root_cause->confidence = 0.7f;
      break;
      
    case k_error_category_memory:
      root_cause->diagnosis  = "Memory allocation or access issue";
      root_cause->confidence = 0.9f;
      break;
      
    case k_error_category_system:
      root_cause->diagnosis  = "System-level failure";
      root_cause->confidence = 0.6f;
      break;
      
    case k_error_category_power:
      root_cause->diagnosis  = "Power-related issue detected";
      root_cause->confidence = 0.85f;
      break;
      
    default:
      root_cause->diagnosis  = "Unknown root cause";
      root_cause->confidence = 0.4f;
      break;
  }

  /* Check for correlated errors to improve analysis */
  if (error_info->correlation_id > 0) {
    error_correlation_t correlation;
    if (error_handler_get_correlation(handler, error_info->correlation_id, &correlation) == ESP_OK) {
      /* If this is the first error in the correlation, it's likely the root cause */
      if (error_info->timestamp == correlation.first_occurrence) {
        root_cause->confidence += 0.2f;
        if (root_cause->confidence > 1.0f) {
          root_cause->confidence = 1.0f;
        }
      }
    }
  }

  /* Store root cause if confidence is high enough */
  if (root_cause->confidence >= 0.7f) {
    /* Check if we already have this root cause */
    bool found = false;
    for (uint32_t i = 0; i < handler->root_cause_count; i++) {
      if (handler->root_causes[i].root_error.code == root_cause->root_error.code &&
          handler->root_causes[i].root_error.category == root_cause->root_error.category &&
          handler->root_causes[i].root_error.line == root_cause->root_error.line &&
          strcmp(handler->root_causes[i].root_error.file, root_cause->root_error.file) == 0) {
        found = true;
        break;
      }
    }
    
    /* Add new root cause if not found and we have space */
    if (!found && handler->root_cause_count < handler->max_root_causes) {
      memcpy(&handler->root_causes[handler->root_cause_count++], 
             root_cause, 
             sizeof(root_cause_info_t));
      
      log_info(handler->tag, 
               "Root Cause Analysis", 
               "New root cause identified: %s (confidence: %.1f%%)", 
               root_cause->diagnosis, 
               root_cause->confidence * 100.0f);
    }
  }

  /* Release mutex */
  if (handler->mutex) {
    xSemaphoreGive(handler->mutex);
  }

  return ESP_OK;
}

esp_err_t error_handler_get_root_cause(const error_handler_t* const handler,
                                       const error_info_t*          error_info,
                                       root_cause_info_t*           root_cause)
{
  if (!handler || !error_info || !root_cause) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!handler->root_causes) {
    return ESP_ERR_NOT_FOUND;
  }

  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Get Root Cause Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  /* First check if this error is itself a root cause */
  for (uint32_t i = 0; i < handler->root_cause_count; i++) {
    if (handler->root_causes[i].root_error.code == error_info->code &&
        handler->root_causes[i].root_error.category == error_info->category &&
        handler->root_causes[i].root_error.line == error_info->line &&
        strcmp(handler->root_causes[i].root_error.file, error_info->file) == 0) {
      
      /* Copy root cause data */
      memcpy(root_cause, &handler->root_causes[i], sizeof(root_cause_info_t));
      
      /* Don't copy the pointer, as it might be freed later */
      root_cause->dependent_errors = NULL;
      
      /* Release mutex */
      if (handler->mutex) {
        xSemaphoreGive(handler->mutex);
      }
      
      return ESP_OK;
    }
  }

  /* If error has a correlation ID, check if any correlated error is a root cause */
  if (error_info->correlation_id > 0) {
    error_correlation_t correlation;
    if (error_handler_get_correlation(handler, error_info->correlation_id, &correlation) == ESP_OK) {
      /* Check if the first error in the correlation is a root cause */
      for (uint32_t i = 0; i < handler->root_cause_count; i++) {
        if (handler->root_causes[i].root_error.timestamp == correlation.first_occurrence) {
          /* Copy root cause data */
          memcpy(root_cause, &handler->root_causes[i], sizeof(root_cause_info_t));
          
          /* Don't copy the pointer, as it might be freed later */
          root_cause->dependent_errors = NULL;
          
          /* Release mutex */
          if (handler->mutex) {
            xSemaphoreGive(handler->mutex);
          }
          
          return ESP_OK;
        }
      }
    }
  }

  /* Release mutex */
  if (handler->mutex) {
    xSemaphoreGive(handler->mutex);
  }

  return ESP_ERR_NOT_FOUND;
}

/* Private Functions **********************************************************/

static esp_err_t priv_propagate_error(const error_handler_t* handler, 
                                      const error_info_t*    error_info)
{
  if (!s_system_initialized || !handler || !error_info) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread safety */
  if (xSemaphoreTake(s_error_handler_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(error_handler_tag, "Propagation Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  
  /* Find the component in the registry */
  const char*    component_id          = NULL;
  const char*    parent_id             = NULL;
  uint32_t       propagation_depth     = 0;
  const uint32_t max_propagation_depth = 5; /* Prevent infinite loops */

  for (uint32_t i = 0; i < s_component_count; i++) {
    if (s_registered_components[i].handler == handler) {
      component_id = s_registered_components[i].component_id;
      parent_id    = s_registered_components[i].parent_id;
      break;
    }
  }

  if (!component_id || !parent_id) {
    /* Component not found or no parent */
    xSemaphoreGive(s_error_handler_mutex);
    return ESP_OK;
  }

  /* Propagate up the chain until we reach the top or max depth */
  while (parent_id && propagation_depth < max_propagation_depth) {
    bool parent_found = false;
    
    for (uint32_t i = 0; i < s_component_count; i++) {
      if (strcmp(s_registered_components[i].component_id, parent_id) == 0) {
        /* Found parent component */
        error_handler_t* parent_handler    = s_registered_components[i].handler;
        const char*      current_component = component_id;
        
        /* Update for next iteration */
        component_id = parent_id;
        parent_id    = s_registered_components[i].parent_id;
        parent_found = true;
        
        log_info(error_handler_tag, 
                 "Error Propagation", 
                 "Propagating error from '%s' to parent '%s'", 
                 current_component, 
                 component_id);
        
        /* Release mutex before calling into another handler */
        xSemaphoreGive(s_error_handler_mutex);
        
        /* Record the error in the parent */
        esp_err_t ret = error_handler_record_error(parent_handler,
                                                  error_info->code,
                                                  error_info->category,
                                                  error_info->severity,
                                                  error_info->file,
                                                  error_info->line,
                                                  error_info->func,
                                                  error_info->description);
        
        /* Take mutex again for next iteration */
        if (xSemaphoreTake(s_error_handler_mutex, portMAX_DELAY) != pdTRUE) {
          log_error(error_handler_tag, "Propagation Error", "Failed to retake mutex");
          return ret;
        }
        
        break;
      }
    }
    
    if (!parent_found) {
      /* Parent not found, stop propagation */
      break;
    }
    
    propagation_depth++;
  }
  
  xSemaphoreGive(s_error_handler_mutex);
  return ESP_OK;
}

static esp_err_t priv_execute_recovery_strategy(error_handler_t* handler)
{
  if (!handler) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = ESP_FAIL;

  /* Initialize recovery context if not already done */
  if (!handler->recovery_context.in_progress) {
    handler->recovery_context.attempt      = 0;
    handler->recovery_context.max_attempts = handler->max_retries;
    handler->recovery_context.in_progress  = true;
  }

  /* Increment attempt counter */
  handler->recovery_context.attempt++;

  switch (handler->recovery_strategy) {
    case k_recovery_strategy_retry:
      /* Simple retry with no reset */
      ret = ESP_OK;
      break;

    case k_recovery_strategy_reset:
      /* Call component-specific reset function */
      if (handler->reset_func) {
        ret = handler->reset_func(handler->context);
        if (ret == ESP_OK) {
          handler->recovery_count++;
          handler->in_error_state               = false;
          handler->retry_count                  = 0;
          handler->retry_interval               = handler->initial_interval;
          handler->recovery_context.in_progress = false;
        } else {
          /* Reset failed, escalate to more severe strategy if critical */
          if (handler->last_error.severity >= k_error_severity_high) {
            log_error(handler->tag, 
                      "Recovery Error", 
                      "Reset failed with error 0x%lx, escalating to shutdown strategy", 
                      (uint32_t)ret);
            
            /* Escalate to shutdown strategy */
            handler->recovery_strategy = k_recovery_strategy_shutdown;
            
            /* Try the new strategy immediately */
            return priv_execute_recovery_strategy(handler);
          }
        }
      } else {
        ret = ESP_ERR_NOT_SUPPORTED;
      }
      break;

    case k_recovery_strategy_alternate:
      /* Implement basic alternate strategy */
      log_info(handler->tag, 
               "Recovery Strategy", 
               "Attempting alternate path recovery");
      
      /* If custom recovery function exists, use it */
      if (handler->recovery_func) {
        ret = handler->recovery_func(&handler->recovery_context);
      } else {
        /* Basic implementation: just try to reset */
        if (handler->reset_func) {
          ret = handler->reset_func(handler->context);
        } else {
          ret = ESP_ERR_NOT_SUPPORTED;
        }
      }
      break;

    case k_recovery_strategy_degrade:
      /* Implement basic degraded mode */
      log_info(handler->tag, 
               "Recovery Strategy", 
               "Entering degraded operation mode");
      
      /* Mark as recovered but in degraded mode */
      handler->recovery_context.in_degraded_mode = true;
      handler->in_error_state                    = false;
      handler->recovery_count++;
      ret = ESP_OK;
      break;

    case k_recovery_strategy_shutdown:
      /* Implement safe shutdown */
      log_warn(handler->tag, 
               "Recovery Strategy", 
               "Performing safe component shutdown");
      
      /* If custom shutdown function exists, use it */
      if (handler->shutdown_func) {
        ret = handler->shutdown_func(handler->context);
      } else {
        /* Mark as permanently failed */
        handler->permanently_failed = true;
        handler->in_error_state     = true;
        ret = ESP_OK; /* Shutdown itself succeeded */
      }
      break;

    case k_recovery_strategy_custom:
      /* Call custom recovery function */
      if (handler->recovery_func) {
        ret = handler->recovery_func(&handler->recovery_context);
        if (ret == ESP_OK) {
          handler->recovery_count++;
          handler->recovery_context.in_progress = false;
          handler->in_error_state               = false;
          handler->retry_count                  = 0;
          handler->retry_interval               = handler->initial_interval;
        }
      } else {
        ret = ESP_ERR_NOT_SUPPORTED;
      }
      break;

    default:
      ret = ESP_ERR_NOT_SUPPORTED;
      break;
  }

  /* Check if we've reached max attempts */
  if (handler->recovery_context.attempt >= handler->recovery_context.max_attempts) {
    log_error(handler->tag, 
              "Recovery Error", 
              "Maximum recovery attempts (%lu) exceeded", 
              (uint32_t)handler->recovery_context.max_attempts);
    
    /* Mark as permanently failed */
    handler->permanently_failed           = true;
    handler->recovery_context.in_progress = false;
    ret                                   = ESP_ERR_INVALID_STATE;
    
    /* Call failure callback if registered */
    if (handler->permanent_failure_callback) {
      handler->permanent_failure_callback(&handler->last_error, handler->callback_context);
    }
  }

  return ret;
}

static uint64_t priv_get_timestamp(void)
{
  /* Get current time in milliseconds */
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/**
 * @brief Safely computes time difference accounting for wraparound
 * 
 * @param[in] current_ticks Current tick count
 * @param[in] previous_ticks Previous tick count
 * @return Time difference in ticks, handling wraparound
 */
static TickType_t priv_safe_tick_diff(TickType_t current_ticks, TickType_t previous_ticks)
{
  /* Handle potential wraparound of tick counter */
  if (current_ticks >= previous_ticks) {
    return current_ticks - previous_ticks;
  } else {
    /* Wraparound occurred */
    return (portMAX_DELAY - previous_ticks) + current_ticks + 1;
  }
}

/**
 * @brief Cleans up resources used by the error handler
 * 
 * @param[in,out] handler Pointer to the error_handler_t structure
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_cleanup(error_handler_t* const handler)
{
  if (!handler) {
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Cleanup Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  
  /* Free correlation data */
  if (handler->correlations) {
    for (uint32_t i = 0; i < handler->correlation_count; i++) {
      if (handler->correlations[i].related_errors) {
        free(handler->correlations[i].related_errors);
      }
    }
    free(handler->correlations);
    handler->correlations      = NULL;
    handler->correlation_count = 0;
  }
  
  /* Free root cause data */
  if (handler->root_causes) {
    for (uint32_t i = 0; i < handler->root_cause_count; i++) {
      if (handler->root_causes[i].dependent_errors) {
        free(handler->root_causes[i].dependent_errors);
      }
    }
    free(handler->root_causes);
    handler->root_causes      = NULL;
    handler->root_cause_count = 0;
  }
  
  /* Release mutex before deleting it */
  if (handler->mutex) {
    xSemaphoreGive(handler->mutex);
    
    /* Use common cleanup function to delete the mutex */
    SemaphoreHandle_t temp_mutex = handler->mutex;
    handler->mutex = NULL;
    common_cleanup_mutex(&temp_mutex, handler->tag);
  }
  
  log_info(handler->tag, "Cleanup Complete", "Error handler resources freed");
  return ESP_OK;
}

/**
 * @brief Purges old correlations to free memory
 * 
 * @param[in,out] handler Pointer to the error_handler_t structure
 * @param[in]     age_ms  Maximum age in milliseconds to keep
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_purge_old_correlations(error_handler_t* const handler, uint64_t age_ms)
{
  if (!handler) {
    return ESP_ERR_INVALID_ARG;
  }
  
  if (!handler->correlations || handler->correlation_count == 0) {
    return ESP_OK;  /* Nothing to purge */
  }
  
  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Purge Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  
  uint64_t now          = priv_get_timestamp();
  uint32_t purged_count = 0;
  
  for (uint32_t i = 0; i < handler->correlation_count; i++) {
    if ((now - handler->correlations[i].last_occurrence) > age_ms) {
      /* Free related errors array */
      if (handler->correlations[i].related_errors) {
        free(handler->correlations[i].related_errors);
      }
      
      /* Shift remaining correlations down */
      if (i < handler->correlation_count - 1) {
        memmove(&handler->correlations[i], 
                &handler->correlations[i + 1], 
                (handler->correlation_count - i - 1) * sizeof(error_correlation_t));
      }
      
      handler->correlation_count--;
      i--;  /* Recheck this index since we moved an element here */
      purged_count++;
    }
  }
  
  /* Release mutex */
  if (handler->mutex) {
    xSemaphoreGive(handler->mutex);
  }
  
  if (purged_count > 0) {
    log_info(handler->tag, 
             "Purge Complete", 
             "Purged %lu old correlations", 
             (uint32_t)purged_count);
  }
  
  return ESP_OK;
}

/**
 * @brief Sets a callback for permanent failure notification
 * 
 * @param[in,out] handler  Pointer to the error_handler_t structure
 * @param[in]     callback Callback function for permanent failures
 * @param[in]     context  Context for callback function
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_set_failure_callback(error_handler_t* const handler,
                                            error_handler_func_t   callback,
                                            void*                  context)
{
  if (!handler) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Set Failure Callback Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  handler->permanent_failure_callback = callback;
  handler->callback_context           = context;

  log_info(handler->tag, 
           "Failure Callback Set", 
           "Permanent failure callback %s", 
           callback ? "registered" : "cleared");

  /* Release mutex */
  if (handler->mutex) {
    xSemaphoreGive(handler->mutex);
  }

  return ESP_OK;
}

/**
 * @brief Cleans up the error handling system
 * 
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_system_cleanup(void)
{
  if (!s_system_initialized) {
    return ESP_OK;  /* Nothing to clean up */
  }

  /* Take mutex for thread safety */
  if (xSemaphoreTake(s_error_handler_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(error_handler_tag, "System Cleanup Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  /* Clean up all registered components */
  for (uint32_t i = 0; i < s_component_count; i++) {
    if (s_registered_components[i].handler) {
      /* Clean up this component's error handler */
      error_handler_cleanup(s_registered_components[i].handler);
    }
  }

  /* Reset component registry */
  memset(s_registered_components, 0, sizeof(s_registered_components));
  s_component_count = 0;

  /* Release and delete mutex */
  xSemaphoreGive(s_error_handler_mutex);
  vSemaphoreDelete(s_error_handler_mutex);
  s_error_handler_mutex = NULL;

  s_system_initialized = false;

  log_info(error_handler_tag, 
           "System Cleanup Complete", 
           "Error handling system cleaned up");

  return ESP_OK;
}

/**
 * @brief Checks if a component has permanently failed
 * 
 * @param[in] handler Pointer to the error_handler_t structure
 * @return true if permanently failed, false otherwise
 */
bool error_handler_is_permanently_failed(const error_handler_t* const handler)
{
  if (!handler) {
    return false;
  }

  /* Take mutex for thread safety */
  if (handler->mutex && xSemaphoreTake(handler->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(handler->tag, "Check Failed Error", "Failed to take mutex");
    return false;
  }

  bool failed = handler->permanently_failed;

  /* Release mutex */
  if (handler->mutex) {
    xSemaphoreGive(handler->mutex);
  }

  return failed;
}

/**
 * @brief Unregisters a component from the error handling system
 * 
 * @param[in] component_id Component ID to unregister
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_unregister_component(const char* component_id)
{
  if (!s_system_initialized) {
    log_error(error_handler_tag, "Unregister Error", "Error handling system not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!component_id) {
    log_error(error_handler_tag, "Unregister Error", "Invalid component ID");
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread safety */
  if (xSemaphoreTake(s_error_handler_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(error_handler_tag, "Unregister Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }

  /* Find the component */
  bool found = false;
  for (uint32_t i = 0; i < s_component_count; i++) {
    if (strcmp(s_registered_components[i].component_id, component_id) == 0) {
      found = true;
      
      /* Shift remaining components down */
      if (i < s_component_count - 1) {
        memmove(&s_registered_components[i], 
                &s_registered_components[i + 1], 
                (s_component_count - i - 1) * sizeof(component_info_t));
      }
      
      s_component_count--;
      
      log_info(error_handler_tag, 
               "Unregister Complete", 
               "Component '%s' unregistered successfully (remaining: %lu)", 
               component_id, 
               (uint32_t)s_component_count);
      
      break;
    }
  }

  /* Release mutex */
  xSemaphoreGive(s_error_handler_mutex);

  if (!found) {
    log_warn(error_handler_tag, 
             "Unregister Warning", 
             "Component '%s' not found", 
             component_id);
    return ESP_ERR_NOT_FOUND;
  }

  return ESP_OK;
}
