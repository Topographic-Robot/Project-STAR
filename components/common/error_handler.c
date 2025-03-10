/* components/common/error_handler.c */

#include "error_handler.h"
#include "log_handler.h"
#include <string.h>
#include <time.h>
#include <stdlib.h>

/* Constants ******************************************************************/

const char* const error_handler_tag = "Error Handler";
#define MAX_CORRELATIONS 32  /**< Maximum number of error correlations to track */
#define MAX_ROOT_CAUSES  16  /**< Maximum number of root causes to track */
#define MAX_RELATED_ERRORS 8 /**< Maximum number of related errors per correlation */

/* Static Variables ***********************************************************/

static component_info_t s_registered_components[MAX_COMPONENTS] = {0};   /**< Registered components */
static uint32_t         s_component_count                       = 0;     /**< Number of registered components */
static bool             s_system_initialized                    = false; /**< System initialized flag */

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

/* Private Function Prototypes ************************************************/

/**
 * @brief Propagates an error to the parent component
 * 
 * @param[in] handler    Pointer to the error handler
 * @param[in] error_info Error information to propagate
 * @return ESP_OK if successful, error code otherwise
 */
static esp_err_t priv_propagate_error(const error_handler_t* handler, const error_info_t* error_info);

/**
 * @brief Executes the recovery strategy for a component
 * 
 * @param[in,out] handler Pointer to the error handler
 * @return ESP_OK if successful, error code otherwise
 */
static esp_err_t priv_execute_recovery_strategy(error_handler_t* handler);

/**
 * @brief Gets the current timestamp in milliseconds
 * 
 * @return Current timestamp in milliseconds
 */
static uint64_t priv_get_timestamp(void);

/* Public Functions ***********************************************************/

esp_err_t error_handler_system_init(void)
{
  if (s_system_initialized) {
    log_warn(error_handler_tag, "Init Warning", "Error handling system already initialized");
    return ESP_OK;
  }

  /* Initialize component registry */
  memset(s_registered_components, 0, sizeof(s_registered_components));
  s_component_count = 0;
  s_system_initialized = true;

  log_info(error_handler_tag, "Init Complete", "Error handling system initialized");
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
      handler->recovery_count++;
    }
    handler->in_error_state = false;
    handler->retry_count    = 0;
    handler->retry_interval = handler->initial_interval;
    return ESP_OK;
  }

  /* Handle error status */
  handler->in_error_state = true;
  handler->error_count++;

  /* Create basic error info for backward compatibility */
  error_info_t error_info = {
    .code = status,
    .category = k_error_category_none,
    .severity = k_error_severity_medium, /* Default to medium severity */
    .file = NULL,
    .line = 0,
    .func = NULL,
    .description = "Error status recorded",
    .timestamp = priv_get_timestamp(),
    .count = 1
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

    /* Execute recovery based on strategy */
    return priv_execute_recovery_strategy(handler);
  } else {
    log_warn(handler->tag, 
             "Retry Delay", 
             "Next retry available in %u ticks",
             (unsigned int)(handler->retry_interval - (now_ticks - handler->last_attempt_ticks)));
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

  /* Create detailed error info */
  error_info_t error_info = {
    .code = code,
    .category = category,
    .severity = severity,
    .file = file,
    .line = line,
    .func = func,
    .description = desc,
    .timestamp = priv_get_timestamp(),
    .count = 1
  };

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
      (severity == k_error_severity_high || severity == k_error_severity_critical)) {
    priv_propagate_error(handler, &error_info);
  }

  /* For critical errors, always start recovery */
  if (severity == k_error_severity_critical) {
    return error_handler_start_recovery(handler);
  }

  /* For non-critical errors, just record the status */
  handler->last_status = code;
  handler->in_error_state = (code != ESP_OK);
  handler->error_count++;

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

  /* Check if we've reached the maximum number of components */
  if (s_component_count >= MAX_COMPONENTS) {
    log_error(error_handler_tag, 
              "Register Error", 
              "Maximum number of components (%d) reached", 
              MAX_COMPONENTS);
    return ESP_ERR_NO_MEM;
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
      return ESP_OK;
    }
  }

  /* Register new component */
  s_registered_components[s_component_count] = *component_info;
  s_component_count++;

  log_info(error_handler_tag, 
           "Register Complete", 
           "Component '%s' registered successfully (total: %u)", 
           component_info->component_id, 
           (unsigned int)s_component_count);

  return ESP_OK;
}

esp_err_t error_handler_set_callback(error_handler_t* const handler,
                                     error_handler_func_t    callback,
                                     void*                   context)
{
  if (!handler) {
    return ESP_ERR_INVALID_ARG;
  }

  handler->error_callback = callback;
  handler->callback_context = context;

  log_info(handler->tag, 
           "Callback Set", 
           "Error callback %s", 
           callback ? "registered" : "cleared");

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

  handler->recovery_strategy = strategy;
  handler->recovery_func = func;
  handler->recovery_context.context = context;
  handler->recovery_context.strategy = strategy;
  handler->recovery_context.attempt = 0;
  handler->recovery_context.max_attempts = handler->max_retries;
  handler->recovery_context.in_progress = false;

  log_info(handler->tag, 
           "Recovery Set", 
           "Recovery strategy set to %s", 
           recovery_strategy_to_string(strategy));

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

  /* Initialize recovery context */
  handler->recovery_context.attempt = 0;
  handler->recovery_context.max_attempts = handler->max_retries;
  handler->recovery_context.timeout = xTaskGetTickCount() + pdMS_TO_TICKS(5000); /* 5 second timeout */
  handler->recovery_context.in_progress = true;

  log_info(handler->tag, 
           "Recovery Start", 
           "Starting recovery with strategy: %s", 
           recovery_strategy_to_string(handler->recovery_strategy));

  /* Execute recovery strategy */
  return priv_execute_recovery_strategy(handler);
}

esp_err_t error_handler_get_status(const error_handler_t* const handler,
                                   error_info_t*                info)
{
  if (!handler || !info) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Copy last error information */
  memcpy(info, &handler->last_error, sizeof(error_info_t));

  return ESP_OK;
}

esp_err_t error_handler_process(void)
{
  if (!s_system_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t result = ESP_OK;

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

  handler->min_severity_to_handle = min_severity;
  handler->categories_to_filter = categories;

  log_info(handler->tag, 
           "Filtering Setup", 
           "Set minimum severity to %s, filtered categories: 0x%08x", 
           error_severity_to_string(min_severity), 
           (unsigned int)categories);

  return ESP_OK;
}

esp_err_t error_handler_correlate_errors(error_handler_t* const handler,
                                        error_info_t*          error_info,
                                        error_info_t*          related_error)
{
  if (!handler || !error_info || !related_error) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if we need to allocate correlation array */
  if (!handler->correlations) {
    handler->correlations = calloc(handler->max_correlations, sizeof(error_correlation_t));
    if (!handler->correlations) {
      log_error(handler->tag, 
                "Correlation Error", 
                "Failed to allocate memory for correlations");
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
      log_error(handler->tag, 
                "Correlation Error", 
                "Maximum number of correlations reached");
      return ESP_ERR_NO_MEM;
    }

    correlation = &handler->correlations[handler->correlation_count++];
    correlation->correlation_id = correlation_id;
    correlation->related_count = 0;
    correlation->related_errors = calloc(MAX_RELATED_ERRORS, sizeof(error_info_t));
    correlation->first_occurrence = error_info->timestamp;
    correlation->last_occurrence = error_info->timestamp;
    correlation->is_root_cause = false;
    
    if (!correlation->related_errors) {
      log_error(handler->tag, 
                "Correlation Error", 
                "Failed to allocate memory for related errors");
      handler->correlation_count--;
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

  /* Add errors to correlation if not already present */
  bool error_info_added = false;
  bool related_error_added = false;
  
  for (uint32_t i = 0; i < correlation->related_count; i++) {
    if (correlation->related_errors[i].code == error_info->code &&
        correlation->related_errors[i].category == error_info->category &&
        correlation->related_errors[i].line == error_info->line &&
        strcmp(correlation->related_errors[i].file, error_info->file) == 0) {
      error_info_added = true;
    }
    
    if (correlation->related_errors[i].code == related_error->code &&
        correlation->related_errors[i].category == related_error->category &&
        correlation->related_errors[i].line == related_error->line &&
        strcmp(correlation->related_errors[i].file, related_error->file) == 0) {
      related_error_added = true;
    }
  }

  /* Add error_info if not already in correlation */
  if (!error_info_added && correlation->related_count < MAX_RELATED_ERRORS) {
    memcpy(&correlation->related_errors[correlation->related_count++], 
           error_info, 
           sizeof(error_info_t));
    
    /* Update the correlation ID in the original error */
    error_info->correlation_id = correlation_id;
  }

  /* Add related_error if not already in correlation */
  if (!related_error_added && correlation->related_count < MAX_RELATED_ERRORS) {
    memcpy(&correlation->related_errors[correlation->related_count++], 
           related_error, 
           sizeof(error_info_t));
    
    /* Update the correlation ID in the related error */
    related_error->correlation_id = correlation_id;
  }

  log_info(handler->tag, 
           "Error Correlation", 
           "Correlated errors with ID %u, total related: %u", 
           (unsigned int)correlation_id, 
           (unsigned int)correlation->related_count);

  return ESP_OK;
}

esp_err_t error_handler_get_correlation(const error_handler_t* const handler,
                                       uint32_t                    correlation_id,
                                       error_correlation_t*        correlation)
{
  if (!handler || !correlation || correlation_id == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!handler->correlations) {
    return ESP_ERR_NOT_FOUND;
  }

  /* Find correlation by ID */
  for (uint32_t i = 0; i < handler->correlation_count; i++) {
    if (handler->correlations[i].correlation_id == correlation_id) {
      /* Copy correlation data */
      memcpy(correlation, &handler->correlations[i], sizeof(error_correlation_t));
      
      /* Don't copy the pointer, as it might be freed later */
      correlation->related_errors = NULL;
      
      return ESP_OK;
    }
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

  /* Check if we need to allocate root causes array */
  if (!handler->root_causes) {
    handler->root_causes = calloc(handler->max_root_causes, sizeof(root_cause_info_t));
    if (!handler->root_causes) {
      log_error(handler->tag, 
                "Root Cause Analysis Error", 
                "Failed to allocate memory for root causes");
      return ESP_ERR_NO_MEM;
    }
  }

  /* Initialize root cause info */
  memset(root_cause, 0, sizeof(root_cause_info_t));
  
  /* Copy the current error as potential root cause */
  memcpy(&root_cause->root_error, error_info, sizeof(error_info_t));
  root_cause->dependent_count = 0;
  root_cause->dependent_errors = NULL;
  root_cause->confidence = 0.5f; /* Default confidence */

  /* Analyze based on error category and severity */
  switch (error_info->category) {
    case k_error_category_hardware:
      root_cause->diagnosis = "Hardware failure detected";
      root_cause->confidence = 0.8f;
      break;
      
    case k_error_category_communication:
      root_cause->diagnosis = "Communication failure, check connections";
      root_cause->confidence = 0.7f;
      break;
      
    case k_error_category_memory:
      root_cause->diagnosis = "Memory allocation or access issue";
      root_cause->confidence = 0.9f;
      break;
      
    case k_error_category_system:
      root_cause->diagnosis = "System-level failure";
      root_cause->confidence = 0.6f;
      break;
      
    case k_error_category_power:
      root_cause->diagnosis = "Power-related issue detected";
      root_cause->confidence = 0.85f;
      break;
      
    default:
      root_cause->diagnosis = "Unknown root cause";
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
          
          return ESP_OK;
        }
      }
    }
  }

  return ESP_ERR_NOT_FOUND;
}

/* Private Functions **********************************************************/

static esp_err_t priv_propagate_error(const error_handler_t* handler, const error_info_t* error_info)
{
  if (!s_system_initialized || !handler || !error_info) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Find the component in the registry */
  const char* component_id = NULL;
  const char* parent_id = NULL;

  for (uint32_t i = 0; i < s_component_count; i++) {
    if (s_registered_components[i].handler == handler) {
      component_id = s_registered_components[i].component_id;
      parent_id = s_registered_components[i].parent_id;
      break;
    }
  }

  if (!component_id || !parent_id) {
    /* Component not found or no parent */
    return ESP_OK;
  }

  /* Find the parent component */
  for (uint32_t i = 0; i < s_component_count; i++) {
    if (strcmp(s_registered_components[i].component_id, parent_id) == 0) {
      /* Propagate error to parent */
      error_handler_t* parent_handler = s_registered_components[i].handler;
      
      log_info(error_handler_tag, 
               "Error Propagation", 
               "Propagating error from '%s' to parent '%s'", 
               component_id, 
               parent_id);
      
      /* Record the error in the parent */
      return error_handler_record_error(parent_handler,
                                        error_info->code,
                                        error_info->category,
                                        error_info->severity,
                                        error_info->file,
                                        error_info->line,
                                        error_info->func,
                                        error_info->description);
    }
  }

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
    handler->recovery_context.attempt = 0;
    handler->recovery_context.max_attempts = handler->max_retries;
    handler->recovery_context.in_progress = true;
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
          handler->in_error_state = false;
          handler->retry_count = 0;
          handler->retry_interval = handler->initial_interval;
          handler->recovery_context.in_progress = false;
        }
      } else {
        ret = ESP_ERR_NOT_SUPPORTED;
      }
      break;

    case k_recovery_strategy_custom:
      /* Call custom recovery function */
      if (handler->recovery_func) {
        ret = handler->recovery_func(&handler->recovery_context);
        if (ret == ESP_OK) {
          handler->recovery_count++;
          handler->recovery_context.in_progress = false;
          handler->in_error_state = false;
          handler->retry_count = 0;
          handler->retry_interval = handler->initial_interval;
        }
      } else {
        ret = ESP_ERR_NOT_SUPPORTED;
      }
      break;

    case k_recovery_strategy_alternate:
    case k_recovery_strategy_degrade:
    case k_recovery_strategy_shutdown:
      /* These strategies require custom implementation */
      log_warn(handler->tag, 
               "Recovery Warning", 
               "Strategy %s not implemented, falling back to retry", 
               recovery_strategy_to_string(handler->recovery_strategy));
      ret = ESP_ERR_NOT_SUPPORTED;
      break;

    default:
      ret = ESP_ERR_NOT_SUPPORTED;
      break;
  }

  /* Check if we've reached max attempts */
  if (handler->recovery_context.attempt >= handler->recovery_context.max_attempts) {
    log_error(handler->tag, 
              "Recovery Error", 
              "Maximum recovery attempts (%u) exceeded", 
              (unsigned int)handler->recovery_context.max_attempts);
    
    handler->recovery_context.in_progress = false;
    ret = ESP_ERR_INVALID_STATE;
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
