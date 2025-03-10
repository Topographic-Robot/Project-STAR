/* main/system_error_handler.c */

#include "system_error_handler.h"
#include "log_handler.h"
#include "esp_system.h"
#include "system_tasks.h"
#include <string.h>

/* Constants ******************************************************************/

const char* const system_error_tag = "System Error Handler";

/* Globals ********************************************************************/

error_handler_t g_system_error_handler = { 0 };

/* Private Function Prototypes ************************************************/

static esp_err_t priv_restart_system(void* context);
static esp_err_t priv_recovery_wrapper(recovery_context_t* context);

/* Public Functions ***********************************************************/

esp_err_t system_error_handler_init(void)
{
  log_info(system_error_tag, "Init Start", "Initializing system error handler");
  
  /* Initialize the system error handler */
  error_handler_init(&g_system_error_handler,
                     system_error_tag,
                     3,                  /* Max retries */
                     1000,               /* Initial interval (ms) */
                     10000,              /* Max interval (ms) */
                     priv_restart_system,/* Reset function */
                     NULL,               /* Context */
                     1000,               /* Initial backoff (ms) */
                     30000);             /* Max backoff (ms) */
  
  /* Register the system component */
  component_info_t system_component = {
    .component_id = "SYSTEM",
    .handler = &g_system_error_handler,
    .parent_id = NULL,  /* No parent for the system component */
    .priority = 0       /* Highest priority */
  };
  
  esp_err_t ret = error_handler_register_component(&system_component);
  if (ret != ESP_OK) {
    log_error(system_error_tag,
              "Registration Error",
              "Failed to register system component: %s",
              esp_err_to_name(ret));
    return ret;
  }
  
  /* Set up error callbacks */
  ret = error_handler_set_callback(&g_system_error_handler,
                                  system_error_callback,
                                  NULL);
  if (ret != ESP_OK) {
    log_error(system_error_tag,
              "Callback Error",
              "Failed to set error callback: %s",
              esp_err_to_name(ret));
    return ret;
  }
  
  /* Set up permanent failure callback */
  ret = error_handler_set_failure_callback(&g_system_error_handler,
                                         system_failure_callback,
                                         NULL);
  if (ret != ESP_OK) {
    log_error(system_error_tag,
              "Callback Error",
              "Failed to set failure callback: %s",
              esp_err_to_name(ret));
    return ret;
  }
  
  /* Set up custom recovery strategy */
  ret = error_handler_set_recovery(&g_system_error_handler,
                                  k_recovery_strategy_custom,
                                  priv_recovery_wrapper,
                                  NULL);
  if (ret != ESP_OK) {
    log_error(system_error_tag,
              "Recovery Error",
              "Failed to set recovery strategy: %s",
              esp_err_to_name(ret));
    return ret;
  }
  
  /* Set up error filtering to handle all categories */
  ret = error_handler_set_filtering(&g_system_error_handler,
                                   k_error_severity_low,  /* Handle all severities */
                                   0);  /* Handle all categories */
  if (ret != ESP_OK) {
    log_error(system_error_tag,
              "Filtering Error",
              "Failed to set error filtering: %s",
              esp_err_to_name(ret));
    return ret;
  }
  
  log_info(system_error_tag, "Init Complete", "System error handler initialized successfully");
  return ESP_OK;
}

esp_err_t system_error_callback(error_info_t* error_info, void* context)
{
  if (error_info == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  
  switch (error_info->severity) {
    case k_error_severity_low:
      log_warn(system_error_tag,
               "Low Severity Error",
               "Component %s reported error: %s",
               error_info->component_id,
               error_info->description);
      break;
      
    case k_error_severity_medium:
      log_warn(system_error_tag,
               "Medium Severity Error",
               "Component %s reported error: %s",
               error_info->component_id,
               error_info->description);
      break;
      
    case k_error_severity_high:
    case k_error_severity_critical:
    case k_error_severity_fatal:
      log_error(system_error_tag,
                "High Severity Error",
                "Component %s reported critical error: %s",
                error_info->component_id,
                error_info->description);
      break;
      
    default:
      break;
  }
  
  return ESP_OK;
}

esp_err_t system_failure_callback(error_info_t* error_info, void* context)
{
  if (error_info == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  
  log_error(system_error_tag,
            "Permanent Failure",
            "Component %s has permanently failed: %s",
            error_info->component_id,
            error_info->description);
  
  if (strcmp(error_info->component_id, "Motor Controller") == 0 ||
      strcmp(error_info->component_id, "Sensor System") == 0) {
    /* Critical components failed, initiate system shutdown */
    log_error(system_error_tag,
              "Critical Failure",
              "Critical component failure detected, initiating system shutdown");
    system_tasks_stop();
    return ESP_OK;
  }
  
  return ESP_OK;
}

esp_err_t system_recovery_function(error_handler_t* handler, void* context)
{
  if (handler == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  
  log_info(system_error_tag,
           "Recovery Attempt",
           "Attempting system recovery");
  
  /* Implement custom recovery logic here */
  /* For example, restart specific tasks or services */
  
  return ESP_OK;
}

esp_err_t system_error_handler_cleanup(void)
{
  log_info(system_error_tag, "Cleanup Start", "Cleaning up system error handler");
  
  /* Unregister the system component */
  esp_err_t ret = error_handler_unregister_component("SYSTEM");
  if (ret != ESP_OK) {
    log_error(system_error_tag,
              "Unregister Error",
              "Failed to unregister system component: %s",
              esp_err_to_name(ret));
  }
  
  /* Clean up the error handler */
  ret = error_handler_cleanup(&g_system_error_handler);
  if (ret != ESP_OK) {
    log_error(system_error_tag,
              "Cleanup Error",
              "Failed to clean up system error handler: %s",
              esp_err_to_name(ret));
    return ret;
  }
  
  log_info(system_error_tag, "Cleanup Complete", "System error handler cleaned up successfully");
  return ESP_OK;
}

static esp_err_t priv_restart_system(void* context)
{
  log_error(system_error_tag, "System Reset", "Restarting system due to unrecoverable error");
  vTaskDelay(pdMS_TO_TICKS(1000)); /* Give time for the log message to be sent */
  esp_restart();
  return ESP_OK; /* This line will never be reached */
}

/**
 * @brief Wrapper function to adapt system_recovery_function to recovery_func_t signature
 * 
 * @param context Recovery context
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
static esp_err_t priv_recovery_wrapper(recovery_context_t* context)
{
  if (context == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  
  log_info(system_error_tag,
           "Recovery Attempt",
           "Attempting system recovery (attempt %lu of %lu)",
           context->attempt,
           context->max_attempts);
  
  esp_err_t ret = system_recovery_function(&g_system_error_handler, NULL);
  
  if (ret != ESP_OK || context->attempt >= context->max_attempts) {
    log_error(system_error_tag,
              "Recovery Failed",
              "Maximum recovery attempts reached or recovery function failed");
    return ESP_FAIL;
  }
  
  return ESP_OK;
} 