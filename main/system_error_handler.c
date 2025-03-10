/* main/system_error_handler.c */

#include "system_error_handler.h"
#include "log_handler.h"
#include "esp_system.h"
#include "system_tasks.h"

/* Constants ******************************************************************/

const char* const system_error_tag = "System Error Handler";

/* Globals ********************************************************************/

error_handler_t g_system_error_handler = { 0 };

/* Private Function Prototypes ************************************************/

static void priv_restart_system(void);

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
    .component_name = "System",
    .component_version = "1.0.0",
    .handler = &g_system_error_handler,
    .parent_id = NULL  /* No parent for the system component */
  };
  
  esp_err_t ret = error_handler_register_component(&system_component);
  if (ret != ESP_OK) {
    log_error(system_error_tag,
              "Registration Error",
              "Failed to register system component: %s",
              esp_err_to_name(ret));
    return ret;
  }
  
  /* Set up error callback */
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
  
  /* Set up failure callback */
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
  
  /* Set up recovery strategy */
  ret = error_handler_set_recovery(&g_system_error_handler,
                                  RECOVERY_STRATEGY_CUSTOM,
                                  system_recovery_function,
                                  NULL);
  if (ret != ESP_OK) {
    log_error(system_error_tag,
              "Recovery Error",
              "Failed to set recovery strategy: %s",
              esp_err_to_name(ret));
    return ret;
  }
  
  /* Set filtering to only handle critical and fatal errors at the system level */
  ret = error_handler_set_filtering(&g_system_error_handler,
                                   ERROR_SEVERITY_CRITICAL,
                                   ERROR_CATEGORY_ALL);
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
    log_error(system_error_tag, "Callback Error", "Error info is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Log the error based on severity */
  switch (error_info->severity) {
    case ERROR_SEVERITY_FATAL:
      log_error(system_error_tag,
                "Fatal Error",
                "Component: %s, Code: 0x%x, Description: %s",
                error_info->component_name,
                error_info->code,
                error_info->description);
      break;
      
    case ERROR_SEVERITY_CRITICAL:
      log_error(system_error_tag,
                "Critical Error",
                "Component: %s, Code: 0x%x, Description: %s",
                error_info->component_name,
                error_info->code,
                error_info->description);
      break;
      
    default:
      log_warn(system_error_tag,
               "Error",
               "Component: %s, Code: 0x%x, Description: %s",
               error_info->component_name,
               error_info->code,
               error_info->description);
      break;
  }
  
  /* For fatal errors, restart the system immediately */
  if (error_info->severity == ERROR_SEVERITY_FATAL) {
    log_error(system_error_tag,
              "System Restart",
              "Fatal error detected, restarting system in 3 seconds");
    vTaskDelay(pdMS_TO_TICKS(3000));
    priv_restart_system();
    return ESP_OK;  /* This line will never be reached */
  }
  
  return ESP_OK;
}

esp_err_t system_failure_callback(error_info_t* error_info, void* context)
{
  if (error_info == NULL) {
    log_error(system_error_tag, "Failure Callback Error", "Error info is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  log_error(system_error_tag,
            "Permanent Failure",
            "Component %s has permanently failed: %s",
            error_info->component_name,
            error_info->description);
  
  /* For critical components, restart the system */
  if (strcmp(error_info->component_name, "Motor Controller") == 0 ||
      strcmp(error_info->component_name, "Sensor System") == 0) {
    log_error(system_error_tag,
              "Critical Failure",
              "Critical component failure detected, restarting system in 5 seconds");
    vTaskDelay(pdMS_TO_TICKS(5000));
    priv_restart_system();
    return ESP_OK;  /* This line will never be reached */
  }
  
  return ESP_OK;
}

esp_err_t system_recovery_function(error_handler_t* handler, void* context)
{
  log_info(system_error_tag, "Recovery Start", "Attempting system recovery");
  
  /* Try to stop and restart system tasks */
  esp_err_t ret = system_tasks_stop();
  if (ret != ESP_OK) {
    log_error(system_error_tag,
              "Recovery Error",
              "Failed to stop system tasks: %s",
              esp_err_to_name(ret));
    return ret;
  }
  
  /* Short delay before restarting */
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  /* Reinitialize system tasks */
  ret = system_tasks_init();
  if (ret != ESP_OK) {
    log_error(system_error_tag,
              "Recovery Error",
              "Failed to reinitialize system tasks: %s",
              esp_err_to_name(ret));
    return ret;
  }
  
  /* Restart system tasks */
  ret = system_tasks_start();
  if (ret != ESP_OK) {
    log_error(system_error_tag,
              "Recovery Error",
              "Failed to restart system tasks: %s",
              esp_err_to_name(ret));
    return ret;
  }
  
  log_info(system_error_tag, "Recovery Complete", "System recovery successful");
  return ESP_OK;
}

esp_err_t system_error_handler_cleanup(void)
{
  log_info(system_error_tag, "Cleanup Start", "Cleaning up system error handler");
  
  /* Unregister the system component */
  esp_err_t ret = error_handler_unregister_component("SYSTEM");
  if (ret != ESP_OK) {
    log_warn(system_error_tag,
             "Unregister Warning",
             "Failed to unregister system component: %s",
             esp_err_to_name(ret));
  }
  
  /* Clean up the error handler */
  ret = error_handler_cleanup(&g_system_error_handler);
  if (ret != ESP_OK) {
    log_warn(system_error_tag,
             "Cleanup Warning",
             "Failed to clean up system error handler: %s",
             esp_err_to_name(ret));
  }
  
  log_info(system_error_tag, "Cleanup Complete", "System error handler cleaned up");
  return ESP_OK;
}

/* Private Functions **********************************************************/

static void priv_restart_system(void)
{
  log_error(system_error_tag, "System Restart", "Restarting system now");
  
  /* Attempt to stop tasks gracefully before restart */
  system_tasks_stop();
  
  /* Delay to allow logs to be written */
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  /* Restart the ESP32 */
  esp_restart();
} 