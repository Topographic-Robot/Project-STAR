/* main/main.c */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pca9685_hal.h"
#include "ec11_hal.h"
#include "system_tasks.h"
#include "log_handler.h"
#include "esp_system.h"
#include "error_handler.h"
#include "system_error_handler.h"
#include "system_setup.h"

/**
 * @brief System shutdown handler
 * 
 * This function is called automatically when the system is shutting down.
 * It ensures that all system tasks are properly stopped and resources are cleaned up.
 */
void shutdown_handler(void) /* NOTE: In the future we need to call this when the battery of the robot is low */
{
  log_info(system_tag, 
           "System Shutdown", 
           "Shutdown handler triggered, cleaning up system resources");
  
  /* Call system_tasks_stop to perform a graceful shutdown of all components */
  esp_err_t ret = system_tasks_stop();
  
  if (ret != ESP_OK) {
    log_warn(system_tag, 
             "Shutdown Warning", 
             "Some components failed to shut down cleanly");
  } else {
    log_info(system_tag, 
             "Shutdown Complete", 
             "All system components shut down successfully");
  }
  
  /* Clean up system hardware */
  system_setup_cleanup();
  
  /* Clean up the system error handler */
  system_error_handler_cleanup();
  
  /* Clean up the error handling system */
  error_handler_system_cleanup();
}

void app_main(void)
{
  log_info(system_tag,
          "Starting initialization",
          "Starting Project-Star initialization sequence");

  /* Initialize the error handling system */
  if (error_handler_system_init() != ESP_OK) {
    log_error(system_tag,
              "Error handler initialization failed",
              "Could not initialize error handling system");
    return; /* Exit if error handler can't be initialized */
  }
  
  /* Initialize the system error handler */
  if (system_error_handler_init() != ESP_OK) {
    log_error(system_tag,
              "System error handler initialization failed",
              "Could not initialize system error handler");
    error_handler_system_cleanup();
    return; /* Exit if system error handler can't be initialized */
  }
  
  /* Initialize system hardware */
  if (system_setup_init() != ESP_OK) {
    log_error(system_tag,
              "System hardware initialization failed",
              "Could not initialize system hardware");
    ERROR_RECORD(&g_system_error_handler, ESP_FAIL, ERROR_CATEGORY_HARDWARE, ERROR_SEVERITY_CRITICAL,
                "System hardware initialization failed");
    system_error_handler_cleanup();
    error_handler_system_cleanup();
    return; /* Exit if system hardware can't be initialized */
  }

  /* Register shutdown handler to ensure clean shutdown */
  esp_register_shutdown_handler(shutdown_handler);
  log_info(system_tag,
           "Shutdown Handler",
           "Registered system shutdown handler");

  /* Initialize System-Level Tasks (motor, sensors, webserver, etc) */
  if (system_tasks_init() != ESP_OK) {
    log_error(system_tag,
              "System initialization failed",
              "One or more components failed to initialize");
    
    /* Record the error in the system error handler */
    ERROR_RECORD(&g_system_error_handler, ESP_FAIL, ERROR_CATEGORY_SYSTEM, ERROR_SEVERITY_CRITICAL,
                "System initialization failed, one or more components failed to initialize");
  } else {
    log_info(system_tag,
             "System initialization complete",
             "All components initialized successfully");
  }

  /* Start System-Level Tasks (motor, sensors, webserver, etc) */
  if (system_tasks_start() != ESP_OK) {
    log_error(system_tag,
              "System startup failed",
              "Critical tasks could not be started manual intervention required");
    
    /* Record the error in the system error handler */
    ERROR_RECORD(&g_system_error_handler, ESP_FAIL, ERROR_CATEGORY_SYSTEM, ERROR_SEVERITY_FATAL,
                "System startup failed, critical tasks could not be started");
    return; /* Exit app_main if tasks cannot start */
  } else {
    log_info(system_tag,
             "System startup complete",
             "All tasks started and running");
  }

  log_info(system_tag,
           "System operational",
           "Project-Star is now operational and monitoring system state");
  
  /* Process errors periodically */
  while (1) {
    error_handler_process();
    vTaskDelay(pdMS_TO_TICKS(1000)); /* Process errors every second */
  }
}
