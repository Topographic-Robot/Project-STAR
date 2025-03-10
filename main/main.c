/* main/main.c */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pca9685_hal.h"
#include "ec11_hal.h"
#include "system_tasks.h"
#include "log_handler.h"
#include "esp_system.h"

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
}

void app_main(void)
{
  log_info(system_tag,
          "Starting initialization",
          "Starting Project-Star initialization sequence");

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
    return; /* Exit app_main if tasks cannot start */
  } else {
    log_info(system_tag,
             "System startup complete",
             "All tasks started and running");
  }

  log_info(system_tag,
           "System operational",
           "Project-Star is now operational and monitoring system state");
}
