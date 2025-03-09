/* main/main.c */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pca9685_hal.h"
#include "ec11_hal.h"
#include "system_tasks.h"
#include "log_handler.h"

void app_main(void)
{
  log_info(system_tag,
          "Starting initialization",
          "Starting Project-Star initialization sequence");

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
