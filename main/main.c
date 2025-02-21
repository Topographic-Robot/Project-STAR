/* main/main.c */

/* TODO:
 * 1. mpu6050 + int
 * 2. ov7670 configs
 * 3. gps
 * 
 * 4. Error Handler
 * - updated bh1750 and ccs811 to use error handler
 * - need to update dht22, gy_neo6mv2, mpu6050, qmc5883l
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "pca9685_hal.h"
#include "ec11_hal.h"
#include "system_tasks.h"

void app_main(void)
{
  ESP_LOGI(system_tag, "Starting Topographic Robot initialization...");

  /* Initialize System-Level Tasks (motor, sensors, webserver, etc) */
  if (system_tasks_init() != ESP_OK) {
    ESP_LOGE(system_tag, "- System initialization failed - one or more components failed to initialize");
  } else {
    ESP_LOGI(system_tag, "- System initialization successful - all components ready");
  }

  /* Start System-Level Tasks (motor, sensors, webserver, etc) */
  if (system_tasks_start() != ESP_OK) {
    ESP_LOGE(system_tag, "- System startup failed - critical tasks could not be started");
    ESP_LOGE(system_tag, "- System halted - manual intervention required");
    return; /* Exit app_main if tasks cannot start */
  } else {
    ESP_LOGI(system_tag, "- System startup successful - all tasks running");
  }

  ESP_LOGI(system_tag, "Topographic Robot is now operational - monitoring system state");
}

