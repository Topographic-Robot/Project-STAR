/* main/main.c */

/* TODO: Add so the SD Card can work, with the time and the filesystem.
 *       Then add in each file for data collection writing/adding to the queue
 *       to write to files on the SD card */

#include "system_tasks.h"
#include "esp_log.h"

void app_main(void)
{
  /* Initialize System-Level Tasks (motor, sensors, webserver, etc) */
  if (system_tasks_init() != ESP_OK) {
    ESP_LOGE(system_tag, "System tasks initialization failed.");
  } else {
    ESP_LOGI(system_tag, "System tasks initialized successfully.");
    /* One sensor might have failed, but others might still be good,
     * dont exit here */
  }

  /* Start System-Level Tasks (motor, sensors, webserver, etc) */
  if (system_tasks_start() != ESP_OK) {
    ESP_LOGE(system_tag, "Failed to start system tasks. Exiting.");
    return; /* Exit app_main if tasks cannot start */
  } else {
    ESP_LOGI(system_tag, "System tasks started successfully.");
  }

  ESP_LOGI(system_tag, "Topographic Robot system initialized and running.");
}

