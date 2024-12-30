/* main/main.c */

/* TODO:
 * 1. mpu6050 + int
 * 2. ov7670 configs
 * 3. sd card + filewrite manager
 * 4. gps
 */

#include "system_tasks.h"
#include "esp_log.h"

/* TODO: TESTING SD CARDs + update filewrite manager if needed */
#include "sd_card_hal.h"

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

  /* TODO: TEST SD CARDs */
  sd_card_init();
}

