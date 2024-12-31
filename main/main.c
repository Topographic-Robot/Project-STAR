/* main/main.c */

/* TODO:
 * 1. mpu6050 + int
 * 2. ov7670 configs
 * 3. gps
 * 4. abstract out the reset_on_errors and other functions like that into some 
 *    common with void ptrs / func pointers. then implement all sensors to use this
 *    ALSO: I just added for DHT22 to be able to fail X times before resetting
 *    make all other sensors do this too. This is good because of timing issues
 *    where we might not need to actually reset the sensor we just need to try 
 *    again. im not sure if this will actually help like preformance or not, but
 *    id think it would since we dont need to reinitalize it again.
 */

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

