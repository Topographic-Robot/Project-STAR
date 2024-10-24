#include "system_tasks.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>

/* Constants ******************************************************************/

const char *system_tag = "Topographic-Robot";

/* Globals (Static) ***********************************************************/

static sensor_data_t     s_sensor_data;
static controller_data_t s_controller_data;

/* Private (Static) Functions *************************************************/

/**
 * @brief Clears and initializes the ESP32's Non-Volatile Storage (NVS) flash.
 * 
 * This function attempts to initialize the NVS flash. If no free pages are found 
 * or a new version of NVS is detected, it erases the flash and reinitializes it.
 * 
 * @note This function will log an error and terminate the program if NVS 
 * initialization fails.
 */
static void priv_clear_nvs_flash(void) 
{
  /* Attempt to initialize NVS (Non-Volatile Storage) */
  esp_err_t ret = nvs_flash_init();

  /* If NVS flash has no free pages or a new version is found, erase and reinitialize */
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    /* Erase NVS flash if initialization failed due to no free pages or version mismatch */
    ESP_ERROR_CHECK(nvs_flash_erase());
    /* Reinitialize NVS flash after erasing */
    ret = nvs_flash_init();
  }

  /* Ensure NVS initialization was successful; terminate if an error occurs */
  ESP_ERROR_CHECK(ret);

  /* Log the successful initialization of NVS flash */
  ESP_LOGI(system_tag, "ESP32 NVS flash cleared and initialized");
}

/* Public Functions ***********************************************************/

void system_tasks_init(void)
{
  /* Initialize the system, clear the nvs flash */
  priv_clear_nvs_flash();

  /* Initialize the sensors */
  sensors_comm_init(&s_sensor_data);

  /* Initialize the motors (controllers) */
  motor_comm_init(&s_controller_data);
}

void system_tasks_start(void)
{
  /* 1. Start Wi-Fi handling task pinned to Core 0 */
  xTaskCreatePinnedToCore(wifi_tasks, "wifi_tasks", 4096, NULL, 5, NULL, 0);

  /* 2. Start motor monitoring task pinned to Core 1 */
  xTaskCreatePinnedToCore(motor_tasks, "motor_tasks", 2048, NULL, 5, NULL, 1);

  /* 3. Start sensor data collection task pinned to Core 1 */
  xTaskCreatePinnedToCore(sensor_tasks, "sensor_tasks", 2048, 
      (void *)(&s_sensor_data), 5, NULL, 1);

  /* 4. Start webserver video relay task pinned to Core 1 */
  xTaskCreatePinnedToCore(webserver_tasks, "webserver_tasks", 2048, NULL, 5, NULL, 1);

  vTaskStartScheduler(); /* Start the Task Scheduler */

  ESP_LOGI(system_tag, "System tasks started");
}
