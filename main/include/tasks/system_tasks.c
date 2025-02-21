/* main/include/tasks/system_tasks.c */

#include "system_tasks.h"
#include "esp_err.h"
#include "esp_log.h"
#include "file_write_manager.h"
#include "ov7670_hal.h"
#include "time_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

/* Constants ******************************************************************/

const char *system_tag = "Topographic-Robot";

/* Globals ********************************************************************/

sensor_data_t    g_sensor_data    = {};
pca9685_board_t *g_pwm_controller = {};
ov7670_data_t    g_camera_data    = {}; /* TODO: Make this support all 6 cameras */

/* Private (Static) Functions *************************************************/

/**
 * @brief Initializes and resets the ESP32's Non-Volatile Storage (NVS) flash if needed.
 *
 * Initializes the NVS flash. If no free pages are available or a version mismatch is detected, 
 * the flash is erased and reinitialized.
 *
 * @return 
 * - `ESP_OK` on successful initialization.
 * - Relevant error code if the operation fails.
 */
static esp_err_t priv_clear_nvs_flash(void)
{
  ESP_LOGI(system_tag, "- NVS initialization - checking flash memory state");
  esp_err_t ret = nvs_flash_init();

  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(system_tag, "- NVS flash requires erase - performing cleanup");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  if (ret != ESP_OK) {
    ESP_LOGE(system_tag, "- NVS initialization failed - flash memory error: %s", esp_err_to_name(ret));
  } else {
    ESP_LOGI(system_tag, "- NVS initialization complete - flash memory ready");
  }

  return ret;
}

/* Public Functions ***********************************************************/

esp_err_t system_tasks_init(void)
{
  esp_err_t ret = ESP_OK;

  /* Initialize NVS storage */
  if (priv_clear_nvs_flash() != ESP_OK) {
    ESP_LOGE(system_tag, "- NVS initialization failed - storage system unavailable");
    ret = ESP_FAIL;
  }

  /* Initialize sensor communication */
  ESP_LOGI(system_tag, "- Starting sensor subsystem initialization");
  if (sensors_init(&g_sensor_data) != ESP_OK) {
    ESP_LOGE(system_tag, "- Sensor initialization failed - communication errors detected");
    ret = ESP_FAIL;
  }

  /* Initialize cameras */
  ESP_LOGI(system_tag, "- Starting camera subsystem initialization");
  if (ov7670_init(&g_camera_data) != ESP_OK) {
    ESP_LOGE(system_tag, "- Camera initialization failed - hardware communication error");
    ret = ESP_FAIL;
  }
  
  /* Initialize motor controllers */
  ESP_LOGI(system_tag, "- Starting motor controller initialization");
  if (motors_init(&g_pwm_controller) != ESP_OK) {
    ESP_LOGE(system_tag, "- Motor controller initialization failed - PWM system error");
    ret = ESP_FAIL;
  }

  /* Initialize gait array (Must be done after initializing motor controllers */
  ESP_LOGI(system_tag, "- Starting gait system initialization");
  if (gait_init(g_pwm_controller) != ESP_OK) {
    ESP_LOGE(system_tag, "- Gait initialization failed - motor mapping error");
    ret = ESP_FAIL;
  }
  
  /* Initialize storage (e.g., SD card or SPIFFS) */
  ESP_LOGI(system_tag, "- Starting storage system initialization");
  if (file_write_manager_init() != ESP_OK) {
    ESP_LOGE(system_tag, "- Storage initialization failed - file system error");
    ret = ESP_FAIL;
  }

  if (ret == ESP_OK) {
    ESP_LOGI(system_tag, "- System initialization complete - all components ready");
  } else {
    ESP_LOGW(system_tag, "- System initialization incomplete - some components failed");
  }
  return ret;
}

esp_err_t system_tasks_start(void)
{
  esp_err_t ret = ESP_OK;

  /* Start WiFi task */
  ESP_LOGI(system_tag, "- Starting WiFi subsystem");
  if (wifi_task_start() != ESP_OK) {
    ESP_LOGE(system_tag, "- WiFi task failed to start - network functionality limited");
    ret = ESP_FAIL;
  }

  ///* Start camera monitoring task */
  //ESP_LOGI(system_tag, "- Starting camera monitoring system");
  //if (ov7670_task_start(&g_camera_data) != ESP_OK) {
  //  ESP_LOGE(system_tag, "- Camera monitoring failed to start - vision system offline");
  //  ret = ESP_FAIL;
  //} /* TODO: Add this back in/make this work */

  /* Start sensor tasks */
  ESP_LOGI(system_tag, "- Starting sensor monitoring system");
  if (sensor_tasks(&g_sensor_data) != ESP_OK) {
    ESP_LOGE(system_tag, "- Sensor tasks failed to start - environmental monitoring limited");
    ret = ESP_FAIL;
  }

  /* Start motor control tasks */
  ESP_LOGI(system_tag, "- Starting motor control system");
  if (motor_tasks_start(g_pwm_controller) != ESP_OK) {
    ESP_LOGE(system_tag, "- Motor tasks failed to start - movement system offline");
    ret = ESP_FAIL;
  }

  if (ret == ESP_OK) {
    ESP_LOGI(system_tag, "- All system tasks started successfully - robot fully operational");
  } else {
    ESP_LOGW(system_tag, "- System tasks partially started - some functionality limited");
  }
  return ret;
}

