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
 * @brief Clears and initializes the ESP32's Non-Volatile Storage (NVS) flash.
 *
 * Attempts to initialize the NVS flash. If no free pages are found
 * or a new version of NVS is detected, it erases the flash and reinitializes it.
 *
 * @return ESP_OK if successful; otherwise, returns an error code.
 */
static esp_err_t priv_clear_nvs_flash(void)
{
  esp_err_t ret = nvs_flash_init();

  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(system_tag, "Erasing NVS flash due to error: %s", esp_err_to_name(ret));
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  return ret;
}

/* Public Functions ***********************************************************/

esp_err_t system_tasks_init(void)
{
  esp_err_t ret = ESP_OK;
  /* Initialize NVS storage */
  if (priv_clear_nvs_flash() != ESP_OK) {
    ESP_LOGE(system_tag, "Failed to initialize NVS: %s", esp_err_to_name(ret));
    ret = ESP_FAIL;
  }

  /* Initialize sensor communication */
  if (sensors_init(&g_sensor_data) != ESP_OK) {
    ESP_LOGE(system_tag, "Sensor communication initialization failed.");
    ret = ESP_FAIL;
  }

  /* Initialize cameras */
  if (ov7670_init(&g_camera_data) != ESP_OK) {
    ESP_LOGE(system_tag, "Camera communication initialization failed.");
    ret = ESP_FAIL;
  }
  
  /* Initialize motor controllers */
  if (motors_init(&g_pwm_controller) != ESP_OK) {
    ESP_LOGE(system_tag, "Motor controller initialization failed.");
    ret = ESP_FAIL;
  }

  /* Initialize gait array (Must be done after initalizing motor controllers */
  if (gait_init(g_pwm_controller) != ESP_OK) {
    ESP_LOGE(system_tag, "Gait static array initialization failed.");
    ret = ESP_FAIL;
  }

  /* Initialize WiFi */
  if (wifi_init_sta() != ESP_OK) {
    ESP_LOGE(system_tag, "Wifi failed to connect / initialize.");
    ret = ESP_FAIL;
  }
  
  /* Initialize time (SNTP) */
  if (time_manager_init() != ESP_OK) {
		ESP_LOGE(system_tag ,"Time initialization failed.");
    ret = ESP_FAIL;
	}
  
  /* Initialize storage (e.g., SD card or SPIFFS) */
  if (file_write_manager_init() != ESP_OK) {
    ESP_LOGE(system_tag, "Storage initialization failed.");
    ret = ESP_FAIL;
  }

  if (ret == ESP_OK) {
    ESP_LOGI(system_tag, "All system components initialized successfully.");
  }
  return ret;
}

esp_err_t system_tasks_start(void)
{
  esp_err_t ret = ESP_OK;
  /* Start sensor tasks */
  if (sensor_tasks(&g_sensor_data) != ESP_OK) {
    ESP_LOGE(system_tag, "Sensor tasks start failed.");
    ret = ESP_FAIL;
  }

  /* Start motor control tasks */
  if (motor_tasks_start(g_pwm_controller) != ESP_OK) {
    ESP_LOGE(system_tag, "Motor tasks start failed.");
    ret = ESP_FAIL;
  }

  if (ret == ESP_OK) {
    ESP_LOGI(system_tag, "System tasks started successfully.");
  }
  return ret;
}

