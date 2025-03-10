/* main/include/tasks/system_tasks.c */

#include "system_tasks.h"
#include "log_handler.h"
#include "log_storage.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "wifi_tasks.h"
#include "sensor_tasks.h"
#include "motor_tasks.h"
#include "webserver_tasks.h"
#include "time_manager.h"
#include "file_write_manager.h"
#include "sd_card_hal.h"
#include "common/bus_manager.h"

/* Defines ********************************************************************/

#ifndef ENABLE_SD_CARD_LOGGING
#define ENABLE_SD_CARD_LOGGING true
#endif

#ifndef ENABLE_LOG_COMPRESSION
#define ENABLE_LOG_COMPRESSION true
#endif

/* Constants ******************************************************************/

const char* const system_tag = "Project Star";

/* Globals ********************************************************************/

sensor_data_t    g_sensor_data    = {};
pca9685_board_t* g_pwm_controller = {};
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
  log_info(system_tag, "NVS Start", "Beginning NVS flash memory initialization");
  esp_err_t ret = nvs_flash_init();

  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    log_warn(system_tag, 
             "NVS Clean", 
             "Flash memory requires cleanup, performing erase");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  if (ret != ESP_OK) {
    log_error(system_tag, 
              "NVS Error", 
              "Flash memory initialization failed: %s", 
              esp_err_to_name(ret));
  } else {
    log_info(system_tag, "NVS Complete", "Flash memory initialized successfully");
  }

  return ret;
}

/* Public Functions ***********************************************************/

esp_err_t system_tasks_init(void)
{
  esp_err_t ret = ESP_OK;

  /* Initialize logging system */
  log_info(system_tag, "Log Start", "Beginning log system initialization");
  if (log_init(ENABLE_SD_CARD_LOGGING) != ESP_OK) {
    ESP_LOGE(system_tag, "Failed to initialize log system");
    ret = ESP_FAIL;
  }
  
  /* Enable log compression for storage efficiency */
  log_storage_set_compression(ENABLE_LOG_COMPRESSION);
  log_info(system_tag, 
           "Log Compression", 
           "Enabled log compression for storage efficiency");

  /* Initialize NVS storage */
  if (priv_clear_nvs_flash() != ESP_OK) {
    log_error(system_tag, "NVS Error", "Storage system initialization failed");
    ret = ESP_FAIL;
  }

  /* Initialize sensor communication */
  log_info(system_tag, "Sensor Start", "Beginning sensor subsystem initialization");
  if (sensors_init(&g_sensor_data) != ESP_OK) {
    log_error(system_tag, 
              "Sensor Error", 
              "Failed to initialize sensors: communication errors detected");
    ret = ESP_FAIL;
  }

  /* Initialize cameras */
  log_info(system_tag, "Camera Start", "Beginning camera subsystem initialization");
  if (ov7670_init(&g_camera_data) != ESP_OK) {
    log_error(system_tag, 
              "Camera Error", 
              "Failed to initialize camera: hardware communication error");
    ret = ESP_FAIL;
  }
  
  /* Initialize motor controllers */
  log_info(system_tag, "Motor Start", "Beginning motor controller initialization");
  if (motors_init(&g_pwm_controller) != ESP_OK) {
    log_error(system_tag, 
              "Motor Error", 
              "Failed to initialize motor controllers: PWM system error");
    ret = ESP_FAIL;
  }

  /* Initialize gait array (Must be done after initializing motor controllers */
  log_info(system_tag, "Gait Start", "Beginning gait system initialization");
  if (gait_init(g_pwm_controller) != ESP_OK) {
    log_error(system_tag, 
              "Gait Error", 
              "Failed to initialize gait system: motor mapping error");
    ret = ESP_FAIL;
  }
  
  /* Initialize storage (e.g., SD card or SPIFFS) */
  log_info(system_tag, "Storage Start", "Beginning storage system initialization");
  if (file_write_manager_init() != ESP_OK) {
    log_error(system_tag, 
              "Storage Error", 
              "Failed to initialize storage: file system error");
    ret = ESP_FAIL;
  }

  if (ret == ESP_OK) {
    log_info(system_tag, 
             "Init Complete", 
             "All system components initialized successfully");
  } else {
    log_warn(system_tag, 
             "Init Warning", 
             "System initialization incomplete: some components failed");
  }
  return ret;
}

esp_err_t system_tasks_start(void)
{
  esp_err_t ret = ESP_OK;

  /* Start WiFi task */
  log_info(system_tag, "WiFi Start", "Beginning WiFi subsystem initialization");
  if (wifi_task_start() != ESP_OK) {
    log_error(system_tag, 
              "WiFi Error", 
              "Failed to start WiFi task: network functionality limited");
    ret = ESP_FAIL;
  }

  /* Start camera monitoring task 
  log_info(system_tag, "Camera Start", "Beginning camera monitoring system");
  if (ov7670_task_start(&g_camera_data) != ESP_OK) {
    log_error(system_tag, "Camera Error", "Failed to start camera monitoring: vision system offline");
    ret = ESP_FAIL;
  } 
  TODO: Add this back in/make this work */

  /* Start sensor tasks */
  log_info(system_tag, "Sensor Start", "Beginning sensor monitoring system");
  if (sensor_tasks(&g_sensor_data) != ESP_OK) {
    log_error(system_tag, 
              "Sensor Error", 
              "Failed to start sensor tasks: environmental monitoring limited");
    ret = ESP_FAIL;
  }

  /* Start motor control tasks */
  log_info(system_tag, "Motor Start", "Beginning motor control system");
  if (motor_tasks_start(g_pwm_controller) != ESP_OK) {
    log_error(system_tag, 
              "Motor Error", 
              "Failed to start motor tasks: movement system offline");
    ret = ESP_FAIL;
  }

  if (ret == ESP_OK) {
    log_info(system_tag, "Task Complete", "All system tasks started successfully");
  } else {
    log_warn(system_tag, "Task Warning", "Some system tasks failed to start");
  }
  return ret;
}

esp_err_t system_tasks_stop(void)
{
  esp_err_t ret      = ESP_OK;
  esp_err_t temp_ret = ESP_OK;

  log_info(system_tag, "Shutdown Start", "Beginning system shutdown sequence");

  /* Stop motor control tasks */
  log_info(system_tag, "Motor Stop", "Stopping motor control system");
  temp_ret = motor_tasks_stop(g_pwm_controller);
  if (temp_ret != ESP_OK) {
    log_warn(system_tag, 
             "Motor Warning", 
             "Failed to stop motor tasks cleanly: %s", 
             esp_err_to_name(temp_ret));
    ret = ESP_FAIL;
  }

  /* Stop sensor tasks */
  log_info(system_tag, "Sensor Stop", "Stopping sensor monitoring system");
  temp_ret = sensor_tasks_stop(&g_sensor_data);
  if (temp_ret != ESP_OK) {
    log_warn(system_tag, 
             "Sensor Warning", 
             "Failed to stop sensor tasks cleanly: %s", 
             esp_err_to_name(temp_ret));
    ret = ESP_FAIL;
  }

  /* Stop camera monitoring task (if it was started) */
  /* Commented out to match the commented section in system_tasks_start
  log_info(system_tag, "Camera Stop", "Stopping camera monitoring system");
  temp_ret = ov7670_task_stop(&g_camera_data);
  if (temp_ret != ESP_OK) {
    log_warn(system_tag, 
             "Camera Warning", 
             "Failed to stop camera tasks cleanly: %s", 
             esp_err_to_name(temp_ret));
    ret = ESP_FAIL;
  }
  */

  /* Stop WiFi task */
  log_info(system_tag, "WiFi Stop", "Stopping WiFi subsystem");
  temp_ret = wifi_task_stop();
  if (temp_ret != ESP_OK) {
    log_warn(system_tag, 
             "WiFi Warning", 
             "Failed to stop WiFi task cleanly: %s", 
             esp_err_to_name(temp_ret));
    ret = ESP_FAIL;
  }

  /* Stop webserver tasks */
  log_info(system_tag, "Webserver Stop", "Stopping webserver subsystem");
  temp_ret = webserver_task_stop();
  if (temp_ret != ESP_OK) {
    log_warn(system_tag, 
             "Webserver Warning", 
             "Failed to stop webserver cleanly: %s", 
             esp_err_to_name(temp_ret));
    ret = ESP_FAIL;
  }

  /* Clean up file write manager */
  log_info(system_tag, "Storage Stop", "Cleaning up storage system");
  temp_ret = file_write_manager_cleanup();
  if (temp_ret != ESP_OK) {
    log_warn(system_tag, 
             "Storage Warning", 
             "Failed to clean up storage system: %s", 
             esp_err_to_name(temp_ret));
    ret = ESP_FAIL;
  }

  /* Clean up gait system */
  log_info(system_tag, "Gait Stop", "Cleaning up gait system");
  temp_ret = gait_cleanup(g_pwm_controller);
  if (temp_ret != ESP_OK) {
    log_warn(system_tag, 
             "Gait Warning", 
             "Failed to clean up gait system: %s", 
             esp_err_to_name(temp_ret));
    ret = ESP_FAIL;
  }

  /* Clean up motor controllers */
  log_info(system_tag, "Motor Cleanup", "Cleaning up motor controllers");
  temp_ret = motors_cleanup(&g_pwm_controller);
  if (temp_ret != ESP_OK) {
    log_warn(system_tag, 
             "Motor Warning", 
             "Failed to clean up motor controllers: %s", 
             esp_err_to_name(temp_ret));
    ret = ESP_FAIL;
  }

  /* Clean up camera subsystem */
  log_info(system_tag, "Camera Cleanup", "Cleaning up camera subsystem");
  temp_ret = ov7670_cleanup(&g_camera_data);
  if (temp_ret != ESP_OK) {
    log_warn(system_tag, 
             "Camera Warning", 
             "Failed to clean up camera subsystem: %s", 
             esp_err_to_name(temp_ret));
    ret = ESP_FAIL;
  }

  /* Clean up sensor subsystem */
  log_info(system_tag, "Sensor Cleanup", "Cleaning up sensor subsystem");
  temp_ret = sensors_cleanup(&g_sensor_data);
  if (temp_ret != ESP_OK) {
    log_warn(system_tag, 
             "Sensor Warning", 
             "Failed to clean up sensor subsystem: %s", 
             esp_err_to_name(temp_ret));
    ret = ESP_FAIL;
  }

  /* Clean up SD card subsystem */
  log_info(system_tag, "SD Card Cleanup", "Cleaning up SD card subsystem");
  temp_ret = sd_card_deinit();
  if (temp_ret != ESP_OK) {
    log_warn(system_tag, 
             "SD Card Warning", 
             "Failed to clean up SD card subsystem: %s", 
             esp_err_to_name(temp_ret));
    ret = ESP_FAIL;
  }

  /* Clean up bus manager */
  log_info(system_tag, "Bus Manager Cleanup", "Cleaning up bus manager");
  temp_ret = bus_manager_deinit_all();
  if (temp_ret != ESP_OK) {
    log_warn(system_tag, 
             "Bus Manager Warning", 
             "Failed to clean up bus manager: %s", 
             esp_err_to_name(temp_ret));
    ret = ESP_FAIL;
  }

  /* Finalize logging system (do this last) */
  log_info(system_tag, "Log Finalize", "Finalizing logging system");
  temp_ret = log_cleanup();
  if (temp_ret != ESP_OK) {
    /* Can't log this error since logging is being shut down */
    ESP_LOGW(system_tag, 
             "Log Cleanup Error: %s", 
             esp_err_to_name(temp_ret));
    ret = ESP_FAIL;
  }

  return ret;
}

