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
#include "common/common_cleanup.h"
#include "private/wrapper_functions.h"
#include "private/wrapper_cleanup.h"

/* Constants ******************************************************************/

const char* const system_tag = "Project Star";

/* Globals ********************************************************************/

sensor_data_t    g_sensor_data    = {};
pca9685_board_t* g_pwm_controller = {};
ov7670_data_t    g_camera_data    = {}; /* TODO: Make this support all 6 cameras */

/* Globals (Static) ***********************************************************/

static esp_err_t priv_clear_nvs_flash(void); /* Forward Declaration */

static system_component_config_t s_system_components[] = {
  { "NVS Flash",       priv_clear_nvs_flash,           NULL,                     NULL,                    NULL,                       true },
  { "Logging",         wrapper_log_init,               NULL,                     NULL,                    log_cleanup,                true },
  { "Log Compression", wrapper_log_storage_set_compression, NULL,                NULL,                    NULL,                       true },
  { "Storage",         file_write_manager_init,        NULL,                     NULL,                    file_write_manager_cleanup, true },
  { "WiFi",            NULL,                           wifi_task_start,          wifi_task_stop,          NULL,                       true },
  { "Webserver",       NULL,                           NULL,                     NULL,                    webserver_task_stop,        true },
  { "Sensors",         wrapper_sensors_init,           wrapper_sensor_tasks,     wrapper_sensor_tasks_stop, wrapper_sensors_cleanup,  true },
  { "Camera",          wrapper_ov7670_init,            NULL,                     NULL,                    wrapper_ov7670_cleanup,     true },
  { "Motors",          wrapper_motors_init,            wrapper_motor_tasks_start, wrapper_motor_tasks_stop, wrapper_motors_cleanup,   true },
  { "Gait",            wrapper_gait_init,              NULL,                     NULL,                    wrapper_gait_cleanup,       true },
};

/* Private (Static) Functions *************************************************/

/**
 * @brief Initializes and resets the ESP32's Non-Volatile Storage (NVS) flash if needed.
 *
 * Initializes the NVS flash. If no free pages are available or a version mismatch is detected, 
 * the function will erase the NVS partition and reinitialize it.
 *
 * @return 
 * - ESP_OK if NVS flash was initialized successfully.
 * - Other error codes from the ESP-IDF NVS functions if initialization fails.
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
  esp_err_t overall_status = ESP_OK;

  log_info(system_tag, "Init Start", "Beginning initialization of system components");

  for (uint8_t i = 0; i < sizeof(s_system_components) / sizeof(system_component_config_t); i++) {
    if (s_system_components[i].enabled && s_system_components[i].init_function) {
      log_info(system_tag, 
               "Component Init", 
               "Initializing %s component", 
               s_system_components[i].component_name);
      
      esp_err_t status = s_system_components[i].init_function();

      if (status == ESP_OK) {
        log_info(system_tag, 
                 "Init Success", 
                 "%s component initialized successfully", 
                 s_system_components[i].component_name);
      } else {
        log_error(system_tag, 
                  "Init Error", 
                  "%s component initialization failed", 
                  s_system_components[i].component_name);
        overall_status = ESP_FAIL;
      }
    }
  }

  if (overall_status == ESP_OK) {
    log_info(system_tag, 
             "Init Complete", 
             "All system components initialized successfully");
  } else {
    log_warn(system_tag, 
             "Init Warning", 
             "System initialization incomplete: some components failed");
  }

  return overall_status;
}

esp_err_t system_tasks_start(void)
{
  esp_err_t overall_status = ESP_OK;

  log_info(system_tag, "Task Start", "Beginning startup of system tasks");

  for (uint8_t i = 0; i < sizeof(s_system_components) / sizeof(system_component_config_t); i++) {
    if (s_system_components[i].enabled && s_system_components[i].start_function) {
      log_info(system_tag, 
               "Task Create", 
               "Starting %s task", 
               s_system_components[i].component_name);
      
      esp_err_t status = s_system_components[i].start_function();

      if (status == ESP_OK) {
        log_info(system_tag, 
                 "Task Success", 
                 "%s task started successfully", 
                 s_system_components[i].component_name);
      } else {
        log_error(system_tag, 
                  "Task Error", 
                  "%s task failed to start", 
                  s_system_components[i].component_name);
        overall_status = ESP_FAIL;
      }
    }
  }

  if (overall_status == ESP_OK) {
    log_info(system_tag, 
             "Task Complete", 
             "All system tasks started successfully");
  } else {
    log_warn(system_tag, 
             "Task Warning", 
             "Some system tasks failed to start");
  }

  return overall_status;
}

esp_err_t system_tasks_stop(void)
{
  esp_err_t overall_status = ESP_OK;

  log_info(system_tag, "Shutdown Start", "Beginning system shutdown sequence");

  /* Iterate in reverse order to ensure proper cleanup */
  for (int8_t i = sizeof(s_system_components) / sizeof(system_component_config_t) - 1; i >= 0; i--) {
    if (s_system_components[i].enabled) {
      /* First try to stop tasks */
      if (s_system_components[i].stop_function) {
        log_info(system_tag, 
                 "Task Stop", 
                 "Stopping %s task", 
                 s_system_components[i].component_name);
        
        esp_err_t status = s_system_components[i].stop_function();

        if (status != ESP_OK) {
          log_warn(system_tag, 
                   "Task Warning", 
                   "Failed to stop %s task cleanly", 
                   s_system_components[i].component_name);
          overall_status = ESP_FAIL;
        }
      }
    }
  }

  /* Group cleanup operations by subsystem */
  esp_err_t (*cleanup_funcs[])(void) = {
    wrapper_cleanup_all_motors,
    wrapper_cleanup_all_sensors,
    wrapper_cleanup_all_system
  };
  
  esp_err_t cleanup_status = common_cleanup_multiple(system_tag, 
                                                    "System", 
                                                    cleanup_funcs, 
                                                    sizeof(cleanup_funcs) / sizeof(cleanup_funcs[0]));
  
  if (cleanup_status != ESP_OK) {
    overall_status = ESP_FAIL;
  }

  if (overall_status == ESP_OK) {
    log_info(system_tag, 
             "Shutdown Complete", 
             "System shutdown completed successfully");
  } else {
    log_warn(system_tag, 
             "Shutdown Warning", 
             "System shutdown partially complete, some components failed");
  }

  return overall_status;
}

