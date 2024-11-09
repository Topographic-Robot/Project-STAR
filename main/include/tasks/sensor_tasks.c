/* Initialization  and Reading of Sensors through Tasks */

#include "sensor_tasks.h"
#include "portmacro.h"
#include "system_tasks.h"
#include <driver/gpio.h>
#include <esp_log.h>

/* Globals (Static) ***********************************************************/

static sensor_config_t sensors[] = {
  { "BH1750",     bh1750_init,     bh1750_tasks,     false }, /* confirmed to work */
  { "QMC5883L",   qmc5883l_init,   qmc5883l_tasks,   false }, /* pretty sure doesnt work */
  { "MPU6050",    mpu6050_init,    mpu6050_tasks,    false }, /* pretty sure this does work */
  { "DHT22",      dht22_init,      dht22_tasks,      false }, /* confirmed to work */
  { "GY-NEO6MV2", gy_neo6mv2_init, gy_neo6mv2_tasks, false }, /* creates stack overflow */
};

/* Public Functions ***********************************************************/

esp_err_t sensors_comm_init(sensor_data_t *sensor_data) {
  esp_err_t status         = ESP_OK;
  esp_err_t overall_status = ESP_OK;

  for (int i = 0; i < sizeof(sensors) / sizeof(sensor_config_t); i++) {
    if (sensors[i].enabled) {
      ESP_LOGI(system_tag, "Initializing sensor: %s", sensors[i].sensor_name);
      status = sensors[i].init_function(sensor_data);

      if (status == ESP_OK) {
        ESP_LOGI(system_tag, "Sensor %s initialized successfully", 
                 sensors[i].sensor_name);
      } else {
        ESP_LOGE(system_tag, "Sensor %s initialization failed with error: %d", 
                 sensors[i].sensor_name, status);
        overall_status = ESP_FAIL;
      }
    } else {
      ESP_LOGI(system_tag, "Sensor %s is disabled", sensors[i].sensor_name);
    }
  }

  return overall_status; /* Return ESP_OK only if all sensors initialized successfully */
}

esp_err_t sensor_tasks(sensor_data_t *sensor_data) {
  esp_err_t overall_status = ESP_OK;

  for (int i = 0; i < sizeof(sensors) / sizeof(sensor_config_t); i++) {
    if (sensors[i].enabled) {
      ESP_LOGI(system_tag, "Creating task for sensor: %s", sensors[i].sensor_name);
      BaseType_t ret = xTaskCreate(sensors[i].task_function, sensors[i].sensor_name, 
                                   2048, sensor_data, 5, NULL);
      if (ret!= pdPASS) {
        ESP_LOGE(system_tag, "Task creation failed for sensor: %s", 
                 sensors[i].sensor_name);
        overall_status = ESP_FAIL;
      }
    } else {
      ESP_LOGI(system_tag, "Task for sensor %s is disabled", sensors[i].sensor_name);
    }
  }

  return overall_status; /* Return ESP_OK only if all tasks start successfully */
}
