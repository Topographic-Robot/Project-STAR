/* main/include/tasks/sensor_tasks.c */

/* Initialization and Reading of Sensors through Tasks */

#include "sensor_tasks.h"
#include "system_tasks.h"
#include <driver/gpio.h>
#include <esp_log.h>

/* Globals (Static) ***********************************************************/

static sensor_config_t sensors[] = {
  { "BH1750",     bh1750_init,     bh1750_tasks,     NULL, false },  /* confirmed to work */
  { "QMC5883L",   qmc5883l_init,   qmc5883l_tasks,   NULL, false },  /* confirmed to work */
  { "DHT22",      dht22_init,      dht22_tasks,      NULL, false },  /* confirmed to work */
  { "MPU6050",    mpu6050_init,    mpu6050_tasks,    NULL, true },  /* pretty sure this doesnt work*/
  { "GY-NEO6MV2", gy_neo6mv2_init, gy_neo6mv2_tasks, NULL, false }, /* creates stack overflow */
};

/* Function to Initialize data_ptr Fields *************************************/

static inline void sensors_init_data_ptrs(sensor_data_t *sensor_data) {
  sensors[0].data_ptr = &(sensor_data->bh1750_data);
  sensors[1].data_ptr = &(sensor_data->qmc5883l_data);
  sensors[2].data_ptr = &(sensor_data->mpu6050_data);
  sensors[3].data_ptr = &(sensor_data->dht22_data);
  sensors[4].data_ptr = &(sensor_data->gy_neo6mv2_data);
}

/* Public Functions ***********************************************************/

esp_err_t sensors_comm_init(sensor_data_t *sensor_data) {
  esp_err_t status         = ESP_OK;
  esp_err_t overall_status = ESP_OK;

  /* Initialize data_ptr fields */
  sensors_init_data_ptrs(sensor_data);

  for (int i = 0; i < sizeof(sensors) / sizeof(sensor_config_t); i++) {
    if (sensors[i].enabled) {
      ESP_LOGI(system_tag, "Initializing sensor: %s", sensors[i].sensor_name);
      status = sensors[i].init_function(sensors[i].data_ptr);

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

  /* Ensure data_ptr fields are initialized */
  sensors_init_data_ptrs(sensor_data);

  for (int i = 0; i < sizeof(sensors) / sizeof(sensor_config_t); i++) {
    if (sensors[i].enabled) {
      ESP_LOGI(system_tag, "Creating task for sensor: %s", sensors[i].sensor_name);
      BaseType_t ret = xTaskCreate(sensors[i].task_function, sensors[i].sensor_name, 
                                   2048, sensors[i].data_ptr, 5, NULL);
      if (ret != pdPASS) {
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

