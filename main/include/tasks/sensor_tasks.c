/* main/include/tasks/sensor_tasks.c */

/* Initialization and Reading of Sensors through Tasks */

#include "sensor_tasks.h"
#include "system_tasks.h"
#include "esp_log.h"

/* Globals (Static) ***********************************************************/

static sensor_config_t s_sensors[] = {
  { "BH1750",     bh1750_init,     bh1750_tasks,     &(g_sensor_data.bh1750_data),     5, 4096, false },
  { "QMC5883L",   qmc5883l_init,   qmc5883l_tasks,   &(g_sensor_data.qmc5883l_data),   5, 4096, false },
  { "MPU6050",    mpu6050_init,    mpu6050_tasks,    &(g_sensor_data.mpu6050_data),    5, 4096, false },
  { "DHT22",      dht22_init,      dht22_tasks,      &(g_sensor_data.dht22_data),      5, 4096, true  },
  { "GY-NEO6MV2", gy_neo6mv2_init, gy_neo6mv2_tasks, &(g_sensor_data.gy_neo6mv2_data), 5, 4096, false },
  { "CCS811",     ccs811_init,     ccs811_tasks,     &(g_sensor_data.ccs811_data),     5, 4096, false },
  { "MQ135",      mq135_init,      mq135_tasks,      &(g_sensor_data.mq135_data),      5, 4096, false },
};

/* Public Functions ***********************************************************/

esp_err_t sensors_init(sensor_data_t *sensor_data)
{
  esp_err_t status         = ESP_OK;
  esp_err_t overall_status = ESP_OK;

  for (int i = 0; i < sizeof(s_sensors) / sizeof(sensor_config_t); i++) {
    if (s_sensors[i].enabled) {
      ESP_LOGI(system_tag, "Initializing sensor: %s", s_sensors[i].sensor_name);
      status = s_sensors[i].init_function(s_sensors[i].data_ptr);

      if (status == ESP_OK) {
        ESP_LOGI(system_tag, "Sensor %s initialized successfully",
            s_sensors[i].sensor_name);
      } else {
        ESP_LOGE(system_tag, "Sensor %s initialization failed with error: %d",
                 s_sensors[i].sensor_name, status);
        overall_status = ESP_FAIL;
      }
    } else {
      ESP_LOGI(system_tag, "Sensor %s is disabled", s_sensors[i].sensor_name);
    }
  }

  return overall_status; /* Return ESP_OK only if all s_sensors initialized successfully */
}

esp_err_t sensor_tasks(sensor_data_t *sensor_data)
{
  esp_err_t overall_status = ESP_OK;

  for (int i = 0; i < sizeof(s_sensors) / sizeof(sensor_config_t); i++) {
    if (s_sensors[i].enabled) {
      ESP_LOGI(system_tag, "Creating task for sensor: %s", s_sensors[i].sensor_name);
      BaseType_t ret = xTaskCreate(s_sensors[i].task_function, s_sensors[i].sensor_name,
                                   s_sensors[i].stack_depth, s_sensors[i].data_ptr, 
                                   s_sensors[i].priority, NULL);
      if (ret != pdPASS) {
        ESP_LOGE(system_tag, "Task creation failed for sensor: %s",
                 s_sensors[i].sensor_name);
        overall_status = ESP_FAIL;
      }
    } else {
      ESP_LOGI(system_tag, "Task for sensor %s is disabled", s_sensors[i].sensor_name);
    }
  }

  return overall_status; /* Return ESP_OK only if all tasks start successfully */
}

