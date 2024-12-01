/* main/include/tasks/sensor_tasks.c */

/* Initialization and Reading of Sensors through Tasks */

#include "sensor_tasks.h"
#include "system_tasks.h"
#include "esp_log.h"

/* Globals (Static) ***********************************************************/

static sensor_config_t sensors[] = {
  { "BH1750",     bh1750_init,     bh1750_tasks,     &(s_sensor_data.bh1750_data),     false },
  { "QMC5883L",   qmc5883l_init,   qmc5883l_tasks,   &(s_sensor_data.qmc5883l_data),   false },
  { "MPU6050",    mpu6050_init,    mpu6050_tasks,    &(s_sensor_data.mpu6050_data),    false },
  { "DHT22",      dht22_init,      dht22_tasks,      &(s_sensor_data.dht22_data),      false },
  { "GY-NEO6MV2", gy_neo6mv2_init, gy_neo6mv2_tasks, &(s_sensor_data.gy_neo6mv2_data), true  },
  { "CCS811",     ccs811_init,     ccs811_tasks,     &(s_sensor_data.ccs811_data),     false },
  { "MQ135",      mq135_init,      mq135_tasks,      &(s_sensor_data.mq135_data),      false },
};

/* Public Functions ***********************************************************/

esp_err_t sensors_init(sensor_data_t *sensor_data)
{
  esp_err_t status         = ESP_OK;
  esp_err_t overall_status = ESP_OK;

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

esp_err_t sensor_tasks(sensor_data_t *sensor_data)
{
  esp_err_t overall_status = ESP_OK;

  for (int i = 0; i < sizeof(sensors) / sizeof(sensor_config_t); i++) {
    if (sensors[i].enabled) {
      ESP_LOGI(system_tag, "Creating task for sensor: %s", sensors[i].sensor_name);
      BaseType_t ret = xTaskCreate(sensors[i].task_function, sensors[i].sensor_name,
          4096, sensors[i].data_ptr, 5, NULL);
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

