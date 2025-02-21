/* main/include/tasks/sensor_tasks.c */

/* Initialization and Reading of Sensors through Tasks */

#include "sensor_tasks.h"
#include "system_tasks.h"
#include "esp_log.h"

/* Globals (Static) ***********************************************************/

static sensor_config_t s_sensors[] = {
  { "BH1750",     bh1750_init,     bh1750_tasks,     &(g_sensor_data.bh1750_data),     5, 4096, false }, /* works bh1750 */
  { "QMC5883L",   qmc5883l_init,   qmc5883l_tasks,   &(g_sensor_data.qmc5883l_data),   5, 4096, false }, /* works qmc5883l */
  { "MPU6050",    mpu6050_init,    mpu6050_tasks,    &(g_sensor_data.mpu6050_data),    5, 4096, false }, /* works mpu6050, but needs to be configured */
  { "DHT22",      dht22_init,      dht22_tasks,      &(g_sensor_data.dht22_data),      5, 4096, false }, /* works dht22 */
  { "GY-NEO6MV2", gy_neo6mv2_init, gy_neo6mv2_tasks, &(g_sensor_data.gy_neo6mv2_data), 5, 4096, false }, /* doesn't work gy-neo6mv2 */
  { "CCS811",     ccs811_init,     ccs811_tasks,     &(g_sensor_data.ccs811_data),     5, 4096, false }, /* doesn't work ccs811 */
  { "MQ135",      mq135_init,      mq135_tasks,      &(g_sensor_data.mq135_data),      5, 4096, false }, /* works mq135 */
};

/* Public Functions ***********************************************************/

esp_err_t sensors_init(sensor_data_t *sensor_data)
{
  esp_err_t status         = ESP_OK;
  esp_err_t overall_status = ESP_OK;

  ESP_LOGI(system_tag, "- Starting sensor initialization - configuring all enabled sensors");

  for (uint8_t i = 0; i < sizeof(s_sensors) / sizeof(sensor_config_t); i++) {
    if (s_sensors[i].enabled) {
      ESP_LOGI(system_tag, "- Initializing %s sensor", s_sensors[i].sensor_name);
      status = s_sensors[i].init_function(s_sensors[i].data_ptr);

      if (status == ESP_OK) {
        ESP_LOGI(system_tag, "- %s: Initialization successful", s_sensors[i].sensor_name);
      } else {
        ESP_LOGE(system_tag, "- %s: Initialization failed - hardware error or communication issue", 
                 s_sensors[i].sensor_name);
        overall_status = ESP_FAIL;
      }
    } else {
      ESP_LOGI(system_tag, "- %s: Initialization skipped - sensor disabled in configuration", 
               s_sensors[i].sensor_name);
    }
  }

  if (overall_status == ESP_OK) {
    ESP_LOGI(system_tag, "- Sensor initialization complete - all enabled sensors ready");
  } else {
    ESP_LOGW(system_tag, "- Sensor initialization partially complete - some sensors failed");
  }

  return overall_status;
}

esp_err_t sensor_tasks(sensor_data_t *sensor_data)
{
  esp_err_t overall_status = ESP_OK;

  ESP_LOGI(system_tag, "- Starting sensor tasks - creating monitoring threads");

  for (uint8_t i = 0; i < sizeof(s_sensors) / sizeof(sensor_config_t); i++) {
    if (s_sensors[i].enabled) {
      ESP_LOGI(system_tag, "- Creating task for %s sensor", s_sensors[i].sensor_name);
      BaseType_t ret = xTaskCreate(s_sensors[i].task_function, s_sensors[i].sensor_name,
                                   s_sensors[i].stack_depth,   s_sensors[i].data_ptr, 
                                   s_sensors[i].priority,      NULL);
      if (ret != pdPASS) {
        ESP_LOGE(system_tag, "- %s: Task creation failed - insufficient memory or resources",
                 s_sensors[i].sensor_name);
        overall_status = ESP_FAIL;
      } else {
        ESP_LOGI(system_tag, "- %s: Task created successfully with priority %u",
                 s_sensors[i].sensor_name, s_sensors[i].priority);
      }
    } else {
      ESP_LOGI(system_tag, "- %s: Task creation skipped - sensor disabled in configuration",
               s_sensors[i].sensor_name);
    }
  }

  if (overall_status == ESP_OK) {
    ESP_LOGI(system_tag, "- Sensor tasks started - all monitoring threads active");
  } else {
    ESP_LOGW(system_tag, "- Sensor tasks partially started - some threads failed to start");
  }

  return overall_status;
}

