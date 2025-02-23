/* main/include/tasks/sensor_tasks.c */

/* Initialization and Reading of Sensors through Tasks */

#include "sensor_tasks.h"
#include "system_tasks.h"
#include "log_handler.h"

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

  log_info(system_tag, "Init Start", "Beginning initialization of all enabled sensors");

  for (uint8_t i = 0; i < sizeof(s_sensors) / sizeof(sensor_config_t); i++) {
    if (s_sensors[i].enabled) {
      log_info(system_tag, "Sensor Init", "Initializing %s sensor", s_sensors[i].sensor_name);
      status = s_sensors[i].init_function(s_sensors[i].data_ptr);

      if (status == ESP_OK) {
        log_info(system_tag, "Init Success", "%s sensor initialized successfully", s_sensors[i].sensor_name);
      } else {
        log_error(system_tag, "Init Error", "%s sensor initialization failed: hardware error or communication issue", 
                 s_sensors[i].sensor_name);
        overall_status = ESP_FAIL;
      }
    } else {
      log_info(system_tag, "Init Skip", "%s sensor initialization skipped (disabled in configuration)", 
               s_sensors[i].sensor_name);
    }
  }

  if (overall_status == ESP_OK) {
    log_info(system_tag, "Init Complete", "All enabled sensors initialized successfully");
  } else {
    log_warn(system_tag, "Init Warning", "Sensor initialization partially complete, some sensors failed");
  }

  return overall_status;
}

esp_err_t sensor_tasks(sensor_data_t *sensor_data)
{
  esp_err_t overall_status = ESP_OK;

  log_info(system_tag, "Task Start", "Creating monitoring tasks for enabled sensors");

  for (uint8_t i = 0; i < sizeof(s_sensors) / sizeof(sensor_config_t); i++) {
    if (s_sensors[i].enabled) {
      log_info(system_tag, "Task Create", "Creating task for %s sensor", s_sensors[i].sensor_name);
      BaseType_t ret = xTaskCreate(s_sensors[i].task_function, s_sensors[i].sensor_name,
                                   s_sensors[i].stack_depth,   s_sensors[i].data_ptr, 
                                   s_sensors[i].priority,      NULL);
      if (ret != pdPASS) {
        log_error(system_tag, "Task Error", "%s sensor task creation failed: insufficient memory or resources",
                 s_sensors[i].sensor_name);
        overall_status = ESP_FAIL;
      } else {
        log_info(system_tag, "Task Success", "%s sensor task created with priority %u",
                 s_sensors[i].sensor_name, s_sensors[i].priority);
      }
    } else {
      log_info(system_tag, "Task Skip", "%s sensor task creation skipped (disabled in configuration)",
               s_sensors[i].sensor_name);
    }
  }

  if (overall_status == ESP_OK) {
    log_info(system_tag, "Task Complete", "All sensor monitoring tasks started successfully");
  } else {
    log_warn(system_tag, "Task Warning", "Some sensor tasks failed to start");
  }

  return overall_status;
}

