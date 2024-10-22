/* Initialization and Reading of Sensors through Tasks */

#include "sensor_tasks.h"
#include "system_tasks.h"
#include <driver/gpio.h>
#include <esp_log.h>

/* Public Functions ***********************************************************/

void sensors_comm_init(sensor_data_t *sensor_data)
{
  /* Initialize I2C/UART communication with each sensor */

  /* Initialize BH1750 */
  bh1750_init(&(sensor_data->bh1750_data), true);

  /* Initialize DHT22 */
  dht22_init(&(sensor_data->dht22_data), true);

  /* Initialize MPU6050 */
  mpu6050_init(&(sensor_data->mpu6050_data), true);

  /* Initialize QMC5883L */
  qmc5883l_init(&(sensor_data->qmc5883l_data), true);
  
  /* Initialize GY-NEO6MV2 */
  /* TODO */
}

void sensor_tasks(void *sensor_data)
{
  sensor_data_t *_sensor_data = (sensor_data_t *)sensor_data;

  /* 1. Record data from BH1750 */
  xTaskCreate(bh1750_tasks, "bh1750_tasks", 2048, _sensor_data, 5, NULL);
  
  /* 2. Record data from DHT22 */
  xTaskCreate(dht22_tasks, "dht22_tasks", 2048, _sensor_data, 5, NULL);

  /* 3. Record data from MPU6050 */
  xTaskCreate(mpu6050_tasks, "mpu6050_tasks", 2048, _sensor_data, 5, NULL);

  /* 4. Record data from QMC5883L */
  xTaskCreate(qmc5883l_tasks, "qmc5883l_tasks", 2048, _sensor_data, 5, NULL);

  /* 5. Record data from GY-NEO6MV2 */
  /* TODO */

  ESP_LOGI(system_tag, "Sensor data recorded and stored");
}

