/* Initialization and Reading of Sensors through Tasks */

#include "sensor_tasks.h"
#include "system_tasks.h"
#include "sensor_hal.h"
#include <driver/gpio.h>
#include <esp_log.h>

/* Public Functions ***********************************************************/

void sensors_comm_init(sensor_data_t *sensor_data)
{
  /* Initialize I2C/UART communication with each sensor */
  bh1750_init(&(sensor_data->bh1750_data), true);
  dht22_init(&(sensor_data->dht22_data), true);
}

void sensor_tasks(void *sensor_data)
{
  sensor_data_t *_sensor_data = (sensor_data_t *)sensor_data;

  /* 1. Record data from BH1750 */
  xTaskCreate(bh1750_tasks, "bh1750_tasks", 2048, _sensor_data, 5, NULL);
  
  /* 2. Record data from DHT22 */
  xTaskCreate(dht22_tasks, "dht22_tasks", 2048, _sensor_data, 5, NULL);

  ESP_LOGI(system_tag, "Sensor data recorded and stored");
}

