#include "motor_tasks.h"

void motor_comm_init(/* data */)
{
  /* Initialize PCA9685 */
  //pca9685_init(&(sensor_data->pca9685_data), true);
}

void motor_tasks(void *pv_params) 
{
  /* 4. Use the PCA9685 */
  //xTaskCreate(pca9685_tasks, "pca9685_tasks", 2048, _sensor_data, 5, NULL);
}
