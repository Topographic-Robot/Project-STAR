#include "motor_tasks.h"

/* Constants ******************************************************************/

const uint8_t num_pca9685_boards = 2;

/* Public Functions ***********************************************************/

void motor_comm_init(controller_data_t *controller_data)
{
  /* Initialize PCA9685 */
  pca9685_init(&(controller_data->pca9685_boards), num_pca9685_boards);
}

void motor_tasks(void *pv_params) 
{
  /* TODO: Implement motor task logic to move the robot */
}

