/* main/include/tasks/include/motor_tasks.h */

#ifndef TOPOROBO_MOTOR_TASKS_H
#define TOPOROBO_MOTOR_TASKS_H

#include "esp_err.h"
#include "pca9685_hal.h"

/* Constants ******************************************************************/

extern const uint8_t num_pca9685_boards; /**< The number of boards used in this project */
extern const char   *motor_tag;          /**< Tag for logs */

/* Public Functions ***********************************************************/

/* TODO: Add doc comments */
esp_err_t motors_init(pca9685_board_t **pwm_controller_linked_list);

/* TODO: Add doc comments */
esp_err_t motor_tasks_start(pca9685_board_t *pwm_controller_linked_list);

#endif /* TOPOROBO_MOTOR_TASKS_H */

