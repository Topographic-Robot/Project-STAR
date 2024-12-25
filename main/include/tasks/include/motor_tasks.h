/* main/include/tasks/include/motor_tasks.h */

#ifndef TOPOROBO_MOTOR_TASKS_H
#define TOPOROBO_MOTOR_TASKS_H

#include "esp_err.h"
#include "pca9685_hal.h"

/* Constants ******************************************************************/

extern const uint8_t num_pca9685_boards; /**< The number of boards used in this project */
extern const char   *motor_tag;          /**< Tag for logs */

/* Public Functions ***********************************************************/

/**
 * @brief Initialize the motor tasks
 *
 * @param pwm_controller Linked list of all the pwm controllers
 *
 * @return ESP_OK on success, otherwise an error code
 */
esp_err_t motors_init(pca9685_board_t **pwm_controller);

/**
  * @brief Start the motor tasks
  *
  * @param pwm_controller Linked list of all the pwm controllers
  *
  * @return ESP_OK on success, otherwise an error code
  */
esp_err_t motor_tasks_start(pca9685_board_t *pwm_controller);

#endif /* TOPOROBO_MOTOR_TASKS_H */

