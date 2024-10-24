#ifndef TOPOROBO_MOTOR_TASKS_H
#define TOPOROBO_MOTOR_TASKS_H

#include "controller_hal.h"

/* Constants ******************************************************************/

extern const uint8_t num_pca9685_boards; /**< Number of PCA9685 boards */

/* Structs ********************************************************************/

/**
 * @struct controller_data_t
 * @brief Structure to hold multiple PCA9685 boards and manage them.
 *
 * This structure holds an array of PCA9685 boards and the number of
 * boards that are being initialized and used in the system.
 */
typedef struct {
  pca9685_board_t *pca9685_boards; /**< Singly Linked List of PCA9685 boards */
} controller_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initialize the motor communication system.
 *
 * This function initializes all PCA9685 boards specified in the 
 * `controller_data` structure. It prepares the boards for controlling 
 * motors and ensures proper setup for robot movement.
 *
 * @param[in,out] controller_data Pointer to the `controller_data_t` structure 
 *                                containing the list of PCA9685 boards to 
 *                                initialize.
 */
void motor_comm_init(controller_data_t *controller_data);

/**
 * @brief Motor task handler function for robot movement.
 *
 * This function is responsible for executing motor-related tasks that control
 * the movement of the robot. It is typically executed within the context of a 
 * FreeRTOS task and manages motor control via the PCA9685 boards.
 *
 * @param[in] pv_params Pointer to task parameters (currently unused).
 */
void motor_tasks(void *pv_params);

#endif /* TOPOROBO_MOTOR_TASKS_H */

