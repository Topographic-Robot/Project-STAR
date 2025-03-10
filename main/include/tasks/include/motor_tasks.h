/* main/include/tasks/include/motor_tasks.h */

#ifndef TOPOROBO_MOTOR_TASKS_H
#define TOPOROBO_MOTOR_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pca9685_hal.h"
#include "esp_err.h"

/* Constants ******************************************************************/

extern const uint8_t     num_pca9685_boards; /**< The number of boards used in this project */
extern const char* const motor_tag;          /**< Tag for logs */

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the motor tasks.
 *
 * Configures and initializes motor control tasks using the provided linked list 
 * of PCA9685 board controllers. This function prepares the controllers for 
 * operation but does not start the tasks; call `motor_tasks_start` to begin 
 * execution.
 *
 * @param[in] pwm_controller Pointer to the linked list of PCA9685 board 
 *                           controllers managing the motors.
 *
 * @return 
 * - ESP_OK              on successful initialization.
 * - ESP_ERR_INVALID_ARG if the `pwm_controller` is NULL.
 * - ESP_FAIL            for other initialization errors.
 *
 * @note Ensure that the PCA9685 controllers are properly initialized before 
 *       invoking this function.
 */
esp_err_t motors_init(pca9685_board_t** pwm_controller);

/**
 * @brief Starts the motor tasks.
 *
 * Activates motor control tasks to manage motor operations using the provided 
 * PCA9685 board controllers. This function should be called after 
 * `motors_init` to ensure all motor tasks are ready for execution.
 *
 * @param[in] pwm_controller Pointer to the linked list of PCA9685 board 
 *                           controllers managing the motors.
 *
 * @return 
 * - ESP_OK              on successful start of motor tasks.
 * - ESP_ERR_INVALID_ARG if the `pwm_controller` is NULL.
 * - ESP_FAIL            for other runtime errors.
 *
 * @note Call this function only after successfully initializing the motor tasks 
 *       using `motors_init`.
 */
esp_err_t motor_tasks_start(pca9685_board_t* pwm_controller);

/**
 * @brief Stops the motor tasks and cleans up resources.
 *
 * Stops all motor control tasks and performs cleanup of resources allocated
 * during motor initialization. This function should be called during system
 * shutdown to ensure proper cleanup of the motor subsystem.
 *
 * @param[in] pwm_controller Pointer to the PCA9685 board controller used to
 *                           manage the motors.
 *
 * @return 
 * - ESP_OK              on successful stop of motor tasks and cleanup.
 * - ESP_ERR_INVALID_ARG if the `pwm_controller` is NULL.
 * - ESP_FAIL            for other cleanup errors.
 *
 * @note This function should be called after all other components that depend on
 *       motor control have been stopped, but before the underlying hardware
 *       interfaces are deinitialized.
 */
esp_err_t motor_tasks_stop(pca9685_board_t* pwm_controller); /* TODO: Implement this function */

/**
 * @brief Cleans up resources used by the motor controllers.
 *
 * Performs cleanup of resources allocated during motor controller initialization.
 * This includes:
 * 1. Stopping all motors
 * 2. Releasing PWM channels
 * 3. Cleaning up I2C resources
 * 4. Freeing memory allocated for motor data structures
 *
 * @param[in,out] pwm_controller Pointer to pointer to the PCA9685 board controller.
 *                              The pointer will be set to NULL after cleanup.
 *
 * @return 
 * - ESP_OK              on successful cleanup.
 * - ESP_ERR_INVALID_ARG if pwm_controller is NULL.
 * - ESP_FAIL            for other cleanup errors.
 *
 * @note This function should be called after motor_tasks_stop() to ensure
 *       tasks are stopped before hardware resources are released.
 */
esp_err_t motors_cleanup(pca9685_board_t** pwm_controller);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_MOTOR_TASKS_H */

