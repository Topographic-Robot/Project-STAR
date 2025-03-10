/* main/include/tasks/include/gait_movement.h */

#ifndef TOPOROBO_GAIT_MOVEMENT_H
#define TOPOROBO_GAIT_MOVEMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "pca9685_hal.h"

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the gait system.
 *
 * Sets up the gait system by:
 * 1. Mapping motors to legs and joints
 * 2. Initializing leg configurations
 * 3. Setting up gait patterns
 *
 * @param[in] pwm_controller Pointer to the PCA9685 board controller used to
 *                          manage the motors.
 *
 * @return 
 * - ESP_OK              on successful initialization.
 * - ESP_ERR_INVALID_ARG if pwm_controller is NULL.
 * - ESP_FAIL           if initialization fails.
 */
esp_err_t gait_init(pca9685_board_t* const pwm_controller);

/**
 * @brief Cleans up resources used by the gait system.
 *
 * Performs cleanup of resources allocated during gait system initialization.
 * This includes:
 * 1. Stopping any active gait patterns
 * 2. Resetting all motors to neutral positions (90 degrees)
 * 3. Freeing memory allocated for gait patterns
 * 4. Resetting gait system state
 *
 * @param[in] pwm_controller Pointer to the PCA9685 board controller used to
 *                          manage the motors.
 *
 * @return 
 * - ESP_OK              on successful cleanup.
 * - ESP_ERR_INVALID_ARG if pwm_controller is NULL.
 * - ESP_FAIL           if cleanup fails.
 *
 * @note This function should be called during system shutdown after motor tasks
 *       have been stopped but before motor hardware resources are released.
 */
esp_err_t gait_cleanup(pca9685_board_t* const pwm_controller);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_GAIT_MOVEMENT_H */ 