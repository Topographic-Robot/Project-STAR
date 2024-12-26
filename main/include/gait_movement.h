/* main/include/gait_movement.h */

#ifndef TOPOROBO_GAIT_MOVEMENT_H
#define TOPOROBO_GAIT_MOVEMENT_H

#include "esp_err.h"
#include "pca9685_hal.h"

/* Constants ******************************************************************/

extern const char *gait_tag; /**< Tag for logs */

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the legs array for the hexapod robot.
 *
 * This function sets up the static array of legs, defining each leg's servo 
 * mappings, joint types, and physical constraints. It ensures the robot's 
 * gait logic can reference consistent and correctly configured data for 
 * controlling the legs during motion. This must be done after running `motor_init`
 * or handling the initialization of the pwm controller your self.
 *
 * @param[in] pwm_controller Pointer to the PCA9685 board controller used to 
 *                           drive the servos for all legs.
 *
 * @return ESP_OK on successful initialization, or ESP_FAIL if the initialization
 *         fails due to invalid configurations or other errors.
 */
esp_err_t gait_init(pca9685_board_t *pwm_controller);

/**
 * @brief Executes a tripod gait motion for the hexapod robot.
 *
 * Moves three legs at a time to achieve smooth and fast locomotion.
 *
 * @param[in] pwm_controller Pointer to the PCA9685 board controller.
 * @param[in] heading        Desired heading in degrees.
 * @param[in] distance       Distance to be traveled in centimeters.
 *
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t tripod_gait(pca9685_board_t *pwm_controller, float heading, 
                      uint16_t distance);
/**
 * @brief Executes a wave gait motion for the hexapod robot.
 *
 * This function moves one leg at a time, maintaining high stability.
 *
 * @param[in] pwm_controller Pointer to the PCA9685 board controller.
 * @param[in] heading        Desired heading in degrees.
 * @param[in] distance       Distance to be traveled in centimeters.
 *
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t wave_gait(pca9685_board_t *pwm_controller, float heading, 
                    uint16_t distance);

/**
 * @brief Executes a ripple gait motion for the hexapod robot.
 *
 * Moves two legs at a time for balanced and moderate-speed locomotion.
 *
 * @param[in] pwm_controller Pointer to the PCA9685 board controller.
 * @param[in] heading        Desired heading in degrees.
 * @param[in] distance       Distance to be traveled in centimeters.
 *
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t ripple_gait(pca9685_board_t *pwm_controller, float heading, 
                      uint16_t distance);

/**
 * @brief Executes a quadruped gait motion for the hexapod robot.
 *
 * Uses four legs to walk, leaving two stationary for enhanced stability.
 *
 * @param[in] pwm_controller Pointer to the PCA9685 board controller.
 * @param[in] heading        Desired heading in degrees.
 * @param[in] distance       Distance to be traveled in centimeters.
 *
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t quadruped_gait(pca9685_board_t *pwm_controller, float heading, 
                         uint16_t distance);

#endif /* TOPOROBO_GAIT_MOVEMENT_H */

