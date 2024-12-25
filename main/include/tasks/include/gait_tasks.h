/* main/include/tasks/include/gait_tasks.h */

#ifndef TOPOROBO_GAIT_TASKS_H
#define TOPOROBO_GAIT_TASKS_H

#include "esp_err.h"
#include "pca9685_hal.h"

/* Constants ******************************************************************/

extern const char   *gait_tag;          /**< Tag for logs */
extern const uint8_t max_active_servos; /**< Maximum number of servos that can be active at a time */

/* Public Functions ***********************************************************/

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
                      float distance);
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
                    float distance);

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
                      float distance);

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
                         float distance);

#endif /* TOPOROBO_GAIT_TASKS_H */

