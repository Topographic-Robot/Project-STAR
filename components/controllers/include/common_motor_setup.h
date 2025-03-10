/* components/controllers/include/common_motor_setup.h */

#ifndef TOPOROBO_COMMON_MOTOR_SETUP_H
#define TOPOROBO_COMMON_MOTOR_SETUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/**
 * @brief Sets up all motor controllers
 * 
 * This function initializes all the necessary hardware resources for motor controllers,
 * including I2C bus for PCA9685 and GPIO pins for encoders.
 * 
 * @return ESP_OK if all setup operations succeeded, ESP_FAIL otherwise
 */
esp_err_t common_motor_setup(void);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_COMMON_MOTOR_SETUP_H */ 