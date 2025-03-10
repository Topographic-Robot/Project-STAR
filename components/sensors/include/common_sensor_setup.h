/* components/sensors/include/common_sensor_setup.h */

#ifndef TOPOROBO_COMMON_SENSOR_SETUP_H
#define TOPOROBO_COMMON_SENSOR_SETUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/**
 * @brief Sets up all sensors
 * 
 * This function initializes all the necessary hardware resources for sensors,
 * including I2C bus and GPIO pins.
 * 
 * @return ESP_OK if all setup operations succeeded, ESP_FAIL otherwise
 */
esp_err_t common_sensor_setup(void);

/**
 * @brief Cleans up all sensors
 * 
 * This function releases all the hardware resources used by sensors,
 * including I2C bus and GPIO pins.
 * 
 * @return ESP_OK if all cleanup operations succeeded, ESP_FAIL otherwise
 */
esp_err_t common_sensor_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_COMMON_SENSOR_SETUP_H */ 