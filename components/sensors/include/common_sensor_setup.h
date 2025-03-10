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

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_COMMON_SENSOR_SETUP_H */ 