/* main/include/tasks/include/sensor_tasks.h */

#ifndef TOPOROBO_SENSOR_TASKS_H
#define TOPOROBO_SENSOR_TASKS_H

#include "sensor_hal.h"
#include <esp_err.h>

/* Structs ********************************************************************/

/**
 * @brief Structure to hold configuration for each sensor.
 *
 * This structure contains information about each sensor, including its name, 
 * initialization and task functions, a pointer to its specific data structure,
 * and an enabled flag to indicate whether the sensor should be active in the system.
 */
typedef struct {
  const char *sensor_name;             /**< Name of the sensor for identification in logs. */
  esp_err_t  (*init_function)(void *); /**< Function pointer to initialize the sensor. */
  void       (*task_function)(void *); /**< Function pointer to the sensor's data recording task. */
  void       *data_ptr;                /**< Pointer to the sensor-specific data structure. */
  bool       enabled;                  /**< Flag to indicate if the sensor is enabled (true) or disabled (false). */
} sensor_config_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initializes communication with various sensors.
 * 
 * This function will set up communication with a set of sensors:
 * - MPU6050+GY-521 (gyroscope + accelerometer)
 * - BH1750+GY-302 (light intensity sensor)
 * - DHT22 (temperature + humidity sensor)
 * - QMC5883L+GY-273 (magnetometer)
 * - GY-NEO6MV2 (GPS)
 * 
 * @note This function will establish communication via I2C, UART, or other protocols 
 * for each sensor type and ensure correct initialization.
 *
 * @param[out] sensor_data Pointer to the `sensor_data_t` struct where sensor readings will be stored.
 * @return ESP_OK if all sensors initialize successfully; ESP_FAIL if any sensor fails.
 */
esp_err_t sensors_comm_init(sensor_data_t *sensor_data);

/**
 * @brief Records sensor data and stores it in a given variable.
 * 
 * This function continuously records data from various sensors and stores the data 
 * in the provided `sensor_data_t` variable, which is passed as a parameter. The 
 * function ensures that the data is processed and ready for further use, such as 
 * uploading to a server in a later stage. The variable passed as a parameter can 
 * be used by another function to upload the data to the server.
 * 
 * Pre-condition: 
 * - The `sensors_comm_init()` function must have been successfully run to 
 *   initialize communication with all sensors before calling this function.
 * 
 * Post-condition:
 * - The `sensor_data_t` structure contains up-to-date readings from all the sensors.
 * - The data is ready to be passed to other functions for processing or uploading.
 * 
 * Sensors recorded:
 * - MPU6050+GY-521 (gyroscope + accelerometer)
 * - BH1750+GY-302 (light intensity sensor)
 * - DHT22 (temperature + humidity sensor)
 * - QMC5883L+GY-273 (magnetometer)
 * - GY-NEO6MV2 (GPS)
 * 
 * @param[in,out] sensor_data Pointer to the `sensor_data_t` struct where sensor readings will be stored.
 * @return ESP_OK if all sensor tasks start successfully; ESP_FAIL if any task fails.
 */
esp_err_t sensor_tasks(sensor_data_t *sensor_data);

#endif /* TOPOROBO_SENSOR_TASKS_H */

