#ifndef TOPOROBO_SENSOR_TASKS_H
#define TOPOROBO_SENSOR_TASKS_H

#include "sensor_hal.h"

/* Structs ********************************************************************/

/**
 * @brief Structure to store data from various sensors.
 * 
 * This structure holds the recorded sensor data from all the connected sensors, 
 * including the BH1750 light sensor, MPU6050 gyroscope + accelerometer, HC-SR04 
 * ultrasonic distance sensor, DHT22 temperature + humidity sensor, HMC5883L magnetometer, 
 * and GY-NEO6MV2 GPS sensor.
 * 
 * Each member of this structure corresponds to the data collected from a specific sensor type.
 * 
 * Members:
 * - `bh1750_data`: Holds data from the BH1750 light intensity sensor.
 * - `mpu6050_data`: Holds data from the MPU6050 gyroscope + accelerometer sensor.
 * - `hc_sr04_data`: Holds data from the HC-SR04 ultrasonic distance sensor.
 * - `dht22_data`: Holds data from the DHT22 temperature + humidity sensor.
 * - `hmc5883l_data`: Holds data from the HMC5883L magnetometer.
 * - `gy_neo6mv2_data`: Holds data from the GY-NEO6MV2 GPS module.
 */
typedef struct {
  bh1750_data_t bh1750_data; ///< BH1750 light intensity data
  dht22_data_t   dht22_data; ///< DHT22 temperature + humidity data 
/*  mpu6050_data_t mpu6050_data;       ///< MPU6050 gyroscope + accelerometer data */
/*  hc_sr04_data_t hc_sr04_data;       ///< HC-SR04 ultrasonic distance data */
/*  hmc5883l_data_t hmc5883l_data;     ///< HMC5883L magnetometer data */
/*  gy_neo6mv2_data_t gy_neo6mv2_data; ///< GY-NEO6MV2 GPS data */
} sensor_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initializes communication with various sensors.
 * 
 * This function will set up communication with a set of sensors:
 * - MPU6050+GY-521 (gyroscope + accelerometer)
 * - BH1750+GY-302 (light intensity sensor)
 * - HC-SR04 (ultrasonic distance sensor)
 * - DHT22 (temperature + humidity sensor)
 * - HMC5883L+GY-273 (magnetometer)
 * - GY-NEO6MV2 (GPS)
 * 
 * @note This function will establish communication via I2C, UART, or other protocols 
 * for each sensor type and ensure correct initialization.
 *
 * @param[out] sensor_data Pointer to the `sensor_data_t` struct where sensor readings will be stored.
 */
void sensors_comm_init(sensor_data_t *sensor_data);

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
 * - HC-SR04 (ultrasonic distance sensor)
 * - DHT22 (temperature + humidity sensor)
 * - HMC5883L+GY-273 (magnetometer)
 * - GY-NEO6MV2 (GPS)
 * 
 * @param[in,out] sensor_data Pointer to the `sensor_data_t` struct where sensor readings will be stored.
 */
void sensor_tasks(void *sensor_data);

#endif /* TOPOROBO_SENSOR_TASKS_H */
