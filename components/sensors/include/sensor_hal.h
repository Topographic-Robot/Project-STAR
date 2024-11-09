#ifndef TOPOROBO_SENSOR_HAL_H
#define TOPOROBO_SENSOR_HAL_H

#include "bh1750_hal.h"
#include "dht22_hal.h"
#include "mpu6050_hal.h"
#include "qmc5883l_hal.h"
#include "gy_neo6mv2_hal.h"

/* Structs ********************************************************************/

/**
 * @brief Structure to store data from various sensors.
 * 
 * This structure holds the recorded sensor data from all the connected sensors, 
 * including the BH1750 light sensor, MPU6050 gyroscope + accelerometer,
 * DHT22 temperature + humidity sensor, QMC5883L magnetometer, 
 * and GY-NEO6MV2 GPS sensor.
 * 
 * Each member of this structure corresponds to the data collected from a specific sensor type.
 * 
 * Members:
 * - `bh1750_data`: Holds data from the BH1750 light intensity sensor.
 * - `mpu6050_data`: Holds data from the MPU6050 gyroscope + accelerometer sensor.
 * - `dht22_data`: Holds data from the DHT22 temperature + humidity sensor.
 * - `qmc5883l_data`: Holds data from the QMC5883L magnetometer.
 * - `gy_neo6mv2_data`: Holds data from the GY-NEO6MV2 GPS module.
 */
typedef struct {
  bh1750_data_t     bh1750_data;     /**< BH1750 light intensity data */
  dht22_data_t      dht22_data;      /**< DHT22 temperature + humidity data */
  mpu6050_data_t    mpu6050_data;    /**< MPU6050 gyroscope + accelerometer data */
  qmc5883l_data_t   qmc5883l_data;   /**< QMC5883L magnetometer data */
  gy_neo6mv2_data_t gy_neo6mv2_data; /**< GY-NEO6MV2 GPS data */
} sensor_data_t;

#endif /* TOPOROBO_SENSOR_HAL_H */
