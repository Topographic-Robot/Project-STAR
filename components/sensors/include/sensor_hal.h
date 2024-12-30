/* components/sensors/include/sensor_hal.h */

#ifndef TOPOROBO_SENSOR_HAL_H
#define TOPOROBO_SENSOR_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bh1750_hal.h"
#include "dht22_hal.h"
#include "mpu6050_hal.h"
#include "qmc5883l_hal.h"
#include "gy_neo6mv2_hal.h"
#include "ccs811_hal.h"
#include "mq135_hal.h"

/* Structs ********************************************************************/

/**
 * @brief Structure to store data from various connected sensors.
 *
 * Contains the recorded data from all connected sensors, including light intensity,
 * temperature, humidity, gyroscope, accelerometer, magnetometer, air quality, and GPS.
 * Each field represents the data collected from a specific sensor type.
 */
typedef struct {
  bh1750_data_t     bh1750_data;     /**< Data from the BH1750 light intensity sensor. */
  dht22_data_t      dht22_data;      /**< Data from the DHT22 temperature and humidity sensor. */
  mpu6050_data_t    mpu6050_data;    /**< Data from the MPU6050 gyroscope and accelerometer sensor. */
  qmc5883l_data_t   qmc5883l_data;   /**< Data from the QMC5883L magnetometer sensor. */
  gy_neo6mv2_data_t gy_neo6mv2_data; /**< Data from the GY-NEO6MV2 GPS module. */
  ccs811_data_t     ccs811_data;     /**< Data from the CCS811 air quality sensor (eCO2 and TVOC levels). */
  mq135_data_t      mq135_data;      /**< Data from the MQ135 air quality sensor (e.g., CO2, ammonia, benzene). */
} sensor_data_t;

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_SENSOR_HAL_H */
