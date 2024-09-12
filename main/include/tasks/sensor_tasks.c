/* Initialization and Reading of Sensors through Tasks */

/*******************************************************************************
 *
 *  
 *      +-----------------------------------+
 *      |          BH1750 Sensor            |
 *      +-----------------------------------+
 *              |                   |
 *       +------+                   +------+
 *       |  SCL (I2C)                      |  SDA (I2C)
 *       |  GPIO_22 (50,000 Hz)            |  GPIO_21 (50,000 Hz)
 *  +-----------+                   +-----------+
 *  | SCL (I2C) |                   | SDA (I2C) |
 *  | GPIO_22   |                   | GPIO_21   |
 *  +-----------+                   +-----------+
 *  
 *                I2C Bus Frequency: 50000 Hz
 ******************************************************************************/

#include "sensor_tasks.h"
#include "bh1750_hal.h"
#include <driver/gpio.h>


/* Constants ******************************************************************/

const uint8_t  bh1750_scl_io  = GPIO_NUM_22;
const uint8_t  bh1750_sda_io  = GPIO_NUM_21;
const uint32_t bh1750_freq_hz = 50000; /* in Hz */

/* Public Functions ***********************************************************/

void sensors_comm_init(sensor_data_t *sensor_data)
{
  /* Initialize I2C/UART communication with each sensor */
  bh1750_init(&(sensor_data->bh1750_data));
}

void sensor_tasks(void *sensor_data)
{
  sensor_data_t *_sensor_data = (sensor_data_t *)sensor_data;
//    /* 1. Record data from MPU6050 */
//    mpu6050_read(sensor_data->mpu6050_data);
//    
  /* 2. Record data from BH1750 */
  bh1750_read(&(_sensor_data->bh1750_data));
  bh1750_reset_on_error(&(_sensor_data->bh1750_data));
    
//    /* 3. Record data from HC-SR04 */
//    hc_sr04_read(sensor_data->hc_sr04_data);
//    
//    /* 4. Record data from DHT22 */
//    dht22_read(sensor_data->dht22_data);
//    
//    /* 5. Record data from HMC5883L */
//    hmc5883l_read(sensor_data->hmc5883l_data);
//    
//    /* 6. Record data from GY-NEO6MV2 */
//    gy_neo6mv2_read(sensor_data->gy_neo6mv2_data);
//    
//    ESP_LOGI(sensor_tag, "Sensor data recorded and stored");
}

