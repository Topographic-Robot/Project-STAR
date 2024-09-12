#include "sensor_tasks.h"
#include "bh1750_hal.h"
#include "hal/i2c_types.h"

/* Public Functions ***********************************************************/

/* TODO: Make this verify, check and do it again maybe like before every run */
/* TODO: This function has to take in sensor_data as a parm for these init functions */
void sensors_comm_init(void)
{
  /* 1. Initialize I2C/UART communication with each sensor */
  /* bh1750_init(uint8_t scl_io, uint8_t sda_io, uint32_t freq_hz, bh1750_data_t *data); */
  /* 2. Verify the initialization of each sensor */
  /* 3. Log the status of each sensor's communication setup */
}

void sensor_tasks(void *sensor_data)
{
//    /* 1. Record data from MPU6050 */
//    mpu6050_read(sensor_data->mpu6050_data);
//    
//    /* 2. Record data from BH1750 */
  bh1750_read(&(((sensor_data_t *)sensor_data)->bh1750_data));
//    
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

