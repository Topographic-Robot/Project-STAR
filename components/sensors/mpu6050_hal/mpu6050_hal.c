#include "mpu6050_hal.h"
#include "common/i2c.h"
#include <esp_log.h>

/* TODO: interrupt, FIFO, PWR_MGMT_2(remove axises we don't need to safe power */

/* Constants ******************************************************************/

const uint8_t  mpu6050_i2c_address        = 0x68;
const char    *mpu6050_tag                = "MPU6050";
const uint8_t  mpu6050_scl_io             = GPIO_NUM_22;
const uint8_t  mpu6050_sda_io             = GPIO_NUM_21;
const uint32_t mpu6050_freq_hz            = 100000;
const uint32_t mpu6050_polling_rate_ticks = pdMS_TO_TICKS(5 * 1000);

/* Public Functions ***********************************************************/


