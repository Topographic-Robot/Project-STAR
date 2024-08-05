#ifndef __BH_1750_H__
#define __BH_1750_H__

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BH1750_SENSOR_ADDR 0x23 /* Address for I2C */
#define BH1750_POWER_ON    0x01 /* Command to power on */
#define BH1750_RESET       0x07 /* Command to reset data register value */

/* Start measurement at 1 lux resolution. Measurement time is typically 120ms */
/* BH1750_CONTINUOUS_HIGH_RESOLUTION_MODE */
#define BH1750_CONT_H_RES_MODE 0x10

#define BH1750_TAG "BH1750" /* Tag for logs */

typedef i2c_config_t bh1750_t;

esp_err_t bh1750_init(uint8_t scl_io, uint8_t sda_io, uint32_t freq_hz,
                      uint8_t i2c_bus);

float bh1750_read_lux(uint8_t i2c_bus);

#ifdef __cplusplus
}
#endif

#endif /* __BH_1750_H__ */
