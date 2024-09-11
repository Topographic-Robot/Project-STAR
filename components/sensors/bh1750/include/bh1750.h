#ifndef TOPOROBO_BH1750_H
#define TOPOROBO_BH1750_H

#include <stdint.h>
#include <esp_err.h>

/* Constants ******************************************************************/

extern const uint8_t bh1750_power_on_cmd;
extern const uint8_t bh1750_i2c_address;
extern const uint8_t bh1750_cont_low_res_mode;
extern const uint8_t bh1750_reset_cmd;
extern const char   *bh1750_tag;

/* Public Functions ***********************************************************/

/**
 * @brief Initialize the BH1750 sensor over I2C.
 *
 * This function initializes the I2C driver for the BH1750 sensor and sets
 * it up for continuous high-resolution measurement mode. It powers on the 
 * device, resets it, and configures the resolution mode.
 *
 * @param scl_io Pin number for the I2C clock line (SCL).
 * @param sda_io Pin number for the I2C data line (SDA).
 * @param freq_hz I2C clock frequency in Hertz.
 * @param i2c_bus I2C bus number to use.
 *
 * @return 
 *      - ESP_OK on success.
 *      - An error code from the esp_err_t enumeration on failure.
 *
 * @note Delays are introduced after power on, reset, and resolution 
 *       mode settings to ensure proper sensor initialization.
 */
esp_err_t bh1750_init(uint8_t scl_io, uint8_t sda_io, uint32_t freq_hz,
                      uint8_t i2c_bus);

/**
 * @brief Reads light intensity data from the BH1750 sensor.
 *
 * This function reads 2 bytes of data from the BH1750 sensor and converts the 
 * raw data into lux. The conversion factor is 1.2 to convert the raw sensor 
 * value into a meaningful lux value.
 *
 * @param i2c_bus I2C bus number to use for communication.
 *
 * @return 
 *      - The light intensity in lux on success.
 *      - -1.0 if reading data from the sensor fails.
 *
 * @note Ensure the I2C bus is initialized before calling this function.
 */
float bh1750_read_lux(uint8_t i2c_bus);

#endif /* TOPOROBO_BH1750_H */
