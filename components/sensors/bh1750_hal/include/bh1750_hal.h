#ifndef TOPOROBO_BH1750_HAL_H
#define TOPOROBO_BH1750_HAL_H

#include <stdint.h>
#include <esp_err.h>

/* Constants ******************************************************************/

extern const uint8_t bh1750_power_on_cmd;
extern const uint8_t bh1750_i2c_address;
extern const uint8_t bh1750_cont_low_res_mode;
extern const uint8_t bh1750_reset_cmd;
extern const char   *bh1750_tag;

/* Structs ********************************************************************/

/**
 * @brief Structure to store BH1750 sensor data.
 *
 * This structure holds the I2C bus number used for communication and the
 * light intensity value measured by the BH1750 sensor.
 */
typedef struct bh1750_data_t {
  uint8_t i2c_bus; ///< I2C bus number used for communication.
  float lux;       ///< Measured light intensity in lux.
} bh1750_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initialize the BH1750 sensor over I2C.
 *
 * This function initializes the I2C driver for the BH1750 sensor and sets
 * it up for continuous high-resolution measurement mode. It powers on the 
 * device, resets it, and configures the resolution mode.
 *
 * @param[in] scl_io Pin number for the I2C clock line (SCL).
 * @param[in] sda_io Pin number for the I2C data line (SDA).
 * @param[in] freq_hz I2C clock frequency in Hertz.
 * @param[in,out] bh1750_data Pointer to the `bh1750_data_t` structure that 
 *   will hold the I2C bus number. The `i2c_bus` member will be set during 
 *   initialization.
 *
 * @return
 *   - ESP_OK on success.
 *   - An error code from the `esp_err_t` enumeration on failure.
 *
 * @note Delays are introduced after power on, reset, and resolution
 *   mode settings to ensure proper sensor initialization.
 */
esp_err_t bh1750_init(uint8_t scl_io, uint8_t sda_io, uint32_t freq_hz,
                      bh1750_data_t *data);

/**
 * @brief Reads light intensity data from the BH1750 sensor.
 *
 * This function reads 2 bytes of data from the BH1750 sensor and converts the
 * raw data into lux. The conversion factor is 1.2 to convert the raw sensor 
 * value into a meaningful lux value.
 *
 * @param[in,out] bh1750_data Pointer to a `bh1750_data_t` struct that contains:
 *   - `i2c_bus`: The I2C bus number to use for communication (input).
 *   - `lux`: Will be updated with the light intensity in lux (output).
 *            If the reading fails, `lux` will be set to -1.0.
 *
 * @note Ensure the I2C bus is initialized before calling this function.
 */
void bh1750_read(bh1750_data_t *bh1750_data);

#endif /* TOPOROBO_BH1750_HAL_H */
