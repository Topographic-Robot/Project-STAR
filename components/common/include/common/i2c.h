#ifndef TOPOROBO_I2C_H
#define TOPOROBO_I2C_H

#include <stdint.h>
#include <esp_err.h>
#include <driver/i2c.h>
#include <driver/gpio.h>

/* Private Functions **********************************************************/

/**
 * @brief Initialize the I2C interface with the specified parameters.
 *
 * This function configures the I2C bus with the provided SCL and SDA pins, 
 * pull-up configuration, and frequency. It sets the I2C mode to master 
 * and initializes the driver without requiring a buffer for the master mode.
 *
 * @param scl_io Pin number for the I2C clock line (SCL).
 * @param sda_io Pin number for the I2C data line (SDA).
 * @param freq_hz Clock frequency in Hertz for the I2C bus.
 * @param i2c_bus I2C bus number to use for communication.
 * @param tag The tag for logging errors.
 *
 * @return 
 *      - ESP_OK on successful initialization.
 *      - An error code from the esp_err_t enumeration on failure.
 *
 * @note The function enables internal pull-ups for both SDA and SCL pins.
 *       Make sure the I2C bus is free before calling this function.
 */
esp_err_t priv_i2c_init(uint8_t scl_io, uint8_t sda_io, uint32_t freq_hz, 
                        uint8_t i2c_bus, const char* tag);

#endif /* TOPOROBO_I2C_H */
