#ifndef TOPOROBO_I2C_H
#define TOPOROBO_I2C_H

#include <stdint.h>
#include <esp_err.h>
#include <driver/i2c.h>
#include <driver/gpio.h>

/* Constants ******************************************************************/

extern const uint32_t i2c_timeout_ticks; /* Timeout for I2C commands in ticks */

/* Private Functions **********************************************************/

/**
 * @brief Initialize the I2C interface with the specified parameters.
 *
 * This function configures the I2C bus with the provided SCL and SDA pins, 
 * pull-up configuration, and frequency. It sets the I2C mode to master 
 * and initializes the driver without requiring a buffer for the master mode.
 *
 * @param[in] scl_io Pin number for the I2C clock line (SCL).
 * @param[in] sda_io Pin number for the I2C data line (SDA).
 * @param[in] freq_hz Clock frequency in Hertz for the I2C bus.
 * @param[in,out] i2c_bus I2C bus number to use for communication.
 * @param[in] tag The tag for logging errors.
 *
 * @return 
 *   - ESP_OK on successful initialization.
 *   - An error code from the esp_err_t enumeration on failure.
 *
 * @note The function enables internal pull-ups for both SDA and SCL pins.
 *       Make sure the I2C bus is free before calling this function.
 */
esp_err_t priv_i2c_init(uint8_t scl_io, uint8_t sda_io, uint32_t freq_hz, 
                        uint8_t i2c_bus, const char *tag);

/**
 * @brief Write a byte to the I2C interface.
 *
 * This function sends a single byte of data to the an I2C device using the
 * specified I2C bus. The command link is created, and the data is written
 * using the I2C protocol.
 *
 * @note this functions don't check any semaphores, it is an internal function.
 *
 * @param[in] data The byte of data to be written to the I2C device.
 * @param[in,out] i2c_bus The I2C bus number to communicate over.
 * @param[in] tag The tag for logging errors.
 *
 * @return 
 *   - ESP_OK on success.
 *   - Error code on failure.
 */
esp_err_t priv_i2c_write_byte(uint8_t data, uint8_t i2c_bus, const char *tag);

/**
 * @brief Read multiple bytes from the I2C interface.
 *
 * This function reads a specified number of bytes from the I2C interface using
 * the I2C bus. It handles reading a single byte or multiple bytes with ACK/NACK 
 * handling for I2C communication.
 *
 * @note this functions don't check any semaphores, it is an internal function.
 *
 * @param[out] data Pointer to the buffer where read data will be stored.
 * @param[in] len The number of bytes to read.
 * @param[in] i2c_bus The I2C bus number to communicate over.
 * @param[in] tag The tag for logging errors.
 *
 * @return 
 *   - ESP_OK on success.
 *   - Error code on failure.
 */
esp_err_t priv_i2c_read_bytes(uint8_t *data, size_t len, 
                              uint8_t i2c_bus, const char *tag);

#endif /* TOPOROBO_I2C_H */
