/* components/common/include/common/i2c.h */

#ifndef TOPOROBO_I2C_H
#define TOPOROBO_I2C_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

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
 * @brief Write a byte to a specific I2C device.
 *
 * This function transmits a single byte of data to a designated I2C device
 * using the specified I2C bus and device address. It creates an I2C command
 * link, sends the I2C address with the write flag, writes the data byte, and
 * then completes the transaction by stopping the I2C communication.
 *
 * @note This function is intended for internal use and does not implement
 *       semaphore checks or concurrency protections.
 *
 * @param[in] data The byte of data to send to the I2C device.
 * @param[in] i2c_bus The I2C bus number to communicate over.
 * @param[in] i2c_address The 7-bit I2C address of the target device.
 * @param[in] tag The tag for logging errors in case of failure.
 *
 * @return
 *   - ESP_OK on success.
 *   - Appropriate ESP_ERR code on failure, with error details logged.
 */
esp_err_t priv_i2c_write_byte(uint8_t data, uint8_t i2c_bus,
                              uint8_t i2c_address, const char *tag);

/**
 * @brief Read multiple bytes from a specific I2C device.
 *
 * This function reads a specified number of bytes from a designated I2C device
 * using the specified I2C bus. It supports reading single or multiple bytes
 * with ACK/NACK handling for proper I2C communication.
 *
 * @note This function does not perform semaphore checks and is intended for
 *       internal use.
 *
 * @param[out] data Pointer to the buffer where read data will be stored.
 * @param[in] len The number of bytes to read from the I2C device.
 * @param[in] i2c_bus The I2C bus number to communicate over.
 * @param[in] i2c_address The 7-bit I2C address of the target device.
 * @param[in] tag The tag for logging errors.
 *
 * @return
 *   - ESP_OK on success.
 *   - Appropriate ESP_ERR code on failure, with error details logged.
 */
esp_err_t priv_i2c_read_bytes(uint8_t *data, size_t len, uint8_t i2c_bus,
                              uint8_t i2c_address, const char *tag);

/* New function declarations */

/**
 * @brief Write a byte to a specific register of an I2C device.
 *
 * @param[in] reg_addr The register address to write to.
 * @param[in] data The data byte to write to the register.
 * @param[in] i2c_bus The I2C bus number to communicate over.
 * @param[in] i2c_address The 7-bit I2C address of the target device.
 * @param[in] tag The tag for logging errors.
 *
 * @return
 *   - ESP_OK on success.
 *   - Appropriate ESP_ERR code on failure, with error details logged.
 */
esp_err_t priv_i2c_write_reg_byte(uint8_t reg_addr, uint8_t data,
                                  uint8_t i2c_bus, uint8_t i2c_address,
                                  const char *tag);

/**
 * @brief Read multiple bytes starting from a specific register of an I2C device.
 *
 * @param[in] reg_addr The starting register address to read from.
 * @param[out] data Pointer to the buffer where read data will be stored.
 * @param[in] len The number of bytes to read.
 * @param[in] i2c_bus The I2C bus number to communicate over.
 * @param[in] i2c_address The 7-bit I2C address of the target device.
 * @param[in] tag The tag for logging errors.
 *
 * @return
 *   - ESP_OK on success.
 *   - Appropriate ESP_ERR code on failure, with error details logged.
 */
esp_err_t priv_i2c_read_reg_bytes(uint8_t reg_addr, uint8_t *data, size_t len,
                                  uint8_t i2c_bus, uint8_t i2c_address,
                                  const char *tag);

#endif /* TOPOROBO_I2C_H */

