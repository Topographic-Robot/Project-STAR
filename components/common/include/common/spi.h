/* components/common/include/common/spi.h */

#ifndef TOPOROBO_SPI_H
#define TOPOROBO_SPI_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"

/* Constants ******************************************************************/

/**
 * @brief Timeout in RTOS ticks used for SPI bus operations.
 *
 * Defines the maximum time, expressed in RTOS ticks, that an SPI transaction can take before
 * timing out. This timeout helps to prevent indefinite blocking in case of communication
 * errors or issues with the SPI device. The value should be adjusted based on the expected
 * response time of the SPI device and the timing constraints of the overall system.
 */
extern const uint32_t spi_timeout_ticks;

/* Private Functions **********************************************************/

/**
 * @brief Initializes the SPI bus and adds a device.
 *
 * This function initializes an SPI bus with the given configuration and attaches
 * a device to the bus.  It sets up the bus parameters, such as data lines (MOSI,
 * MISO, SCLK), chip select (CS) pin, clock speed, and mode, before adding the
 * specified device.  A handle for the SPI device is allocated and returned through
 * the handle_out parameter, which is used in subsequent transactions.
 *
 * **Logic and Flow:**
 * - Configure the SPI bus using the provided parameters.
 * - Add the SPI device to the bus, storing the handle in *handle_out.
 *
 * @param host_id The SPI host to use (e.g., SPI2_HOST).
 * @param sclk GPIO pin for SPI clock.
 * @param mosi GPIO pin for master-out-slave-in (MOSI).
 * @param miso GPIO pin for master-in-slave-out (MISO).
 * @param cs GPIO pin for chip select.
 * @param[out] handle_out Pointer to the SPI device handle. On successful
 *                         initialization, this will point to the newly created
 *                         handle.
 * @param[in] tag A string tag for error logging and identification.
 *
 * @return
 *    - ESP_OK on success
 *    - ESP_FAIL on failure
 */
esp_err_t priv_spi_init(spi_host_device_t host_id, uint8_t sclk, uint8_t mosi,
                        uint8_t miso, uint8_t cs, spi_device_handle_t* handle_out, 
                        const char *tag);

/**
 * @brief Write a single byte of data over the SPI bus.
 *
 * This function transmits a single byte of data to the SPI device specified
 * by the provided handle. It uses a simple SPI transaction to send the data.
 * Error checking is performed after the transmission to ensure successful
 * delivery of the byte.
 *
 * **Logic and Flow:**
 * - Create and initialize an SPI transaction structure.
 * - Set transaction length and transmit buffer to the provided data.
 * - Execute the SPI transaction using the provided handle.
 *
 * @param handle The handle of the SPI device to write to.
 * @param data The byte of data to write.
 * @param[in] tag A string tag for error logging and identification.
 *
 * @return
 *   - ESP_OK if data is successfully read.
 *   - ESP_FAIL or another error code if reading fails or times out.
 */
esp_err_t priv_spi_write_byte(spi_device_handle_t handle, uint8_t data, const char *tag);

/**
 * @brief Write a single byte to a specific register of an SPI device.
 *
 * This function writes a single byte of data to the specified register of the
 * SPI device associated with the provided handle.  It utilizes the
 * `priv_spi_write_byte` function internally to perform the SPI transaction,
 * ensuring the register address is sent before the data byte. It currently
 * assumes the OV7670 camera's register write mechanism.
 *
 * **Logic and Flow:**
 * - Write the register address using `priv_spi_write_byte`.
 * - Write the data to the register using `priv_spi_write_byte`.
 *
 * @param handle The handle of the SPI device to write to.
 * @param reg_addr The register address to write to.
 * @param data The data byte to write to the register.
 * @param[in] tag A string tag for error logging and identification.
 *
 * @return
 *   - ESP_OK if data is successfully written to the register.
 *   - ESP_FAIL or another error code if the operation fails.
 */
esp_err_t priv_spi_write_reg_byte(spi_device_handle_t handle, uint8_t reg_addr, 
                                  uint8_t data, const char *tag);

#endif /* TOPOROBO_SPI_H */
