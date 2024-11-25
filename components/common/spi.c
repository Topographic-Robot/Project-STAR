/* components/common/spi.c */

#include "common/spi.h"
#include <string.h>
#include "driver/spi_master.h"
#include "esp_log.h"

/* Constants ******************************************************************/

/**
 * @brief Timeout in RTOS ticks used for SPI bus operations.
 *
 * Defines the maximum time, expressed in RTOS ticks, that an SPI transaction can take before
 * timing out. This timeout helps to prevent indefinite blocking in case of communication
 * errors or issues with the SPI device. The value should be adjusted based on the expected
 * response time of the SPI device and the timing constraints of the overall system.
 */
const uint32_t spi_timeout_ticks = pdMS_TO_TICKS(1000);

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
                        const char *tag)
{
  /* SPI bus configuration */
  spi_bus_config_t buscfg = {
    .miso_io_num     = miso,
    .mosi_io_num     = mosi,
    .sclk_io_num     = sclk,
    .quadwp_io_num   = -1,   /* Not used */
    .quadhd_io_num   = -1,   /* Not used */
    .max_transfer_sz = 4000, /* Maximum transfer size; adjust as needed */
  };

  /* SPI device interface configuration */
  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 4 * 1000 * 1000, /* 4 MHz clock speed */
    .mode           = 0,               /* SPI mode 0 */
    .spics_io_num   = cs,              /* Chip select pin */
    .queue_size     = 7,               /* Transaction queue size */
    .pre_cb         = NULL,            /* Pre-transfer callback; not used */
  };

  /* Initialize the SPI bus */
  esp_err_t ret = spi_bus_initialize(host_id, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    ESP_LOGE(tag, "SPI bus initialization failed: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Attach the device to the SPI bus */
  ret = spi_bus_add_device(host_id, &devcfg, handle_out);
  if (ret != ESP_OK) {
    ESP_LOGE(tag, "Failed to add device to SPI bus: %s", esp_err_to_name(ret));
    return ret;
  }

  return ESP_OK;
}

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
esp_err_t priv_spi_write_byte(spi_device_handle_t handle, uint8_t data, const char *tag)
{
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); /* Zero out the transaction structure */
  t.length    = 8;          /* Length is 8 bits (1 byte) */
  t.tx_buffer = &data;      /* Data to be sent */

  esp_err_t ret = spi_device_transmit(handle, &t); /* Transmit the data */
  if (ret != ESP_OK) {
    ESP_LOGE(tag, "SPI write byte failed: %s", esp_err_to_name(ret));
    return ret;
  }

  return ESP_OK;
}


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
                                  uint8_t data, const char *tag)
{

  esp_err_t ret = priv_spi_write_byte(handle, reg_addr, tag);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = priv_spi_write_byte(handle, data, tag);
  return ret;
}
