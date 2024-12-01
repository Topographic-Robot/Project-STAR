/* components/common/spi.c */

#include "common/spi.h"
#include <string.h>
#include "driver/spi_master.h"
#include "esp_log.h"

/* Constants ******************************************************************/

const uint32_t spi_timeout_ticks = pdMS_TO_TICKS(1000);

/* Private Functions **********************************************************/

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
