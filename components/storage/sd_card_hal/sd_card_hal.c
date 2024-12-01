/* components/storage/sd_card_hal/sd_card_hal.c */

/* TODO: Test this */

#include "sd_card_hal.h"
#include "common/spi.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"

/* Constants ******************************************************************/

const char             *sd_card_tag                  = "SD_CARD";
const char             *sd_card_mount                = "/sdcard";
const uint8_t           sd_card_cs_io                = GPIO_NUM_5;
const uint8_t           sd_card_mosi_io              = GPIO_NUM_23;
const uint8_t           sd_card_miso_io              = GPIO_NUM_19;
const uint8_t           sd_card_clk_io               = GPIO_NUM_14;
const uint32_t          sd_card_spi_freq_hz          = 1000000; /* 1 MHz SPI frequency */
const spi_host_device_t sd_card_spi_host             = SPI2_HOST;
const uint8_t           sd_card_max_files            = 5;
const uint32_t          sd_card_allocation_unit_size = 16 * 1024;

/* Globals (Static) ***********************************************************/

static spi_device_handle_t sd_card_spi_handle = NULL;

/* Public Functions ***********************************************************/

esp_err_t sd_card_init(void)
{
  ESP_LOGI(sd_card_tag, "Starting SD card initialization...");

  /* Initialize the SPI bus */
  esp_err_t ret = priv_spi_init(sd_card_spi_host, sd_card_clk_io, sd_card_mosi_io,
                                sd_card_miso_io, sd_card_cs_io, &sd_card_spi_handle,
                                sd_card_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(sd_card_tag, "SPI initialization failed: %s", esp_err_to_name(ret));
    return ESP_FAIL;
  }

  /* SD card mount configuration */
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files              = sd_card_max_files,
    .allocation_unit_size   = sd_card_allocation_unit_size,
  };

  sdmmc_card_t *card;
  sdmmc_host_t  host = SDSPI_HOST_DEFAULT();
  host.slot          = sd_card_spi_host;

  sdspi_device_config_t device_config = {
    .gpio_cs = sd_card_cs_io,
    .host_id = host.slot,
  };

  /* Mount the SD card */
  ret = esp_vfs_fat_sdspi_mount(sd_card_mount, &host, &device_config, &mount_config, &card);
  if (ret != ESP_OK) {
    ESP_LOGE(sd_card_tag, "Failed to mount FATFS: %s", esp_err_to_name(ret));
    return ESP_FAIL;
  }

  ESP_LOGI(sd_card_tag, "SD card mounted at %s", sd_card_mount);
  return ESP_OK;
}

