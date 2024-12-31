/* components/storage/sd_card_hal/sd_card_hal.c */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

/* Constants ******************************************************************/
const char             *sd_card_tag                  = "SD_CARD";
const char             *sd_card_mount                = "/sdcard";
const uint8_t           sd_card_cs                   = GPIO_NUM_5;
const uint8_t           sd_card_data_to_card         = GPIO_NUM_23;
const uint8_t           sd_card_data_from_card       = GPIO_NUM_19;
const uint8_t           sd_card_clk                  = GPIO_NUM_14;
const uint32_t          sd_card_spi_freq_hz          = 1000000;     /* 1 MHz SPI frequency */
const spi_host_device_t sd_card_spi_host             = SPI2_HOST;
const uint8_t           sd_card_max_files            = 5;
const uint32_t          sd_card_allocation_unit_size = 16 * 1024;
const uint32_t          sd_card_max_transfer_sz      = 4092;        /* Default size in Bytes */

/* Public Functions ***********************************************************/

esp_err_t sd_card_init(void) 
{
  esp_err_t ret;

  ESP_LOGI(sd_card_tag, "Initializing SD card");

  /* Configure the SPI bus */
  spi_bus_config_t bus_cfg = {
    .mosi_io_num     = sd_card_data_to_card,
    .miso_io_num     = sd_card_data_from_card,
    .sclk_io_num     = sd_card_clk,
    .quadwp_io_num   = -1,
    .quadhd_io_num   = -1,
    .max_transfer_sz = sd_card_max_transfer_sz,
  };

  ret = spi_bus_initialize(sd_card_spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    ESP_LOGE(sd_card_tag, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Configure the SD card slot */
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot         = sd_card_spi_host;
  host.max_freq_khz = sd_card_spi_freq_hz / 1000;

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs               = sd_card_cs;
  slot_config.host_id               = sd_card_spi_host;

  /* Mount the filesystem */
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files              = sd_card_max_files,
    .allocation_unit_size   = sd_card_allocation_unit_size,
  };

  sdmmc_card_t *card;
  ret = esp_vfs_fat_sdspi_mount(sd_card_mount, &host, &slot_config, &mount_config, &card);

  if (ret != ESP_OK) {
    ESP_LOGE(sd_card_tag, "Failed to mount filesystem: %s", esp_err_to_name(ret));
    spi_bus_free(sd_card_spi_host);
    return ret;
  }

  /* Print card info */
  sdmmc_card_print_info(stdout, card);
  ESP_LOGI(sd_card_tag, "SD card mounted at %s", sd_card_mount);

  return ESP_OK;
}


