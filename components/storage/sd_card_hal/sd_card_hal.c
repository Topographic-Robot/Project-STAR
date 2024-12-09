/* components/storage/sd_card_hal/sd_card_hal.c */

/* TODO: Test this */

#include "sd_card_hal.h"
#include "common/spi.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "sdmmc_cmd.h"

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

/* Public Functions ***********************************************************/

esp_err_t sd_card_init(void)
{
    ESP_LOGI(sd_card_tag, "Starting SD card initialization...");

    /* Initialize the SPI bus */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = sd_card_mosi_io,
        .miso_io_num     = sd_card_miso_io,
        .sclk_io_num     = sd_card_clk_io,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret = spi_bus_initialize(sd_card_spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(sd_card_tag, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
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

    sdspi_device_config_t device_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    device_config.gpio_cs = sd_card_cs_io;
    device_config.host_id = host.slot;

    /* Mount the SD card */
    ret = esp_vfs_fat_sdspi_mount(sd_card_mount, &host, &device_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(sd_card_tag, "Failed to mount FATFS: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ESP_LOGI(sd_card_tag, "SD card mounted at %s", sd_card_mount);

    /* Optional: Print card info for debugging */
    sdmmc_card_print_info(stdout, card);

    return ESP_OK;
}

