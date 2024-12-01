/* components/storage/sd_card_hal.c */

/* TODO: Test this */

#include "sd_card_hal.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"

/* Constants ******************************************************************/

static const char *sd_card_tag   = "SD_CARD";
static const char *sd_card_mount = "/sdcard";

/* Public Functions ***********************************************************/

esp_err_t sd_card_init(void)
{
  ESP_LOGI(sd_card_tag, "Initializing SD card...");

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files              = 5,
    .allocation_unit_size   = 16 * 1024,
  };

  sdmmc_card_t *card;
  const char    mount_point[] = "/sdcard";
  sdmmc_host_t  host        = SDSPI_HOST_DEFAULT();
  sdspi_device_config_t device_config = {
    .gpio_cs = GPIO_NUM_5, /* TODO: Verify this */
    .host_id = host.slot,
  };

  esp_err_t ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &device_config, &mount_config, &card);

  if (ret != ESP_OK) {
    ESP_LOGE(sd_card_tag, "Failed to mount FATFS (%s)", esp_err_to_name(ret));
    return ESP_FAIL;
  }

  ESP_LOGI(sd_card_tag, "SD card mounted at %s", sd_card_mount);
  return ESP_OK;
}

