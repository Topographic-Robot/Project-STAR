/* components/pstar_sdmmc_hal/pstar_sdmmc_hal.c */

#include "pstar_sdmmc_hal.h"

#include "pstar_bus_config.h"
#include "pstar_bus_spi.h"
#include "pstar_pin_validator.h"

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"

#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdkconfig.h" /* Include sdkconfig for Kconfig defines */
#include "sdmmc_cmd.h"

static const char* TAG = "SDMMC HAL";

/* --- Internal Handle Structure --- */
typedef struct pstar_sdmmc_hal_dev_t {
  pstar_sdmmc_interface_mode_t interface_mode; /**< The active interface mode */
  char*                        mount_point;    /**< Filesystem mount point (copied) */
  sdmmc_card_t*                card;           /**< Pointer to card information */
  sdmmc_host_t                 host_config;    /**< SDMMC host configuration (used by both modes) */
  bool                         is_mounted;     /**< Flag indicating if the filesystem is mounted */
  /* Store SPI bus name only if SPI mode is used, for potential cleanup reference */
  char* spi_bus_name;
  /* Store the relevant slot config directly in the handle */
  sdspi_device_config_t spi_slot_config;
  sdmmc_slot_config_t   sdmmc_slot_config;
} pstar_sdmmc_hal_dev_t;

/* --- Helper Functions --- */

static esp_err_t
priv_sdmmc_mount_fs(pstar_sdmmc_hal_handle_t handle, int max_files, bool format_if_mount_failed)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mount_point, ESP_ERR_INVALID_STATE, TAG, "Mount point not set");

  esp_err_t                        ret;
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = format_if_mount_failed,
    .max_files              = max_files,
    .allocation_unit_size   = 16 * 1024 /* Sensible default */
  };

  ESP_LOGI(TAG, "Mounting filesystem on '%s'", handle->mount_point);

  if (handle->interface_mode == PSTAR_SDMMC_MODE_SPI) {
    ret =
      esp_vfs_fat_sdspi_mount(handle->mount_point,
                              &handle->host_config,
                              &handle->spi_slot_config, /* Use the config stored in HAL handle */
                              &mount_config,
                              &handle->card);
  } else {
    ret = esp_vfs_fat_sdmmc_mount(
      handle->mount_point,
      &handle->host_config,
      &handle->sdmmc_slot_config, /* Use SDMMC slot config from HAL handle */
      &mount_config,
      &handle->card);
  }

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL && format_if_mount_failed) {
      ESP_LOGW(TAG,
               "Failed to mount filesystem. Formatting may be attempted if enabled in config.");
    } else if (ret == ESP_ERR_NO_MEM) {
      ESP_LOGE(TAG, "Failed to mount filesystem: Not enough memory");
    } else {
      ESP_LOGE(TAG,
               "Failed to initialize the card (%s). Check wiring/pull-ups.",
               esp_err_to_name(ret));
    }
    handle->is_mounted = false;
    return ret;
  }

  ESP_LOGI(TAG, "Filesystem mounted on '%s'", handle->mount_point);
  handle->is_mounted = true;
  sdmmc_card_print_info(stdout, handle->card);
  return ESP_OK;
}

static esp_err_t priv_sdmmc_unmount_fs(pstar_sdmmc_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Handle is NULL");

  if (!handle->is_mounted) {
    ESP_LOGD(TAG, "Filesystem not mounted, skipping unmount.");
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Unmounting filesystem from '%s'", handle->mount_point);
  esp_err_t ret = esp_vfs_fat_sdcard_unmount(handle->mount_point, handle->card);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to unmount filesystem: %s", esp_err_to_name(ret));
  } else {
    handle->is_mounted = false;
    ESP_LOGI(TAG, "Filesystem unmounted successfully.");
  }
  return ret;
}

/* --- Public Function Implementations --- */

esp_err_t pstar_sdmmc_hal_init(const pstar_bus_manager_t*      manager, /* Needed for SPI */
                               const pstar_sdmmc_hal_config_t* config,
                               pstar_sdmmc_hal_handle_t*       out_handle)
{
  ESP_RETURN_ON_FALSE(config && config->mount_point && out_handle,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Config, mount_point, or out_handle is NULL");
  *out_handle = NULL;

  pstar_sdmmc_hal_handle_t dev = (pstar_sdmmc_hal_handle_t)malloc(sizeof(pstar_sdmmc_hal_dev_t));
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Failed to allocate device handle");
  memset(dev, 0, sizeof(pstar_sdmmc_hal_dev_t));

  dev->interface_mode = config->interface_mode;
  dev->mount_point    = strdup(config->mount_point);
  if (!dev->mount_point) {
    ESP_LOGE(TAG, "Failed to duplicate mount point");
    free(dev);
    return ESP_ERR_NO_MEM;
  }
  dev->is_mounted = false;
  dev->card       = NULL;

  esp_err_t ret = ESP_OK;

  if (dev->interface_mode == PSTAR_SDMMC_MODE_SPI) {
    ESP_LOGI(TAG, "Initializing SD card in SPI Mode...");
    ESP_RETURN_ON_FALSE(manager && config->mode_config.spi.spi_bus_name,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "SPI mode requires valid manager and spi_bus_name");

    pstar_bus_config_t* bus_config =
      pstar_bus_manager_find_bus(manager, config->mode_config.spi.spi_bus_name);
    ESP_GOTO_ON_FALSE(bus_config,
                      ESP_ERR_NOT_FOUND,
                      init_fail,
                      TAG,
                      "SPI device '%s' not found",
                      config->mode_config.spi.spi_bus_name);
    ESP_GOTO_ON_FALSE(bus_config->initialized,
                      ESP_ERR_INVALID_STATE,
                      init_fail,
                      TAG,
                      "SPI device '%s' not initialized",
                      config->mode_config.spi.spi_bus_name);
    ESP_GOTO_ON_FALSE(bus_config->type == k_pstar_bus_type_spi,
                      ESP_ERR_INVALID_ARG,
                      init_fail,
                      TAG,
                      "Bus '%s' is not SPI",
                      config->mode_config.spi.spi_bus_name);

    dev->spi_bus_name = strdup(config->mode_config.spi.spi_bus_name);
    if (!dev->spi_bus_name) {
      ESP_LOGE(TAG, "Failed to duplicate SPI bus name");
      ret = ESP_ERR_NO_MEM;
      goto init_fail;
    }

    sdmmc_host_t host_cfg = SDSPI_HOST_DEFAULT();
    host_cfg.slot         = bus_config->proto.spi.host;
    dev->host_config      = host_cfg;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.host_id               = bus_config->proto.spi.host;
    slot_config.gpio_cs               = bus_config->proto.spi.dev_cfg.spics_io_num;

    dev->card = (sdmmc_card_t*)malloc(sizeof(sdmmc_card_t));
    ESP_GOTO_ON_FALSE(dev->card, ESP_ERR_NO_MEM, init_fail, TAG, "Failed to allocate card struct");
    memset(dev->card, 0, sizeof(sdmmc_card_t));
    dev->spi_slot_config = slot_config;

  } else {
    ESP_LOGI(TAG,
             "Initializing SD card in SDMMC %d-bit Mode...",
             (dev->interface_mode == PSTAR_SDMMC_MODE_SDMMC_4BIT) ? 4 : 1);

    sdmmc_host_t host_cfg = SDMMC_HOST_DEFAULT();
    host_cfg.flags   = (dev->interface_mode == PSTAR_SDMMC_MODE_SDMMC_4BIT) ? SDMMC_HOST_FLAG_4BIT
                                                                            : SDMMC_HOST_FLAG_1BIT;
    dev->host_config = host_cfg;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk                 = config->mode_config.sdmmc.clk_pin;
    slot_config.cmd                 = config->mode_config.sdmmc.cmd_pin;
    slot_config.d0                  = config->mode_config.sdmmc.d0_pin;
    if (dev->interface_mode == PSTAR_SDMMC_MODE_SDMMC_4BIT) {
      slot_config.d1    = config->mode_config.sdmmc.d1_pin;
      slot_config.d2    = config->mode_config.sdmmc.d2_pin;
      slot_config.d3    = config->mode_config.sdmmc.d3_pin;
      slot_config.width = 4;
    } else {
      slot_config.width = 1;
    }

    dev->card = (sdmmc_card_t*)malloc(sizeof(sdmmc_card_t));
    ESP_GOTO_ON_FALSE(dev->card, ESP_ERR_NO_MEM, init_fail, TAG, "Failed to allocate card struct");
    memset(dev->card, 0, sizeof(sdmmc_card_t));
    dev->sdmmc_slot_config = slot_config;
  }

  ret = priv_sdmmc_mount_fs(dev, config->max_files, config->format_if_mount_failed);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to mount SD card filesystem: %s", esp_err_to_name(ret));
    goto init_fail;
  }

  ESP_LOGI(TAG, "SDMMC HAL Initialized Successfully");
  *out_handle = dev;
  return ESP_OK;

init_fail:
  ESP_LOGE(TAG, "SDMMC HAL initialization failed");
  if (dev) {
    if (dev->card) {
      free(dev->card);
    }
    if (dev->mount_point) {
      free(dev->mount_point);
    }
    if (dev->spi_bus_name) {
      free(dev->spi_bus_name);
    }
    free(dev);
  }
  *out_handle = NULL;
  return ret;
}

esp_err_t pstar_sdmmc_hal_deinit(pstar_sdmmc_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGI(TAG, "Deinitializing SDMMC HAL...");

  esp_err_t ret = priv_sdmmc_unmount_fs(handle);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Filesystem unmount failed during deinit, continuing cleanup...");
  }

  if (handle->interface_mode == PSTAR_SDMMC_MODE_SPI) {
    ESP_LOGD(TAG,
             "SPI host deinit handled by bus manager when device '%s' is removed.",
             handle->spi_bus_name ? handle->spi_bus_name : "UNKNOWN");
  } else {
    esp_err_t deinit_ret = sdmmc_host_deinit();
    if (deinit_ret != ESP_OK) {
      ESP_LOGE(TAG, "sdmmc_host_deinit failed: %s", esp_err_to_name(deinit_ret));
      if (ret == ESP_OK)
        ret = deinit_ret;
    }
  }

  if (handle->card) {
    free(handle->card);
  }
  if (handle->mount_point) {
    free(handle->mount_point);
  }
  if (handle->spi_bus_name) {
    free(handle->spi_bus_name);
  }
  free(handle);

  return ret;
}

esp_err_t pstar_sdmmc_hal_get_card_info(pstar_sdmmc_hal_handle_t handle,
                                        sdmmc_card_t**           out_card_info)
{
  ESP_RETURN_ON_FALSE(handle && out_card_info,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Handle or out_card_info is NULL");
  ESP_RETURN_ON_FALSE(handle->card, ESP_ERR_INVALID_STATE, TAG, "Card not initialized");

  *out_card_info = handle->card;
  return ESP_OK;
}

const char* pstar_sdmmc_hal_get_mount_point(pstar_sdmmc_hal_handle_t handle)
{
  if (handle) {
    return handle->mount_point;
  }
  return NULL;
}

bool pstar_sdmmc_hal_is_mounted(pstar_sdmmc_hal_handle_t handle)
{
  return (handle != NULL && handle->is_mounted);
}

/* --- Convenience Creation Functions --- */

#if CONFIG_PSTAR_KCONFIG_SDMMC_ENABLED
esp_err_t pstar_sdmmc_hal_create_kconfig_default(pstar_bus_manager_t*      manager,
                                                 pstar_sdmmc_hal_handle_t* out_handle)
{
  ESP_RETURN_ON_FALSE(manager && out_handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

  *out_handle   = NULL;
  esp_err_t ret = ESP_OK;

  pstar_sdmmc_hal_config_t hal_config = {
    .mount_point = CONFIG_PSTAR_KCONFIG_SDMMC_MOUNT_POINT,
    .max_files   = CONFIG_PSTAR_KCONFIG_SDMMC_MAX_FILES,
#ifdef CONFIG_PSTAR_KCONFIG_SDMMC_FORMAT_IF_FAIL
    .format_if_mount_failed = true,
#else
    .format_if_mount_failed = false,
#endif
  };

#if CONFIG_PSTAR_KCONFIG_SDMMC_MODE_SPI
  hal_config.interface_mode               = PSTAR_SDMMC_MODE_SPI;
  hal_config.mode_config.spi.spi_bus_name = CONFIG_PSTAR_KCONFIG_SDMMC_SPI_BUS_NAME;
  const char*         bus_name            = CONFIG_PSTAR_KCONFIG_SDMMC_SPI_BUS_NAME;
  bool                config_created      = false;
  bool                config_added        = false;
  pstar_bus_config_t* sdmmc_spi_config    = NULL;

  ESP_LOGI(TAG, "Creating SDMMC HAL (SPI Mode) with KConfig defaults");

  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = CONFIG_PSTAR_KCONFIG_SDMMC_SPI_FREQ_HZ,
    .mode           = 0,
    .spics_io_num   = CONFIG_PSTAR_KCONFIG_SDMMC_CS_PIN,
    .queue_size     = 7,
    .flags          = 0,
  };
  sdmmc_spi_config = pstar_bus_config_create_spi_device(bus_name,
                                                        CONFIG_PSTAR_KCONFIG_SDMMC_SPI_HOST,
                                                        CONFIG_PSTAR_KCONFIG_SDMMC_POSI_PIN,
                                                        CONFIG_PSTAR_KCONFIG_SDMMC_PISO_PIN,
                                                        CONFIG_PSTAR_KCONFIG_SDMMC_SCLK_PIN,
                                                        SPI_DMA_CH_AUTO,
                                                        &devcfg);
  ESP_GOTO_ON_FALSE(sdmmc_spi_config != NULL,
                    ESP_ERR_NO_MEM,
                    cleanup_spi,
                    TAG,
                    "Failed create SPI config");
  config_created = true;

  ret = pstar_bus_manager_add_bus(manager, sdmmc_spi_config);
  if (ret == ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "Bus config '%s' already exists, finding...", bus_name);
    sdmmc_spi_config = pstar_bus_manager_find_bus(manager, bus_name);
    ESP_GOTO_ON_FALSE(sdmmc_spi_config,
                      ESP_FAIL,
                      cleanup_spi,
                      TAG,
                      "Failed find existing bus '%s'",
                      bus_name);
    config_added   = true;
    config_created = false;
    ret            = ESP_OK;
  } else {
    ESP_GOTO_ON_ERROR(ret, cleanup_spi, TAG, "Failed add SDMMC config: %s", esp_err_to_name(ret));
    config_added   = true;
    config_created = false;
  }

  if (!sdmmc_spi_config->initialized) {
    ret = pstar_bus_config_init(sdmmc_spi_config, manager);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup_spi,
                      TAG,
                      "Failed to initialize bus hardware for '%s': %s",
                      bus_name,
                      esp_err_to_name(ret));
  }

  ret = pstar_sdmmc_hal_init(manager, &hal_config, out_handle);
  ESP_GOTO_ON_ERROR(ret, cleanup_spi, TAG, "Failed init SDMMC HAL (SPI): %s", esp_err_to_name(ret));

  ESP_LOGI(TAG, "SDMMC HAL (SPI) created successfully with KConfig defaults");
  return ESP_OK;

cleanup_spi:
  if (config_created && !config_added) {
    pstar_bus_config_destroy(sdmmc_spi_config);
  }
  goto common_cleanup; /* Jump to common cleanup */

#elif CONFIG_PSTAR_KCONFIG_SDMMC_MODE_SDMMC_1BIT
  hal_config.interface_mode            = PSTAR_SDMMC_MODE_SDMMC_1BIT;
  hal_config.mode_config.sdmmc.clk_pin = CONFIG_PSTAR_KCONFIG_SDMMC_CLK_PIN;
  hal_config.mode_config.sdmmc.cmd_pin = CONFIG_PSTAR_KCONFIG_SDMMC_CMD_PIN;
  hal_config.mode_config.sdmmc.d0_pin  = CONFIG_PSTAR_KCONFIG_SDMMC_D0_PIN;
  hal_config.mode_config.sdmmc.d1_pin  = -1;
  hal_config.mode_config.sdmmc.d2_pin  = -1;
  hal_config.mode_config.sdmmc.d3_pin  = -1;
  ESP_LOGI(TAG, "Creating SDMMC HAL (SDMMC 1-bit Mode) with KConfig defaults");
  ret = pstar_sdmmc_hal_init(NULL, &hal_config, out_handle);
  ESP_GOTO_ON_ERROR(ret,
                    common_cleanup,
                    TAG,
                    "Failed init SDMMC HAL (1-bit): %s",
                    esp_err_to_name(ret));
  ESP_LOGI(TAG, "SDMMC HAL (1-bit) created successfully with KConfig defaults");
  return ESP_OK;

#elif CONFIG_PSTAR_KCONFIG_SDMMC_MODE_SDMMC_4BIT
  hal_config.interface_mode            = PSTAR_SDMMC_MODE_SDMMC_4BIT;
  hal_config.mode_config.sdmmc.clk_pin = CONFIG_PSTAR_KCONFIG_SDMMC_CLK_PIN;
  hal_config.mode_config.sdmmc.cmd_pin = CONFIG_PSTAR_KCONFIG_SDMMC_CMD_PIN;
  hal_config.mode_config.sdmmc.d0_pin  = CONFIG_PSTAR_KCONFIG_SDMMC_D0_PIN;
  hal_config.mode_config.sdmmc.d1_pin  = CONFIG_PSTAR_KCONFIG_SDMMC_D1_PIN;
  hal_config.mode_config.sdmmc.d2_pin  = CONFIG_PSTAR_KCONFIG_SDMMC_D2_PIN;
  hal_config.mode_config.sdmmc.d3_pin  = CONFIG_PSTAR_KCONFIG_SDMMC_D3_PIN;
  ESP_LOGI(TAG, "Creating SDMMC HAL (SDMMC 4-bit Mode) with KConfig defaults");
  ret = pstar_sdmmc_hal_init(NULL, &hal_config, out_handle);
  ESP_GOTO_ON_ERROR(ret,
                    common_cleanup,
                    TAG,
                    "Failed init SDMMC HAL (4-bit): %s",
                    esp_err_to_name(ret));
  ESP_LOGI(TAG, "SDMMC HAL (4-bit) created successfully with KConfig defaults");
  return ESP_OK;

#else
#error "No valid SDMMC interface mode selected in Kconfig!"
  ret = ESP_ERR_INVALID_STATE;
  goto common_cleanup;
#endif

common_cleanup: /* Now reachable from all paths */
  *out_handle = NULL;
  return ret;
}
#else  /* CONFIG_PSTAR_KCONFIG_SDMMC_ENABLED */
/* Stub function if component disabled */
esp_err_t pstar_sdmmc_hal_create_kconfig_default(pstar_bus_manager_t*      manager,
                                                 pstar_sdmmc_hal_handle_t* out_handle)
{
  ESP_LOGE(TAG, "SDMMC HAL component is disabled in Kconfig.");
  if (out_handle)
    *out_handle = NULL;
  return ESP_ERR_NOT_SUPPORTED;
}
#endif /* CONFIG_PSTAR_KCONFIG_SDMMC_ENABLED */

esp_err_t pstar_sdmmc_hal_create_custom_spi(pstar_bus_manager_t*      manager,
                                            const char*               spi_bus_name,
                                            spi_host_device_t         host,
                                            int                       clk_speed_hz,
                                            gpio_num_t                posi_pin,
                                            gpio_num_t                piso_pin,
                                            gpio_num_t                sclk_pin,
                                            gpio_num_t                cs_pin,
                                            const char*               mount_point,
                                            int                       max_files,
                                            bool                      format_if_mount_failed,
                                            pstar_sdmmc_hal_handle_t* out_handle)
{
  ESP_RETURN_ON_FALSE(manager && spi_bus_name && mount_point && out_handle,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid arguments for custom SPI");

  *out_handle                          = NULL;
  esp_err_t           ret              = ESP_OK;
  bool                config_created   = false;
  bool                config_added     = false;
  pstar_bus_config_t* sdmmc_spi_config = NULL;

  ESP_LOGI(TAG, "Creating SDMMC HAL (SPI Mode) with custom config for bus '%s'", spi_bus_name);

  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = clk_speed_hz,
    .mode           = 0,
    .spics_io_num   = cs_pin,
    .queue_size     = 7,
    .flags          = 0,
  };
  sdmmc_spi_config = pstar_bus_config_create_spi_device(spi_bus_name,
                                                        host,
                                                        posi_pin,
                                                        piso_pin,
                                                        sclk_pin,
                                                        SPI_DMA_CH_AUTO,
                                                        &devcfg);
  ESP_GOTO_ON_FALSE(sdmmc_spi_config != NULL,
                    ESP_ERR_NO_MEM,
                    cleanup_custom_spi,
                    TAG,
                    "Failed create custom SPI config");
  config_created = true;

  ret = pstar_bus_manager_add_bus(manager, sdmmc_spi_config);
  if (ret == ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "Custom bus config '%s' already exists, finding...", spi_bus_name);
    sdmmc_spi_config = pstar_bus_manager_find_bus(manager, spi_bus_name);
    ESP_GOTO_ON_FALSE(sdmmc_spi_config,
                      ESP_FAIL,
                      cleanup_custom_spi,
                      TAG,
                      "Failed find existing custom bus '%s'",
                      spi_bus_name);
    config_added   = true;
    config_created = false;
    ret            = ESP_OK;
  } else {
    ESP_GOTO_ON_ERROR(ret,
                      cleanup_custom_spi,
                      TAG,
                      "Failed add custom SDMMC config: %s",
                      esp_err_to_name(ret));
    config_added   = true;
    config_created = false;
  }

  if (!sdmmc_spi_config->initialized) {
    ret = pstar_bus_config_init(sdmmc_spi_config, manager);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup_custom_spi,
                      TAG,
                      "Failed to initialize bus hardware for '%s': %s",
                      spi_bus_name,
                      esp_err_to_name(ret));
  }

  pstar_sdmmc_hal_config_t hal_config = {
    .interface_mode               = PSTAR_SDMMC_MODE_SPI,
    .mount_point                  = mount_point,
    .max_files                    = max_files,
    .format_if_mount_failed       = format_if_mount_failed,
    .mode_config.spi.spi_bus_name = spi_bus_name,
  };
  ret = pstar_sdmmc_hal_init(manager, &hal_config, out_handle);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup_custom_spi,
                    TAG,
                    "Failed init custom SDMMC HAL (SPI): %s",
                    esp_err_to_name(ret));

  ESP_LOGI(TAG, "SDMMC HAL (Custom SPI) created successfully for bus '%s'", spi_bus_name);
  return ESP_OK;

cleanup_custom_spi:
  if (config_created && !config_added) {
    pstar_bus_config_destroy(sdmmc_spi_config);
  }
  *out_handle = NULL;
  return ret;
}

esp_err_t pstar_sdmmc_hal_create_custom_sdmmc(bool                      use_1_bit_mode,
                                              gpio_num_t                clk_pin,
                                              gpio_num_t                cmd_pin,
                                              gpio_num_t                d0_pin,
                                              gpio_num_t                d1_pin,
                                              gpio_num_t                d2_pin,
                                              gpio_num_t                d3_pin,
                                              const char*               mount_point,
                                              int                       max_files,
                                              bool                      format_if_mount_failed,
                                              pstar_sdmmc_hal_handle_t* out_handle)
{
  ESP_RETURN_ON_FALSE(mount_point && out_handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

  *out_handle = NULL;
  ESP_LOGI(TAG,
           "Creating SDMMC HAL (SDMMC %d-bit Mode) with custom pin config",
           use_1_bit_mode ? 1 : 4);

  pstar_sdmmc_hal_config_t hal_config = {
    .interface_mode = use_1_bit_mode ? PSTAR_SDMMC_MODE_SDMMC_1BIT : PSTAR_SDMMC_MODE_SDMMC_4BIT,
    .mount_point    = mount_point,
    .max_files      = max_files,
    .format_if_mount_failed = format_if_mount_failed,
  };
  hal_config.mode_config.sdmmc.clk_pin = clk_pin;
  hal_config.mode_config.sdmmc.cmd_pin = cmd_pin;
  hal_config.mode_config.sdmmc.d0_pin  = d0_pin;
  hal_config.mode_config.sdmmc.d1_pin  = use_1_bit_mode ? -1 : d1_pin;
  hal_config.mode_config.sdmmc.d2_pin  = use_1_bit_mode ? -1 : d2_pin;
  hal_config.mode_config.sdmmc.d3_pin  = use_1_bit_mode ? -1 : d3_pin;

  esp_err_t ret = pstar_sdmmc_hal_init(NULL, &hal_config, out_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG,
             "Failed init custom SDMMC HAL (%d-bit): %s",
             use_1_bit_mode ? 1 : 4,
             esp_err_to_name(ret));
    *out_handle = NULL;
    return ret;
  }

  ESP_LOGI(TAG, "SDMMC HAL (Custom SDMMC %d-bit) created successfully", use_1_bit_mode ? 1 : 4);
  return ESP_OK;
}

/* --- Pin Registration Functions --- */

esp_err_t pstar_sdmmc_register_kconfig_pins(void)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_SDMMC_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering SDMMC HAL pins with validator (from KConfig)...");

#if CONFIG_PSTAR_KCONFIG_SDMMC_MODE_SPI
  ESP_LOGD(TAG, "Registering SPI pins for SDMMC HAL");
  ret = pstar_sdmmc_register_custom_spi_pins(CONFIG_PSTAR_KCONFIG_SDMMC_POSI_PIN,
                                             CONFIG_PSTAR_KCONFIG_SDMMC_PISO_PIN,
                                             CONFIG_PSTAR_KCONFIG_SDMMC_SCLK_PIN,
                                             CONFIG_PSTAR_KCONFIG_SDMMC_CS_PIN);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register SDMMC KConfig SPI pins");

#elif CONFIG_PSTAR_KCONFIG_SDMMC_MODE_SDMMC_1BIT
  ESP_LOGD(TAG, "Registering SDMMC 1-bit pins for SDMMC HAL");
  ret = pstar_sdmmc_register_custom_sdmmc_pins(true, /* 1-bit mode */
                                               CONFIG_PSTAR_KCONFIG_SDMMC_CLK_PIN,
                                               CONFIG_PSTAR_KCONFIG_SDMMC_CMD_PIN,
                                               CONFIG_PSTAR_KCONFIG_SDMMC_D0_PIN,
                                               -1,
                                               -1,
                                               -1);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register SDMMC KConfig 1-bit pins");

#elif CONFIG_PSTAR_KCONFIG_SDMMC_MODE_SDMMC_4BIT
  ESP_LOGD(TAG, "Registering SDMMC 4-bit pins for SDMMC HAL");
  ret = pstar_sdmmc_register_custom_sdmmc_pins(false, /* 4-bit mode */
                                               CONFIG_PSTAR_KCONFIG_SDMMC_CLK_PIN,
                                               CONFIG_PSTAR_KCONFIG_SDMMC_CMD_PIN,
                                               CONFIG_PSTAR_KCONFIG_SDMMC_D0_PIN,
                                               CONFIG_PSTAR_KCONFIG_SDMMC_D1_PIN,
                                               CONFIG_PSTAR_KCONFIG_SDMMC_D2_PIN,
                                               CONFIG_PSTAR_KCONFIG_SDMMC_D3_PIN);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register SDMMC KConfig 4-bit pins");
#endif

  ESP_LOGI(TAG, "SDMMC KConfig pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator or SDMMC HAL disabled, skipping SDMMC KConfig pin registration.");
  return ESP_OK;
#endif
}

esp_err_t pstar_sdmmc_register_custom_spi_pins(int posi_pin, int piso_pin, int sclk_pin, int cs_pin)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG,
           "Registering custom SDMMC SPI pins (POSI:%d, PISO:%d, SCLK:%d, CS:%d) with validator...",
           posi_pin,
           piso_pin,
           sclk_pin,
           cs_pin);

  ret = pstar_register_pin(posi_pin, "SDMMC Custom SPI POSI", true);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom POSI pin (%d)", posi_pin);
  ret = pstar_register_pin(piso_pin, "SDMMC Custom SPI PISO", true);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom PISO pin (%d)", piso_pin);
  ret = pstar_register_pin(sclk_pin, "SDMMC Custom SPI SCLK", true);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom SCLK pin (%d)", sclk_pin);
  ret = pstar_register_pin(cs_pin, "SDMMC Custom SPI CS", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom CS pin (%d)", cs_pin);

  ESP_LOGI(TAG, "SDMMC custom SPI pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator disabled, skipping SDMMC custom SPI pin registration.");
  return ESP_OK;
#endif
}

esp_err_t pstar_sdmmc_register_custom_sdmmc_pins(bool use_1_bit_mode,
                                                 int  clk_pin,
                                                 int  cmd_pin,
                                                 int  d0_pin,
                                                 int  d1_pin,
                                                 int  d2_pin,
                                                 int  d3_pin)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(
    TAG,
    "Registering custom SDMMC %d-bit pins (CLK:%d, CMD:%d, D0:%d, D1:%d, D2:%d, D3:%d) with validator...",
    use_1_bit_mode ? 1 : 4,
    clk_pin,
    cmd_pin,
    d0_pin,
    d1_pin,
    d2_pin,
    d3_pin);

  ret = pstar_register_pin(clk_pin, "SDMMC Custom CLK", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom CLK pin (%d)", clk_pin);
  ret = pstar_register_pin(cmd_pin, "SDMMC Custom CMD", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom CMD pin (%d)", cmd_pin);
  ret = pstar_register_pin(d0_pin, "SDMMC Custom D0", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom D0 pin (%d)", d0_pin);

  if (!use_1_bit_mode) {
    ret = pstar_register_pin(d1_pin, "SDMMC Custom D1", false);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom D1 pin (%d)", d1_pin);
    ret = pstar_register_pin(d2_pin, "SDMMC Custom D2", false);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom D2 pin (%d)", d2_pin);
    ret = pstar_register_pin(d3_pin, "SDMMC Custom D3", false);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom D3 pin (%d)", d3_pin);
  }

  ESP_LOGI(TAG, "SDMMC custom pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator disabled, skipping SDMMC custom pin registration.");
  return ESP_OK;
#endif
}