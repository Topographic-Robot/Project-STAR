/* components/pstar_storage_hal/pstar_storage_sd_card_hal.c */

#include "pstar_storage_hal.h"
#include "pstar_storage_common.h"
#include "pstar_storage_spi_hal.h"
#include "pstar_storage_sdio_hal.h"

#include "pstar_bus_config.h"
#include "pstar_bus_event_types.h"
#include "pstar_bus_gpio.h"
#include "pstar_log_handler.h"
#include "pstar_pin_validator.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "sdmmc_cmd.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <stdatomic.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

/* Constants ******************************************************************/
#define SD_INTERFACE_MAX_ERRORS (3)
static const char* TAG = "SD Card HAL";

/* ISR/Task Functions - Forward Declarations **********************************/
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
static void IRAM_ATTR priv_sd_card_detection_isr(void* arg);
#endif
static void priv_sd_card_mount_task(void* arg);

/* Default pin configurations *************************************************/
static const sd_card_pin_config_t sd_card_default_pins = {
  /* GPIO pins */
  .gpio_det_pin =
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
    CONFIG_PSTAR_KCONFIG_SD_CARD_DET_GPIO,
#else
    -1,
#endif

  /* SPI pins */
  .spi_di_pin =
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
    CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_DI_GPIO,
#else
    -1,
#endif
  .spi_do_pin =
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
    CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_DO_GPIO,
#else
    -1,
#endif
  .spi_sclk_pin =
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
    CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_CLK_GPIO,
#else
    -1,
#endif
  .spi_cs_pin =
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
    CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_CS_GPIO,
#else
    -1,
#endif

  /* SDIO pins */
  .sdio_clk_pin =
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
    CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_CLK_GPIO,
#else
    -1,
#endif
  .sdio_cmd_pin =
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
    CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_CMD_GPIO,
#else
    -1,
#endif
  .sdio_d0_pin =
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
    CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_D0_GPIO,
#else
    -1,
#endif
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_4BIT_MODE
  .sdio_d1_pin =
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
    CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_D1_GPIO,
#else
    -1,
#endif
  .sdio_d2_pin =
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
    CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_D2_GPIO,
#else
    -1,
#endif
  .sdio_d3_pin =
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
    CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_D3_GPIO,
#else
    -1,
#endif
#else  /* Not 4BIT_MODE */
  .sdio_d1_pin = -1,
  .sdio_d2_pin = -1,
  .sdio_d3_pin = -1,
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_4BIT_MODE */
};

/* Public API Functions *******************************************************/

esp_err_t sd_card_mount(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL || sd_card->card == NULL) {
    log_error(TAG, "Mount Error", "Invalid SD card HAL pointer or card structure");
    return ESP_ERR_INVALID_ARG;
  }
  /* Check if already mounted */
  if (atomic_load(&sd_card->card_available)) {
    log_info(sd_card->tag, "Mount Info", "SD card already mounted");
    return ESP_OK;
  }
  /* Check if physically inserted */
  if (!storage_sd_card_is_inserted(sd_card)) {
    log_warn(sd_card->tag, "Mount Warning", "Cannot mount: SD card not inserted");
    return ESP_ERR_NOT_FOUND;
  }
  log_info(sd_card->tag,
           "Mount Started",
           "Mounting SD card at path: %s with %s interface",
           sd_card->mount_path,
           sd_card_interface_to_string(sd_card->current_interface));

  /* Validate mount path using the simpler validation function */
  if (!storage_path_is_safe_simple(sd_card->mount_path)) {
    log_error(sd_card->tag, "Mount Error", "Unsafe mount path: %s", sd_card->mount_path);
    return ESP_ERR_INVALID_ARG;
  }

  /* Ensure mount path directory exists */
  esp_err_t dir_result = storage_create_directory_if_needed(sd_card, sd_card->mount_path);
  if (dir_result != ESP_OK) {
    log_error(TAG,
              "Mount Error",
              "Failed to create or verify mount directory: %s",
              esp_err_to_name(dir_result));
    return dir_result;
  }

  /* Initialize FAT filesystem */
  esp_vfs_fat_mount_config_t mount_config = {
    .format_if_mount_failed = false,                         /* Don't format if mount fails */
    .max_files              = sd_card->max_files,            /* Maximum number of open files */
    .allocation_unit_size   = sd_card->allocation_unit_size, /* Allocation unit size */
  };

  /* Try to mount the SD card */
  sdmmc_card_t* card_out = NULL; /* Mount function will update this */
  esp_err_t     err;

  /* Slot config is only used for SDIO, pass NULL for SPI */
  sdmmc_slot_config_t* slot_config_ptr = NULL;
  sdmmc_slot_config_t  slot_config     = SDMMC_SLOT_CONFIG_DEFAULT();
  if (sd_card->current_interface == k_sd_interface_sdio) {
    slot_config.clk   = sd_card->pin_config.sdio_clk_pin;
    slot_config.cmd   = sd_card->pin_config.sdio_cmd_pin;
    slot_config.d0    = sd_card->pin_config.sdio_d0_pin;
    slot_config.width = sd_card->bus_width; /* Use current HAL bus width */

    if (sd_card->bus_width == k_sd_bus_width_4bit) {
      slot_config.d1 = sd_card->pin_config.sdio_d1_pin;
      slot_config.d2 = sd_card->pin_config.sdio_d2_pin;
      slot_config.d3 = sd_card->pin_config.sdio_d3_pin;
      slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    }
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
    slot_config.gpio_cd = sd_card->pin_config.gpio_det_pin;
#else
    slot_config.gpio_cd = SDMMC_SLOT_NO_CD;
#endif
    slot_config.gpio_wp = SDMMC_SLOT_NO_WP;
    slot_config_ptr     = &slot_config; /* Point to the local struct */
  }

  err = esp_vfs_fat_sdmmc_mount(sd_card->mount_path,
                                &sd_card->card->host,
                                slot_config_ptr,
                                &mount_config,
                                &card_out);

  /* Handle potential bus width issues specifically for SDIO */
  if (err != ESP_OK && sd_card->current_interface == k_sd_interface_sdio &&
      sd_card->bus_width == k_sd_bus_width_4bit) {
    /* If 4-bit mode failed, try falling back to 1-bit mode */
    log_warn(sd_card->tag,
             "Mount Warning",
             "Failed to mount SDIO with 4-bit bus width, trying 1-bit mode");

    /* Update the host and slot config for 1-bit */
    sd_card->card->host.flags &= ~SDMMC_HOST_FLAG_4BIT;
    sd_card->card->host.flags |= SDMMC_HOST_FLAG_1BIT;
    slot_config.width = 1; /* Use the locally defined slot_config struct */

    /* Try mounting again with 1-bit mode */
    err = esp_vfs_fat_sdmmc_mount(sd_card->mount_path,
                                  &sd_card->card->host,
                                  &slot_config, /* Use updated 1-bit slot config */
                                  &mount_config,
                                  &card_out);

    if (err == ESP_OK) {
      /* Mount succeeded with 1-bit mode, update the setting */
      sd_card->bus_width = k_sd_bus_width_1bit;
      log_info(sd_card->tag,
               "Mount Info",
               "Successfully mounted with 1-bit mode after 4-bit mode failed");
    }
  }

  if (err != ESP_OK) {
    /* If failed, log appropriate message */
    if (err == ESP_FAIL) {
      log_error(sd_card->tag, "Mount Error", "Failed to mount FAT filesystem on SD card");
    } else if (err == ESP_ERR_INVALID_STATE) {
      log_error(sd_card->tag, "Mount Error", "SD card already mounted");
    } else if (err == ESP_ERR_NO_MEM) {
      log_error(sd_card->tag, "Mount Error", "Memory allocation failed");
    } else {
      log_error(sd_card->tag,
                "Mount Error",
                "Failed to initialize SD card: %s",
                esp_err_to_name(err));
    }
    /* Clean up the potentially allocated card_out if mount failed */
    if (card_out != NULL) {
      free(card_out);
    }
    return err;
  }

  /* Save the card structure returned from the mount function */
  /* We already allocated sd_card->card, but mount might reallocate.
   * Free the old one IF it's different from the new one */
  if (sd_card->card != card_out) {
    if (sd_card->card != NULL) {
      free(sd_card->card);
    }
    sd_card->card = card_out; /* Use the one returned by mount */
  }

  /* Create logs directory if not exists */
  char logs_path[CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH];
  if (snprintf(logs_path, sizeof(logs_path), "%s/logs", sd_card->mount_path) >=
      (int)sizeof(logs_path)) {
    log_warn(sd_card->tag, "Mount Warning", "Log path truncated");
  }

  /* Create logs directory if needed - non-critical operation */
  esp_err_t logs_dir_result = storage_create_directory_if_needed(sd_card, logs_path);
  if (logs_dir_result != ESP_OK) {
    log_warn(sd_card->tag,
             "Mount Warning",
             "Failed to create logs directory: %s",
             esp_err_to_name(logs_dir_result));
    /* Continue anyway, this is not critical */
  }
  /* Keep stdout for card info print */
  /* Get and log SD card info */
  sdmmc_card_print_info(stdout, sd_card->card);
  log_info(sd_card->tag,
           "Mount Success",
           "SD card mounted at %s, %lluMB, Interface: %s, %s bus width",
           sd_card->mount_path,
           ((uint64_t)sd_card->card->csd.capacity * sd_card->card->csd.sector_size) / (1024 * 1024),
           sd_card_interface_to_string(sd_card->current_interface),
           storage_bus_width_to_string(sd_card->bus_width));

  /* Update state */
  atomic_store(&sd_card->card_available, true);

  /* Schedule performance measurement on the new card */
  sd_card->performance.measurement_needed = true;

  /* Notify listeners about availability */
  storage_notify_availability(sd_card, true);

  return ESP_OK;
}

esp_err_t sd_card_unmount(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    log_error(TAG, "Unmount Error", "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }
  /* Check if already unmounted */
  if (!atomic_load(&sd_card->card_available)) {
    log_info(sd_card->tag, "Unmount Info", "SD card already unmounted");
    return ESP_OK;
  }
  log_info(sd_card->tag,
           "Unmount Started",
           "Unmounting SD card from path: %s",
           sd_card->mount_path);

  /* Unmount the SD card */
  esp_err_t err = esp_vfs_fat_sdcard_unmount(sd_card->mount_path, sd_card->card);
  if (err != ESP_OK) {
    log_error(sd_card->tag, "Unmount Error", "Failed to unmount SD card: %s", esp_err_to_name(err));
    /* Even if unmount fails, mark as unavailable */
    atomic_store(&sd_card->card_available, false);
    storage_notify_availability(sd_card, false); /* Notify listeners */
    /* Don't nullify sd_card->card here, cleanup needs it */
    return err;
  }

  /* Card pointer is now invalid after successful unmount */
  /* Let cleanup handle freeing the card structure */
  /* sd_card->card = NULL; */

  /* Update state */
  atomic_store(&sd_card->card_available, false);

  /* Clean up the specific interface resources */
  if (sd_card->current_interface == k_sd_interface_spi && sd_card->card != NULL) {
    sdspi_host_remove_device(sd_card->card->host.slot);
    sdspi_host_deinit();
  } else if (sd_card->current_interface == k_sd_interface_sdio) {
    sdmmc_host_deinit();
  }
  sd_card->current_interface = k_sd_interface_none; /* Reset interface */

  /* Notify listeners about unavailability */
  storage_notify_availability(sd_card, false);

  log_info(sd_card->tag, "Unmount Success", "SD card unmounted successfully");
  return ESP_OK;
}

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
esp_err_t sd_card_setup_detection(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    log_error(TAG, "Detection Setup Error", "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if detection pin is valid */
  if (sd_card->pin_config.gpio_det_pin < 0) {
    log_warn(sd_card->tag,
             "Detection Setup Warning",
             "Card detection enabled but pin is invalid (-1). Skipping setup.");
    return ESP_OK; /* Not an error, just can't set up */
  }

  log_info(sd_card->tag, "Detection Setup", "Setting up card detection mechanism");

  /* Check if the card detect pin conflicts with SDIO D3 pin when using 4-bit mode */
  if (sd_card->bus_width == k_sd_bus_width_4bit && sd_card->pin_config.gpio_det_pin >= 0 &&
      sd_card->pin_config.gpio_det_pin == sd_card->pin_config.sdio_d3_pin) {
    log_warn(sd_card->tag,
             "Pin Conflict",
             "Card detect pin (GPIO %d) conflicts with SDIO D3 for 4-bit mode. "
             "Detection may be unreliable.",
             sd_card->pin_config.gpio_det_pin);
    /* Note: We proceed, but SDIO setup might force 1-bit mode later */
  }

  /* Register the Card Detect pin with the validator */
  esp_err_t reg_err = pin_validator_register_pin(sd_card->pin_config.gpio_det_pin,
                                                 "SD Card HAL",
                                                 "Card Detect",
                                                 false);
  if (reg_err != ESP_OK) {
    log_error(sd_card->tag,
              "Detection Pin Registration Error",
              "Failed to register Card Detect pin %d: %s",
              sd_card->pin_config.gpio_det_pin,
              esp_err_to_name(reg_err));
    return reg_err;
  }

  /* Check if the GPIO bus already exists */
  pstar_bus_config_t* existing_gpio =
    pstar_bus_manager_find_bus(&sd_card->bus_manager, CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME);
  if (existing_gpio != NULL) {
    log_info(sd_card->tag,
             "Detection Setup",
             "GPIO bus '%s' already exists. Assuming it's configured correctly.",
             CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME);
    /* If bus exists, just add the ISR */
    esp_err_t isr_err = pstar_bus_gpio_isr_add(&sd_card->bus_manager,
                                               CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME,
                                               sd_card->pin_config.gpio_det_pin,
                                               priv_sd_card_detection_isr,
                                               sd_card);
    /* Allow ESP_ERR_INVALID_STATE if ISR service or handler already added */
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
      log_error(sd_card->tag,
                "Detection Setup Error",
                "Failed to add ISR to existing GPIO bus: %s",
                esp_err_to_name(isr_err));
      /* Don't forget to unregister the pin if ISR add fails */
      pin_validator_unregister_pin(sd_card->pin_config.gpio_det_pin, "SD Card HAL");
      return isr_err;
    }
    log_info(sd_card->tag,
             "Detection Setup Complete",
             "Card detection ISR added/verified on existing GPIO bus.");
    return ESP_OK; /* Return success even if ISR already added */
  }

  /* Create GPIO bus configuration */
  pstar_bus_config_t* gpio_config =
    pstar_bus_config_create_gpio(CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME,
                                 k_pstar_mode_interrupt);
  if (gpio_config == NULL) {
    log_error(sd_card->tag, "Detection Setup Error", "Failed to create GPIO bus configuration");
    /* Unregister pin if bus creation fails */
    pin_validator_unregister_pin(sd_card->pin_config.gpio_det_pin, "SD Card HAL");
    return ESP_ERR_NO_MEM;
  }

  /* Configure detection pin */
  gpio_config->config.gpio.config.pin_bit_mask = (1ULL << sd_card->pin_config.gpio_det_pin);
  gpio_config->config.gpio.config.mode         = GPIO_MODE_INPUT;
  gpio_config->config.gpio.config.pull_up_en   = GPIO_PULLUP_ENABLE; /* Enable pullup by default */
  gpio_config->config.gpio.config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config->config.gpio.config.intr_type    = GPIO_INTR_ANYEDGE;

  /* Initialize default GPIO operations */
  pstar_bus_gpio_init_default_ops(&gpio_config->config.gpio.ops);

  /* Add GPIO bus to the manager */
  esp_err_t err = pstar_bus_manager_add_bus(&sd_card->bus_manager, gpio_config);
  if (err != ESP_OK) {
    log_error(sd_card->tag,
              "Detection Setup Error",
              "Failed to add GPIO bus to manager: %s",
              esp_err_to_name(err));
    pstar_bus_config_destroy(gpio_config);
    /* Unregister pin if bus add fails */
    pin_validator_unregister_pin(sd_card->pin_config.gpio_det_pin, "SD Card HAL");
    return err;
  }

  /* Initialize the GPIO bus */
  err = pstar_bus_config_init(gpio_config);
  if (err != ESP_OK) {
    log_error(sd_card->tag,
              "Detection Setup Error",
              "Failed to initialize GPIO bus: %s",
              esp_err_to_name(err));
    pstar_bus_manager_remove_bus(&sd_card->bus_manager, CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME);
    /* Unregister pin if bus init fails */
    pin_validator_unregister_pin(sd_card->pin_config.gpio_det_pin, "SD Card HAL");
    return err;
  }

  /* Register ISR handler for the detection pin */
  err = pstar_bus_gpio_isr_add(&sd_card->bus_manager,
                               CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME,
                               sd_card->pin_config.gpio_det_pin,
                               priv_sd_card_detection_isr,
                               sd_card);
  if (err != ESP_OK) {
    log_error(sd_card->tag,
              "Detection Setup Error",
              "Failed to register ISR for card detection pin: %s",
              esp_err_to_name(err));
    pstar_bus_config_deinit(gpio_config);
    pstar_bus_manager_remove_bus(&sd_card->bus_manager, CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME);
    /* Unregister pin if ISR add fails */
    pin_validator_unregister_pin(sd_card->pin_config.gpio_det_pin, "SD Card HAL");
    return err;
  }

  log_info(sd_card->tag,
           "Detection Setup Complete",
           "Card detection mechanism configured successfully with %s-active pin %d",
           sd_card->card_detect_low_active ? "low" : "high",
           sd_card->pin_config.gpio_det_pin);
  return ESP_OK;
}

/**
 * @brief ISR handler for SD card detection changes
 *
 * This function runs in interrupt context and must be extremely lightweight.
 * We simply notify the mount task which will handle debouncing and actual card processing.
 *
 * @param[in] arg Pointer to the SD card HAL instance
 */
static void IRAM_ATTR priv_sd_card_detection_isr(void* arg)
{
  /* This is an ISR, so we can only do minimal processing here */
  /* Validate input pointer - critical in ISR context */
  if (arg == NULL) {
    return;
  }

  sd_card_hal_t* sd_card = (sd_card_hal_t*)arg;

  /* Double check the mount task handle to avoid potential NULL dereference */
  TaskHandle_t task_handle = sd_card->mount_task_handle;
  if (task_handle == NULL) {
    return;
  }

  /* Use direct-to-task notification - most efficient method for ISR */
  BaseType_t higher_task_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(task_handle, &higher_task_priority_task_woken);

  /* Yield only if necessary to minimize context switch overhead */
  if (higher_task_priority_task_woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED */

esp_err_t sd_card_try_interfaces(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    log_error(TAG, "Interface Error", "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if any interface mode is enabled */
  if (!storage_spi_is_supported() && !storage_sdio_is_supported()) {
    log_error(sd_card->tag,
              "Interface Error",
              "No SD card interfaces (SDIO or SPI) are enabled in Kconfig.");
    return ESP_ERR_NOT_SUPPORTED;
  }

  sd_card->interface_attempt_count++;
  esp_err_t     last_error = ESP_FAIL; /* Default to failure */
  sdmmc_card_t* temp_card  = NULL;     /* For temporary storage if freeing needed */

  /* Determine the order of interfaces to try */
  sd_interface_type_t interfaces_to_try[k_sd_interface_count];
  int                 try_count = 0;

  sd_interface_type_t first_interface  = sd_card->preferred_interface;
  sd_interface_type_t second_interface = k_sd_interface_none;

  if (first_interface == k_sd_interface_sdio && sd_card->sdio_mode_enabled) {
    interfaces_to_try[try_count++] = k_sd_interface_sdio;
    if (sd_card->enable_fallback && sd_card->spi_mode_enabled) {
      second_interface = k_sd_interface_spi;
    }
  } else if (first_interface == k_sd_interface_spi && sd_card->spi_mode_enabled) {
    interfaces_to_try[try_count++] = k_sd_interface_spi;
    if (sd_card->enable_fallback && sd_card->sdio_mode_enabled) {
      second_interface = k_sd_interface_sdio;
    }
  } else {
    /* Preferred interface is disabled, try the other one if enabled */
    if (sd_card->sdio_mode_enabled) {
      interfaces_to_try[try_count++] = k_sd_interface_sdio;
    } else if (sd_card->spi_mode_enabled) {
      interfaces_to_try[try_count++] = k_sd_interface_spi;
    }
    /* Fallback is irrelevant if only one is enabled */
    second_interface = k_sd_interface_none;
  }

  /* Add the second interface if applicable */
  if (second_interface != k_sd_interface_none) {
    interfaces_to_try[try_count++] = second_interface;
  }

  if (try_count == 0) {
    log_error(sd_card->tag, "Interface Error", "No enabled SD card interfaces to try.");
    return ESP_ERR_NOT_SUPPORTED;
  }

  log_info(sd_card->tag,
           "Interface Discovery",
           "Starting interface discovery (attempt %lu)",
           sd_card->interface_attempt_count);

  /* Try interfaces in the determined order */
  for (int i = 0; i < try_count; i++) {
    sd_interface_type_t current_try = interfaces_to_try[i];

    /* Clean up resources from previous attempt (if any) */
    if (sd_card->current_interface == k_sd_interface_spi && sd_card->card != NULL) {
      sdspi_host_remove_device(sd_card->card->host.slot);
      sdspi_host_deinit();
    } else if (sd_card->current_interface == k_sd_interface_sdio) {
      sdmmc_host_deinit();
    }
    if (sd_card->card != NULL) {
      temp_card     = sd_card->card; /* Hold pointer to free after setup */
      sd_card->card = NULL;
    }
    sd_card->current_interface = k_sd_interface_none; /* Reset current */

    log_info(sd_card->tag,
             "Interface Try",
             "Attempting interface: %s",
             sd_card_interface_to_string(current_try));

    esp_err_t setup_err = ESP_FAIL;
    if (current_try == k_sd_interface_sdio) {
      setup_err = storage_sdio_setup(sd_card);
    } else if (current_try == k_sd_interface_spi) {
      setup_err = storage_spi_setup(sd_card);
    }

    /* Free the old card structure if setup allocated a new one */
    if (temp_card != NULL) {
      free(temp_card);
      temp_card = NULL;
    }

    if (setup_err != ESP_OK) {
      log_warn(sd_card->tag,
               "Interface Failure",
               "Failed to setup %s: %s",
               sd_card_interface_to_string(current_try),
               esp_err_to_name(setup_err));
      last_error = setup_err;
      sd_card->interface_info[current_try].error_count++;
      sd_card->interface_info[current_try].attempted = true; /* Mark as attempted */
      continue;                                              /* Try next interface */
    }

    /* Try to mount with this interface */
    esp_err_t mount_err = sd_card_mount(sd_card);
    if (mount_err != ESP_OK) {
      log_warn(sd_card->tag,
               "Interface Failure",
               "Failed to mount with %s: %s",
               sd_card_interface_to_string(current_try),
               esp_err_to_name(mount_err));
      last_error = mount_err;
      sd_card->interface_info[current_try].error_count++;
      sd_card->interface_info[current_try].attempted = true; /* Mark as attempted */
      continue;                                              /* Try next interface */
    }

    /* Success! */
    log_info(sd_card->tag,
             "Interface Success",
             "Successfully mounted SD card with %s",
             sd_card_interface_to_string(current_try));
    sd_card->current_interface                             = current_try;
    sd_card->interface_info[current_try].successful        = true;
    sd_card->interface_info[current_try].last_success_time = esp_timer_get_time();
    sd_card->interface_info[current_try].error_count       = 0; /* Reset error count on success */
    sd_card->interface_info[current_try].attempted         = true; /* Mark as attempted */
    sd_card->interface_discovery_complete                  = true;
    return ESP_OK; /* Exit loop on first success */
  }

  /* If we reach here, all tried interfaces failed */
  log_error(sd_card->tag,
            "Interface Error",
            "All enabled interfaces failed after %lu attempts",
            sd_card->interface_attempt_count);
  sd_card->current_interface = k_sd_interface_none; /* Ensure no interface is set */
  /* Clean up resources from the last failed attempt */
  if (sd_card->current_interface == k_sd_interface_spi && sd_card->card != NULL) {
    sdspi_host_remove_device(sd_card->card->host.slot);
    sdspi_host_deinit();
  } else if (sd_card->current_interface == k_sd_interface_sdio) {
    sdmmc_host_deinit();
  }
  if (sd_card->card != NULL) {
    free(sd_card->card);
    sd_card->card = NULL;
  }
  return last_error; /* Return the error from the last failed attempt */
}

/**
 * @brief FreeRTOS task for monitoring SD card insertion/removal and mounting/unmounting
 *
 * @param[in] arg Pointer to the SD card HAL instance
 */
static void priv_sd_card_mount_task(void* arg)
{
  sd_card_hal_t* sd_card = (sd_card_hal_t*)arg;
  if (sd_card == NULL) {
    log_error(TAG, "Mount Task Error", "Invalid task parameter");
    vTaskDelete(NULL);
    return;
  }

  /* Check if any interface mode is enabled */
  if (!storage_spi_is_supported() && !storage_sdio_is_supported()) {
    log_warn(sd_card->tag,
           "Mount Task Info",
           "SD Card HAL disabled (no interfaces enabled). Exiting mount task.");
    sd_card->mount_task_handle = NULL; /* Clear handle before exit */
    vTaskDelete(NULL);                 /* Exit task if HAL is disabled */
    return;
  }
  log_info(sd_card->tag, "Mount Task Started", "SD card mount/unmount task is running");

  bool mutex_taken = false;

  /* Initialize state machine */
  sd_card->state                  = k_sd_state_idle;
  sd_card->last_state_change_time = esp_timer_get_time();

  /* Initialize exit request flag */
  sd_card->mount_task_exit_requested = false;

  /* Load previous working configuration from NVS if available */
  storage_load_working_config(sd_card);

  /* Initial card check */
  bool is_inserted;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  is_inserted               = storage_sd_card_is_inserted(sd_card);
  uint32_t debounce_counter = 0; /* Declare here if detection enabled */
#else
  is_inserted = true; /* Assume inserted if detection is disabled, rely on mount success */
#endif
  if (is_inserted) {
    log_info(sd_card->tag, "Initial Detection", "SD card detected at startup");

    /* Try to mount the card */
    if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
      mutex_taken = true;
      /* Update state machine */
      storage_update_state_machine(sd_card, k_sd_state_card_inserted);
      storage_update_state_machine(sd_card, k_sd_state_interface_discovery);

      /* Try interfaces */
      esp_err_t err = sd_card_try_interfaces(sd_card);
      if (err != ESP_OK) {
        log_error(sd_card->tag,
                  "Initial Mount Error",
                  "Failed to mount SD card at startup: %s",
                  esp_err_to_name(err));
        /* Update state machine to reflect the error */
        storage_update_state_machine(sd_card, k_sd_state_error);
      } else if (atomic_load(&sd_card->card_available)) {
        /* Interface discovery successful, update state */
        storage_update_state_machine(sd_card, k_sd_state_interface_ready);
      }

      /* Always release mutex when done */
      storage_release_mutex_if_taken(sd_card, &mutex_taken);
    } else {
      log_error(sd_card->tag,
                "Initial Detection Error",
                "Failed to acquire mutex for initial card detection");
    }

  } else {
    log_info(sd_card->tag, "Initial Detection", "No SD card detected at startup");
  }

  /* Main task loop */
  while (!sd_card->mount_task_exit_requested) {
    /* Wait for detection event or timeout - shorter timeout to check exit flag more frequently */
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

    /* Check exit flag again after potential long wait */
    if (sd_card->mount_task_exit_requested) {
      break;
    }

/* Check card status */
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
    is_inserted = storage_sd_card_is_inserted(sd_card);
#else
    /* If detection is disabled, rely on health check or external events */
    is_inserted = atomic_load(&sd_card->card_available);
    /* Assume still inserted if mounted */
#endif

/* Debounce the detection (only if detection enabled) */
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
    if (is_inserted != atomic_load(&sd_card->card_available)) {
      if (debounce_counter == 0) {
        /* First detection of change */
      }
      debounce_counter++; /* Increment counter on every check during change */

      if (debounce_counter >= pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_SD_CARD_DEBOUNCE_MS) /
                                pdMS_TO_TICKS(100)) { /* Check if debounce time passed */
#else
    /* If detection is disabled, we only react to mount/unmount failures (handled elsewhere) */
    /* or explicit requests. No debouncing needed here. */
    if (false) { /* Dummy condition to keep structure */
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED */
/* Debounce period expired or detection disabled, consider it a stable change */
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
        log_info(sd_card->tag,
                 "Card Status Change",
                 "SD card %s",
                 is_inserted ? "inserted" : "removed");
#endif

        /* Take mutex for thread safety */
        mutex_taken = false;
        if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
          mutex_taken = true;
          if (is_inserted && !atomic_load(&sd_card->card_available)) {
            /* Card inserted - update state machine */
            storage_update_state_machine(sd_card, k_sd_state_card_inserted);
            storage_update_state_machine(sd_card, k_sd_state_interface_discovery);

            /* Try interfaces in priority order */
            esp_err_t err = sd_card_try_interfaces(sd_card);
            if (err != ESP_OK) {
              log_error(sd_card->tag,
                        "Mount Error",
                        "Failed to mount SD card: %s",
                        esp_err_to_name(err));

              /* Record the error */
              RECORD_ERROR(&sd_card->error_handler, err, "Failed to mount SD card");

              /* Update state machine to reflect the error */
              storage_update_state_machine(sd_card, k_sd_state_error);

              /* If we can retry, do so after some delay */
              if (error_handler_can_retry(&sd_card->error_handler)) {
                log_info(sd_card->tag, "Retry", "Will retry mounting after delay");
                /* The error handler will call the reset function */
              } else {
                /* Maximum retries reached */
                storage_update_state_machine(sd_card, k_sd_state_failed);
              }
            } else if (atomic_load(&sd_card->card_available)) {
              /* Mount successful - update state machine */
              storage_update_state_machine(sd_card, k_sd_state_interface_ready);
              /* Reset error handler */
              error_handler_reset_state(&sd_card->error_handler);
            }
          } else if (!is_inserted && atomic_load(&sd_card->card_available)) {
            /* Card removed - unmount first */
            esp_err_t err = sd_card_unmount(sd_card);
            if (err != ESP_OK) {
              log_error(sd_card->tag,
                        "Unmount Error",
                        "Failed to unmount SD card: %s",
                        esp_err_to_name(err));

              /* Even if unmount fails, we need to update the state machine */
              /* to reflect that the card is physically removed */
            }

            /* Then update state machine */
            storage_update_state_machine(sd_card, k_sd_state_idle);

            /* Reset interface discovery state */
            sd_card->interface_discovery_complete = false;

            log_info(sd_card->tag,
                     "Card Removed",
                     "SD card has been physically removed and unmounted");
          }

          /* Always release mutex when done */
          storage_release_mutex_if_taken(sd_card, &mutex_taken);
        } else {
          log_error(sd_card->tag,
                    "Card State Change Error",
                    "Failed to acquire mutex for card state change handling");
        }
/* Update stable state */
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
        debounce_counter = 0; /* Reset counter after stable change */
#endif
      }
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
    } else {
      /* No change detected, reset debounce counter */
      debounce_counter = 0;
    }
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED */

    /* Check exit flag before potentially long operation */
    if (sd_card->mount_task_exit_requested) {
      break;
    }

    /* Handle card in error state - check for retries */
    if (sd_card->state == k_sd_state_error && is_inserted &&
        error_handler_can_retry(&sd_card->error_handler)) { /* This takes+gives mutex */

      /* Get the delay *before* taking the main mutex for the operation */
      uint32_t delay_ms   = 0;
      bool     need_delay = false;
      /* Temporarily take mutex just to get delay value */
      if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
        /* Only delay if it's not the first retry (retry count > 0) */
        if (sd_card->error_handler.current_retry > 0) {
          delay_ms   = sd_card->error_handler.current_retry_delay;
          need_delay = true;
        }
        xSemaphoreGive(sd_card->mutex);
      } else {
        log_error(sd_card->tag, "Error Retry Error", "Failed to acquire mutex to get delay");
        continue; /* Skip retry attempt */
      }

      /* Perform delay *outside* the main mutex lock if needed */
      if (need_delay) {
        log_info(sd_card->tag,
                 "Error Retry",
                 "Delaying %lu ms before retry %lu/%lu",
                 delay_ms,
                 sd_card->error_handler.current_retry + 1, /* Log the upcoming retry number */
                 sd_card->error_handler.max_retries);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
      }

      /* Now acquire mutex for the actual reset attempt */
      mutex_taken = false;
      if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
        mutex_taken = true;

        /* Double-check state and retry condition *after* delay and acquiring mutex */
        if (sd_card->state == k_sd_state_error && is_inserted &&
            error_handler_can_retry(&sd_card->error_handler)) {
          log_info(sd_card->tag,
                   "Error Recovery",
                   "Attempting to recover from error (retry %lu/%lu)",
                   sd_card->error_handler.current_retry + 1, /* Log the upcoming retry number */
                   sd_card->error_handler.max_retries);

          /* Call reset function (which now assumes mutex is held) */
          esp_err_t err = storage_sd_card_reset(sd_card);

          if (err != ESP_OK) {
            log_error(sd_card->tag,
                      "Reset Error",
                      "Failed to reset SD card: %s",
                      esp_err_to_name(err));

            /* Record the error *after* the reset attempt */
            /* Note: error_handler_record_error increments the retry count internally */
            RECORD_ERROR(&sd_card->error_handler, err, "Failed to reset SD card");

            /* Check if max retries reached *after* recording the error */
            if (!error_handler_can_retry(&sd_card->error_handler)) {
              storage_update_state_machine(sd_card, k_sd_state_failed);
            }
            /* State remains k_sd_state_error if retries remain */
          } else if (atomic_load(&sd_card->card_available)) {
            /* Reset successful, card mounted */
            storage_update_state_machine(sd_card, k_sd_state_interface_ready);
            error_handler_reset_state(&sd_card->error_handler); /* Reset error state on success */
          } else {
            /* Reset OK, but card still not available (e.g. removed) */
            storage_update_state_machine(sd_card, k_sd_state_idle);
            error_handler_reset_state(&sd_card->error_handler); /* Reset error state */
          }
        } else {
          log_info(sd_card->tag,
                   "Error Retry Skip",
                   "Condition for retry no longer met after delay/mutex acquisition.");
        }
        /* Release mutex after operation */
        storage_release_mutex_if_taken(sd_card, &mutex_taken);
      } else {
        log_error(sd_card->tag,
                  "Error Retry Error",
                  "Failed to acquire mutex for error retry attempt");
      }
    } /* end if can_retry */

    /* Perform performance measurements if needed and card is ready */
    if (sd_card->state == k_sd_state_interface_ready && atomic_load(&sd_card->card_available) &&
        sd_card->performance.measurement_needed) {
      /* Take mutex for thread safety */
      mutex_taken = false;
      if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
        mutex_taken = true;

        /* Run performance test */
        storage_measure_card_performance(sd_card);

        /* Release mutex */
        storage_release_mutex_if_taken(sd_card, &mutex_taken);
      }
    }

    /* Periodic health check when card is mounted */
    if (sd_card->state == k_sd_state_interface_ready && atomic_load(&sd_card->card_available)) {
      /* Only check every 10 seconds (100 ticks × 100ms) */
      static uint32_t health_check_counter = 0;
      if (++health_check_counter >= 100) {
        health_check_counter = 0;

        /* Take mutex for thread safety */
        mutex_taken = false;
        if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
          mutex_taken = true;

          /* Check if card is still functioning by reading basic info */
          if (sd_card->card != NULL) {
            /* Just a simple check - try to access the card */
            struct stat st;
            if (stat(sd_card->mount_path, &st) != 0) {
              log_warn(sd_card->tag,
                       "Health Check",
                       "Failed to access SD card mount point - possible file system issue");

              /* Record the error */
              RECORD_ERROR(&sd_card->error_handler,
                           errno, /* Use errno from stat */
                           "Failed health check (stat failed)");

              /* Update state */
              storage_update_state_machine(sd_card, k_sd_state_error);

              /* Let the error retry logic handle the reset */
            }
          }

          /* Release mutex */
          storage_release_mutex_if_taken(sd_card, &mutex_taken);
        }
      }
    }
  }

  log_info(sd_card->tag, "Mount Task Exiting", "SD card mount/unmount task received exit request");

  sd_card->mount_task_handle = NULL; /* Clear handle before deleting */
  vTaskDelete(NULL);
}
