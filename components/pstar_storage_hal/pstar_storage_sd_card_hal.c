/* components/pstar_storage_hal/pstar_storage_sd_card_hal.c */

#include "pstar_bus_config.h"
#include "pstar_bus_event_types.h"
#include "pstar_bus_gpio.h"
#include "pstar_bus_manager.h" // <-- Added include
#include "pstar_log_handler.h"
#include "pstar_pin_validator.h"
#include "pstar_storage_common.h"
#include "pstar_storage_hal.h"
#include "pstar_storage_sdio_hal.h" // <-- Need to include for declarations
#include "pstar_storage_spi_hal.h"  // <-- Need to include for declarations

#include "driver/gpio.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <stdatomic.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "sdmmc_cmd.h"

/* Constants ******************************************************************/
#define SD_INTERFACE_MAX_ERRORS (3)
static const char* TAG = "SD Card HAL";

/* ISR/Task Functions - Forward Declarations **********************************/
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
static void priv_sd_card_detection_isr(void* arg);
#endif
static void priv_sd_card_mount_task(void* arg);

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

  /* FIX: Check if physically inserted *before* proceeding */
  /* This function now handles mutex internally and logs errors */
  if (!storage_sd_card_is_inserted(sd_card)) {
    log_warn(sd_card->tag,
             "Mount Warning",
             "Cannot mount: SD card not inserted (or detection failed)");
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
    .format_if_mount_failed = false,
    .max_files              = sd_card->max_files,
    .allocation_unit_size   = sd_card->allocation_unit_size,
  };

  /* Try to mount the SD card */
  sdmmc_card_t* card_out = NULL;
  esp_err_t     err;

  /* Slot config is only used for SDIO, pass NULL for SPI */
  sdmmc_slot_config_t* slot_config_ptr = NULL;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  if (sd_card->current_interface == k_sd_interface_sdio) {
    slot_config.clk   = sd_card->pin_config.sdio_clk_pin;
    slot_config.cmd   = sd_card->pin_config.sdio_cmd_pin;
    slot_config.d0    = sd_card->pin_config.sdio_d0_pin;
    slot_config.width = sd_card->bus_width;

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
    slot_config_ptr     = &slot_config;
  }
#endif // SDIO_MODE_ENABLED

  /* FIX: Ensure mutex is NOT held during potentially blocking VFS mount */
  err = esp_vfs_fat_sdmmc_mount(sd_card->mount_path,
                                &sd_card->card->host,
                                slot_config_ptr,
                                &mount_config,
                                &card_out);

/* Handle potential bus width issues specifically for SDIO */
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
  if (err != ESP_OK && sd_card->current_interface == k_sd_interface_sdio &&
      sd_card->bus_width == k_sd_bus_width_4bit) {
    log_warn(sd_card->tag,
             "Mount Warning",
             "Failed to mount SDIO with 4-bit bus width, trying 1-bit mode");

    sd_card->card->host.flags &= ~SDMMC_HOST_FLAG_4BIT;
    sd_card->card->host.flags |= SDMMC_HOST_FLAG_1BIT;
    slot_config.width = 1;

    /* FIX: Ensure mutex is NOT held during potentially blocking VFS mount */
    err = esp_vfs_fat_sdmmc_mount(sd_card->mount_path,
                                  &sd_card->card->host,
                                  &slot_config,
                                  &mount_config,
                                  &card_out);

    if (err == ESP_OK) {
      sd_card->bus_width = k_sd_bus_width_1bit;
      log_info(sd_card->tag,
               "Mount Info",
               "Successfully mounted with 1-bit mode after 4-bit mode failed");
    }
  }
#endif // SDIO_MODE_ENABLED

  if (err != ESP_OK) {
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
    if (card_out != NULL) {
      free(card_out);
    }
    return err;
  }

  if (sd_card->card != card_out) {
    if (sd_card->card != NULL) {
      free(sd_card->card);
    }
    sd_card->card = card_out;
  }

  char logs_path[CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH];
  if (snprintf(logs_path, sizeof(logs_path), "%s/logs", sd_card->mount_path) >=
      (int)sizeof(logs_path)) {
    log_warn(sd_card->tag, "Mount Warning", "Log path truncated");
  }

  esp_err_t logs_dir_result = storage_create_directory_if_needed(sd_card, logs_path);
  if (logs_dir_result != ESP_OK) {
    log_warn(sd_card->tag,
             "Mount Warning",
             "Failed to create logs directory: %s",
             esp_err_to_name(logs_dir_result));
  }
  sdmmc_card_print_info(stdout, sd_card->card);
  log_info(sd_card->tag,
           "Mount Success",
           "SD card mounted at %s, %lluMB, Interface: %s, %s bus width",
           sd_card->mount_path,
           ((uint64_t)sd_card->card->csd.capacity * sd_card->card->csd.sector_size) / (1024 * 1024),
           sd_card_interface_to_string(sd_card->current_interface),
           storage_bus_width_to_string(sd_card->bus_width));

  atomic_store(&sd_card->card_available, true);
  sd_card->performance.measurement_needed = true;
  storage_notify_availability(sd_card, true);

  return ESP_OK;
}

esp_err_t sd_card_unmount(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    log_error(TAG, "Unmount Error", "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }
  if (!atomic_load(&sd_card->card_available)) {
    log_info(sd_card->tag, "Unmount Info", "SD card already unmounted");
    return ESP_OK;
  }
  log_info(sd_card->tag,
           "Unmount Started",
           "Unmounting SD card from path: %s",
           sd_card->mount_path);

  /* FIX: Ensure mutex is NOT held during potentially blocking VFS unmount */
  esp_err_t err = esp_vfs_fat_sdcard_unmount(sd_card->mount_path, sd_card->card);
  if (err != ESP_OK) {
    log_error(sd_card->tag, "Unmount Error", "Failed to unmount SD card: %s", esp_err_to_name(err));
    atomic_store(&sd_card->card_available, false);
    storage_notify_availability(sd_card, false);
    return err;
  }

  atomic_store(&sd_card->card_available, false);

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  if (sd_card->current_interface == k_sd_interface_spi && sd_card->card != NULL) {
    sdspi_host_remove_device(sd_card->card->host.slot);
    sdspi_host_deinit();
  }
#endif
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
  else if (sd_card->current_interface == k_sd_interface_sdio) {
    sdmmc_host_deinit();
  }
#endif
  sd_card->current_interface = k_sd_interface_none;

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

  if (sd_card->pin_config.gpio_det_pin < 0) {
    log_warn(sd_card->tag,
             "Detection Setup Warning",
             "Card detection enabled but pin is invalid (-1). Skipping setup.");
    return ESP_OK;
  }

  log_info(sd_card->tag, "Detection Setup", "Setting up card detection mechanism");

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
  if (sd_card->bus_width == k_sd_bus_width_4bit && sd_card->pin_config.gpio_det_pin >= 0 &&
      sd_card->pin_config.gpio_det_pin == sd_card->pin_config.sdio_d3_pin) {
    log_warn(sd_card->tag,
             "Pin Conflict",
             "Card detect pin (GPIO %d) conflicts with SDIO D3 for 4-bit mode. "
             "Detection may be unreliable.",
             sd_card->pin_config.gpio_det_pin);
  }
#endif

  /* Pin registration is now handled in storage_register_sd_card_pins */

  pstar_bus_config_t* existing_gpio =
    pstar_bus_manager_find_bus(&sd_card->bus_manager, CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME);
  if (existing_gpio != NULL) {
    log_info(sd_card->tag,
             "Detection Setup",
             "GPIO bus '%s' already exists. Assuming it's configured correctly.",
             CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME);
    esp_err_t isr_err = pstar_bus_gpio_isr_add(&sd_card->bus_manager,
                                               CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME,
                                               sd_card->pin_config.gpio_det_pin,
                                               priv_sd_card_detection_isr,
                                               sd_card);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
      log_error(sd_card->tag,
                "Detection Setup Error",
                "Failed to add ISR to existing GPIO bus: %s",
                esp_err_to_name(isr_err));
      return isr_err;
    }
    log_info(sd_card->tag,
             "Detection Setup Complete",
             "Card detection ISR added/verified on existing GPIO bus.");
    return ESP_OK;
  }

  pstar_bus_config_t* gpio_config =
    pstar_bus_config_create_gpio(CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME,
                                 k_pstar_mode_interrupt);
  if (gpio_config == NULL) {
    log_error(sd_card->tag, "Detection Setup Error", "Failed to create GPIO bus configuration");
    return ESP_ERR_NO_MEM;
  }

  gpio_config->config.gpio.config.pin_bit_mask = (1ULL << sd_card->pin_config.gpio_det_pin);
  gpio_config->config.gpio.config.mode         = GPIO_MODE_INPUT;
  gpio_config->config.gpio.config.pull_up_en   = GPIO_PULLUP_ENABLE;
  gpio_config->config.gpio.config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config->config.gpio.config.intr_type    = GPIO_INTR_ANYEDGE;

  pstar_bus_gpio_init_default_ops(&gpio_config->config.gpio.ops);

  esp_err_t err = pstar_bus_manager_add_bus(&sd_card->bus_manager, gpio_config);
  if (err != ESP_OK) {
    log_error(sd_card->tag,
              "Detection Setup Error",
              "Failed to add GPIO bus to manager: %s",
              esp_err_to_name(err));
    pstar_bus_config_destroy(gpio_config);
    return err;
  }

  err = pstar_bus_config_init(gpio_config);
  if (err != ESP_OK) {
    log_error(sd_card->tag,
              "Detection Setup Error",
              "Failed to initialize GPIO bus: %s",
              esp_err_to_name(err));
    pstar_bus_manager_remove_bus(&sd_card->bus_manager, CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME);
    return err;
  }

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
 */
static void IRAM_ATTR priv_sd_card_detection_isr(void* arg)
{
  if (arg == NULL) {
    return;
  }
  sd_card_hal_t* sd_card     = (sd_card_hal_t*)arg;
  TaskHandle_t   task_handle = sd_card->mount_task_handle;
  if (task_handle == NULL) {
    return;
  }
  BaseType_t higher_task_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(task_handle, &higher_task_priority_task_woken);
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

  bool sdio_supported = false;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
  sdio_supported = storage_sdio_is_supported();
#endif
  bool spi_supported = false;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  spi_supported = storage_spi_is_supported();
#endif

  if (!spi_supported && !sdio_supported) {
    log_error(sd_card->tag,
              "Interface Error",
              "No SD card interfaces (SDIO or SPI) are enabled in Kconfig.");
    return ESP_ERR_NOT_SUPPORTED;
  }

  sd_card->interface_attempt_count++;
  esp_err_t           last_error = ESP_FAIL;
  sdmmc_card_t*       temp_card  = NULL;
  sd_interface_type_t interfaces_to_try[k_sd_interface_count];
  int                 try_count = 0;

  sd_interface_type_t first_interface  = sd_card->preferred_interface;
  sd_interface_type_t second_interface = k_sd_interface_none;

  if (first_interface == k_sd_interface_sdio && sd_card->sdio_mode_enabled && sdio_supported) {
    interfaces_to_try[try_count++] = k_sd_interface_sdio;
    if (sd_card->enable_fallback && sd_card->spi_mode_enabled && spi_supported) {
      second_interface = k_sd_interface_spi;
    }
  } else if (first_interface == k_sd_interface_spi && sd_card->spi_mode_enabled && spi_supported) {
    interfaces_to_try[try_count++] = k_sd_interface_spi;
    if (sd_card->enable_fallback && sd_card->sdio_mode_enabled && sdio_supported) {
      second_interface = k_sd_interface_sdio;
    }
  } else {
    // Preferred interface is not enabled/supported, try others
    if (sd_card->sdio_mode_enabled && sdio_supported) {
      interfaces_to_try[try_count++] = k_sd_interface_sdio;
    } else if (sd_card->spi_mode_enabled && spi_supported) {
      interfaces_to_try[try_count++] = k_sd_interface_spi;
    }
    second_interface = k_sd_interface_none;
  }

  if (second_interface != k_sd_interface_none) {
    interfaces_to_try[try_count++] = second_interface;
  }

  if (try_count == 0) {
    log_error(sd_card->tag, "Interface Error", "No enabled/supported SD card interfaces to try.");
    return ESP_ERR_NOT_SUPPORTED;
  }

  log_info(sd_card->tag,
           "Interface Discovery",
           "Starting interface discovery (attempt %lu)",
           sd_card->interface_attempt_count);

  for (int i = 0; i < try_count; i++) {
    sd_interface_type_t current_try = interfaces_to_try[i];

    /* --- Cleanup previous interface attempt --- */
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
    if (sd_card->current_interface == k_sd_interface_spi && sd_card->card != NULL) {
      sdspi_host_remove_device(sd_card->card->host.slot);
      sdspi_host_deinit();
    }
#endif
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
    else if (sd_card->current_interface == k_sd_interface_sdio) {
      sdmmc_host_deinit();
    }
#endif
    if (sd_card->card != NULL) {
      temp_card     = sd_card->card;
      sd_card->card = NULL;
    }
    sd_card->current_interface = k_sd_interface_none;
    /* --- End Cleanup --- */

    log_info(sd_card->tag,
             "Interface Try",
             "Attempting interface: %s",
             sd_card_interface_to_string(current_try));

    esp_err_t setup_err = ESP_FAIL;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
    if (current_try == k_sd_interface_sdio) {
      setup_err = storage_sdio_setup(sd_card);
    }
#endif
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
    if (current_try == k_sd_interface_spi) { // Use 'if' not 'else if' in case only SPI is enabled
      setup_err = storage_spi_setup(sd_card);
    }
#endif

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
      sd_card->interface_info[current_try].attempted = true;
      continue; // Try next interface
    }

    // Setup succeeded, now try to mount
    // FIX: Release mutex before calling mount (sd_card_mount handles its own mutex needs)
    esp_err_t mount_err = sd_card_mount(sd_card);
    if (mount_err != ESP_OK) {
      log_warn(sd_card->tag,
               "Interface Failure",
               "Failed to mount with %s: %s",
               sd_card_interface_to_string(current_try),
               esp_err_to_name(mount_err));
      last_error = mount_err;
      sd_card->interface_info[current_try].error_count++;
      sd_card->interface_info[current_try].attempted = true;
      continue; // Try next interface
    }

    // Mount succeeded!
    log_info(sd_card->tag,
             "Interface Success",
             "Successfully mounted SD card with %s",
             sd_card_interface_to_string(current_try));
    sd_card->current_interface                             = current_try;
    sd_card->interface_info[current_try].successful        = true;
    sd_card->interface_info[current_try].last_success_time = esp_timer_get_time();
    sd_card->interface_info[current_try].error_count       = 0;
    sd_card->interface_info[current_try].attempted         = true;
    sd_card->interface_discovery_complete                  = true;
    return ESP_OK; // Success!
  }

  // All tried interfaces failed
  log_error(sd_card->tag,
            "Interface Error",
            "All enabled interfaces failed after %lu attempts",
            sd_card->interface_attempt_count);
  sd_card->current_interface = k_sd_interface_none;
  /* --- Cleanup last failed attempt --- */
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  if (sd_card->current_interface == k_sd_interface_spi && sd_card->card != NULL) {
    sdspi_host_remove_device(sd_card->card->host.slot);
    sdspi_host_deinit();
  }
#endif
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
  else if (sd_card->current_interface == k_sd_interface_sdio) {
    sdmmc_host_deinit();
  }
#endif
  if (sd_card->card != NULL) {
    free(sd_card->card);
    sd_card->card = NULL;
  }
  /* --- End Cleanup --- */
  return last_error;
}

/* ADD the definition of sd_card_init here */
esp_err_t sd_card_init(sd_card_hal_t* sd_card)
{
  /* Validate arguments */
  if (sd_card == NULL) {
    log_error(TAG, "Init Error", "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }
  /* Check if already initialized */
  if (sd_card->initialized) {
    log_warn(sd_card->tag, "Init Warning", "SD card HAL already initialized");
    return ESP_OK;
  }

  /* Check if any interface mode is enabled */
  bool sdio_supported = false;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
  sdio_supported = storage_sdio_is_supported();
#endif
  bool spi_supported = false;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  spi_supported = storage_spi_is_supported();
#endif
  if (!spi_supported && !sdio_supported) {
    log_error(sd_card->tag,
              "Init Error",
              "Cannot initialize SD Card HAL: Both SDIO and SPI modes are disabled.");
    return ESP_ERR_NOT_SUPPORTED;
  }

  log_info(sd_card->tag,
           "Init Started",
           "Initializing SD card HAL with %s bus width",
           storage_bus_width_to_string(sd_card->bus_width));

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  /* Set up card detection GPIO */
  esp_err_t det_err = sd_card_setup_detection(sd_card);
  if (det_err != ESP_OK) {
    log_error(sd_card->tag,
              "Init Error",
              "Failed to set up card detection: %s",
              esp_err_to_name(det_err));
    return det_err;
  }
#else
  log_info(sd_card->tag, "Detection Info", "SD Card detection is disabled via Kconfig.");
#endif

  /* Register SD card pins with the pin validator */
  esp_err_t pin_err = storage_register_sd_card_pins(sd_card);
  if (pin_err != ESP_OK) {
    log_error(sd_card->tag,
              "Init Error",
              "Failed to register SD card pins: %s",
              esp_err_to_name(pin_err));
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
    bool mutex_taken_cleanup = false;
    if (sd_card->mutex != NULL &&
        xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
      mutex_taken_cleanup = true;
      if (sd_card->pin_config.gpio_det_pin >= 0) {
        pstar_bus_gpio_isr_remove(&sd_card->bus_manager,
                                  CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME,
                                  sd_card->pin_config.gpio_det_pin);
        pin_validator_unregister_pin(sd_card->pin_config.gpio_det_pin, "SD Card HAL");
      }
      storage_release_mutex_if_taken(sd_card, &mutex_taken_cleanup);
    }
#endif
    return pin_err;
  }

  /* Initialize mount task */
  BaseType_t task_created = xTaskCreate(priv_sd_card_mount_task,
                                        "sd_mount_task",
                                        sd_card->task_config.stack_size,
                                        (void*)sd_card,
                                        sd_card->task_config.priority,
                                        &sd_card->mount_task_handle);
  if (task_created != pdPASS) {
    log_error(TAG, "Failed to create mount task", "sd_card: %p", sd_card);
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
    bool mutex_taken = false;
    if (sd_card->mutex != NULL &&
        xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
      mutex_taken = true;
      if (sd_card->pin_config.gpio_det_pin >= 0) {
        pstar_bus_gpio_isr_remove(&sd_card->bus_manager,
                                  CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME,
                                  sd_card->pin_config.gpio_det_pin);
        pin_validator_unregister_pin(sd_card->pin_config.gpio_det_pin, "SD Card HAL");
      }
      storage_release_mutex_if_taken(sd_card, &mutex_taken);
    }
#endif
    /* TODO: Unregister other pins if task creation fails */
    return ESP_ERR_NO_MEM;
  }

  /* Mark as initialized */
  sd_card->initialized = true;
  log_info(sd_card->tag, "Init Complete", "SD card HAL initialization complete");
  return ESP_OK;
}

/**
 * @brief FreeRTOS task for monitoring SD card insertion/removal and mounting/unmounting
 */
static void priv_sd_card_mount_task(void* arg)
{
  sd_card_hal_t* sd_card = (sd_card_hal_t*)arg;
  if (sd_card == NULL) {
    log_error(TAG, "Mount Task Error", "Invalid task parameter");
    vTaskDelete(NULL);
    return;
  }

  bool sdio_supported = false;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
  sdio_supported = storage_sdio_is_supported();
#endif
  bool spi_supported = false;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  spi_supported = storage_spi_is_supported();
#endif
  if (!spi_supported && !sdio_supported) {
    log_warn(sd_card->tag,
             "Mount Task Info",
             "SD Card HAL disabled (no interfaces enabled). Exiting mount task.");
    sd_card->mount_task_handle = NULL;
    vTaskDelete(NULL);
    return;
  }

  log_info(sd_card->tag, "Mount Task Started", "SD card mount/unmount task is running");

  bool mutex_taken = false;

  sd_card->state                     = k_sd_state_idle;
  sd_card->last_state_change_time    = esp_timer_get_time();
  sd_card->mount_task_exit_requested = false;

  storage_load_working_config(sd_card);

  bool is_inserted;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  is_inserted               = storage_sd_card_is_inserted(sd_card);
  uint32_t debounce_counter = 0;
#else
  is_inserted = true; // Assume inserted if detection disabled
#endif

  // Initial check on task start
  if (is_inserted) {
    log_info(sd_card->tag, "Initial Detection", "SD card detected at startup");

    // *** NEW LOGIC for initial mount ***
    bool initial_mount_needed = false;
    if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
      mutex_taken = true;
      // Check if already available (e.g., from loaded config)
      if (!atomic_load(&sd_card->card_available)) {
        storage_update_state_machine(sd_card, k_sd_state_card_inserted);
        storage_update_state_machine(sd_card, k_sd_state_interface_discovery);
        initial_mount_needed = true;
      }
      storage_release_mutex_if_taken(sd_card, &mutex_taken);
    } else {
      log_error(sd_card->tag,
                "Initial Detection Error",
                "Failed to acquire mutex for initial card detection. Cannot attempt mount.");
      storage_update_state_machine(sd_card, k_sd_state_error); // Update state to ERROR
    }

    // Attempt initial mount outside the mutex lock
    if (initial_mount_needed) {
      esp_err_t err = sd_card_try_interfaces(sd_card); // This handles its own mutex needs

      // Update state based on mount result AFTER the attempt
      mutex_taken = false;
      if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
        mutex_taken = true;
        if (err != ESP_OK) {
          log_error(sd_card->tag,
                    "Initial Mount Error",
                    "Failed to mount SD card at startup: %s",
                    esp_err_to_name(err));
          storage_update_state_machine(sd_card, k_sd_state_error);
        } else if (atomic_load(&sd_card->card_available)) {
          storage_update_state_machine(sd_card, k_sd_state_interface_ready);
        } else {
          // Should not happen if try_interfaces returned OK
          log_warn(sd_card->tag,
                   "Initial Mount Status",
                   "try_interfaces OK, but card not available.");
          storage_update_state_machine(sd_card, k_sd_state_error);
        }
        storage_release_mutex_if_taken(sd_card, &mutex_taken);
      } else {
        log_error(sd_card->tag,
                  "Initial Mount State Error",
                  "Failed to acquire mutex to update state after initial mount attempt.");
      }
    }
    // *** END NEW LOGIC for initial mount ***

  } else {
    log_info(sd_card->tag, "Initial Detection", "No SD card detected at startup");
  }

  // Main task loop
  while (!sd_card->mount_task_exit_requested) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)); // Wait for ISR or timeout

    if (sd_card->mount_task_exit_requested) {
      break;
    }

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
    // --- Card Detection Logic (Only if enabled) ---
    bool current_inserted_state = storage_sd_card_is_inserted(sd_card);
    bool previous_available_state =
      atomic_load(&sd_card->card_available); // Get the current mount status

    // Check if the physical state differs from the logical mount state
    if (current_inserted_state != previous_available_state) {
      // If debounce counter is 0, this is the start of a potential change
      if (debounce_counter == 0) {
        log_debug(sd_card->tag,
                  "Debounce Start",
                  "Potential card status change detected: inserted=%d, available=%d",
                  current_inserted_state,
                  previous_available_state);
      }
      debounce_counter++;

      // Check if debounce time has passed
      if (debounce_counter >=
          pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_SD_CARD_DEBOUNCE_MS) / pdMS_TO_TICKS(100)) {
        log_info(sd_card->tag,
                 "Debounce Complete",
                 "Confirmed card status change: inserted=%d, previously_available=%d",
                 current_inserted_state,
                 previous_available_state);

        // *** NEW LOGIC for debounce handling ***
        bool mount_needed   = false;
        bool unmount_needed = false;

        mutex_taken = false;
        if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
          mutex_taken = true;

          // Re-check states *after* acquiring mutex to ensure atomicity
          bool current_inserted_state_locked =
            storage_sd_card_is_inserted(sd_card); // Safe nested take
          bool previous_available_state_locked =
            atomic_load(&sd_card->card_available); // Re-check available state

          if (current_inserted_state_locked && !previous_available_state_locked) {
            // Card Inserted: Update state BEFORE trying interfaces
            log_info(sd_card->tag, "Mount Trigger", "Card inserted, preparing for mount...");
            storage_update_state_machine(sd_card, k_sd_state_card_inserted);
            storage_update_state_machine(sd_card, k_sd_state_interface_discovery);
            mount_needed = true; // Mark that we need to try mounting
          } else if (!current_inserted_state_locked && previous_available_state_locked) {
            // Card Removed: Mark for unmount
            log_info(sd_card->tag, "Unmount Trigger", "Card removed, preparing for unmount...");
            unmount_needed = true;
          } else {
            // State hasn't actually changed between mutex lock and now
            log_debug(sd_card->tag,
                      "Debounce",
                      "Card status consistent after mutex lock, no action needed.");
          }

          storage_release_mutex_if_taken(
            sd_card,
            &mutex_taken); // Release mutex BEFORE trying interfaces/unmount

        } else { // Mutex take failed
          log_error(sd_card->tag,
                    "Card State Change Error",
                    "Failed to acquire mutex for card state change handling");
        }

        // Perform mount or unmount *outside* the initial mutex lock if needed
        if (mount_needed) {
          esp_err_t err =
            sd_card_try_interfaces(sd_card); // This function handles its internal needs

          // Update state based on mount result AFTER the attempt
          mutex_taken = false;
          if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
            mutex_taken = true;
            if (err != ESP_OK) {
              log_error(sd_card->tag,
                        "Mount Error",
                        "Failed to mount SD card: %s",
                        esp_err_to_name(err));
              RECORD_ERROR(&sd_card->error_handler, err, "Failed to mount SD card");
              storage_update_state_machine(sd_card, k_sd_state_error);
              if (!error_handler_can_retry(&sd_card->error_handler)) {
                storage_update_state_machine(sd_card, k_sd_state_failed);
              }
            } else if (atomic_load(&sd_card->card_available)) { // Check if mount succeeded
              storage_update_state_machine(sd_card, k_sd_state_interface_ready);
              error_handler_reset_state(&sd_card->error_handler);
            } else {
              // Mount didn't succeed, even if try_interfaces returned OK? Should not happen.
              log_warn(sd_card->tag,
                       "Mount Status",
                       "try_interfaces returned OK, but card not available.");
              storage_update_state_machine(sd_card, k_sd_state_error);
            }
            storage_release_mutex_if_taken(sd_card, &mutex_taken);
          } else {
            log_error(sd_card->tag,
                      "Mount State Error",
                      "Failed to acquire mutex to update state after mount attempt.");
          }
        } else if (unmount_needed) {
          esp_err_t err = sd_card_unmount(sd_card); // unmount handles internal state

          // Update state after unmount attempt
          mutex_taken = false;
          if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
            mutex_taken = true;
            if (err != ESP_OK) {
              log_error(sd_card->tag,
                        "Unmount Error",
                        "Failed to unmount SD card: %s",
                        esp_err_to_name(err));
            }
            // State should be updated by unmount, but ensure it's idle
            storage_update_state_machine(sd_card, k_sd_state_idle);
            sd_card->interface_discovery_complete = false;
            log_info(sd_card->tag,
                     "Card Removed",
                     "SD card has been physically removed and unmounted");
            error_handler_reset_state(&sd_card->error_handler); // Reset errors on removal
            storage_release_mutex_if_taken(sd_card, &mutex_taken);
          } else {
            log_error(sd_card->tag,
                      "Unmount State Error",
                      "Failed to acquire mutex to update state after unmount attempt.");
          }
        }
        // *** END NEW LOGIC for debounce handling ***

        debounce_counter = 0; // Reset debounce counter after handling
      }
    } else {
      debounce_counter = 0; // Reset if state is stable
    }
    // --- End Card Detection Logic ---
#endif // CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED

    if (sd_card->mount_task_exit_requested) {
      break;
    }

    // --- Error Retry Logic ---
    bool card_physically_present = storage_sd_card_is_inserted(sd_card);
    if (sd_card->state == k_sd_state_error && card_physically_present &&
        error_handler_can_retry(&sd_card->error_handler)) {
      uint32_t delay_ms   = 0;
      bool     need_delay = false;
      if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
        // Only delay if it's not the very first retry attempt (count > 0)
        if (sd_card->error_handler.current_retry > 0) {
          delay_ms   = sd_card->error_handler.current_retry_delay;
          need_delay = true;
        }
        xSemaphoreGive(sd_card->mutex);
      } else {
        log_error(sd_card->tag, "Error Retry Error", "Failed to acquire mutex to get delay");
        continue; // Skip retry attempt if mutex fails
      }

      if (need_delay) {
        log_info(sd_card->tag,
                 "Error Retry",
                 "Delaying %lu ms before retry %lu/%lu",
                 delay_ms,
                 sd_card->error_handler.current_retry + 1, // Log next attempt number
                 sd_card->error_handler.max_retries);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
      }

      // Check exit request again after potential delay
      if (sd_card->mount_task_exit_requested) {
        break;
      }

      // Attempt recovery
      mutex_taken = false;
      if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
        mutex_taken = true;

        // Re-check conditions after acquiring mutex and potential delay
        card_physically_present = storage_sd_card_is_inserted(sd_card);
        if (sd_card->state == k_sd_state_error && card_physically_present &&
            error_handler_can_retry(&sd_card->error_handler)) {
          log_info(sd_card->tag,
                   "Error Recovery",
                   "Attempting to recover from error (retry %lu/%lu)",
                   sd_card->error_handler.current_retry + 1,
                   sd_card->error_handler.max_retries);

          // Attempt reset (which includes trying interfaces again)
          esp_err_t err = storage_sd_card_reset(sd_card);

          if (err != ESP_OK) {
            log_error(sd_card->tag,
                      "Reset Error",
                      "Failed to reset SD card: %s",
                      esp_err_to_name(err));
            // Error handler is called within storage_sd_card_reset if mount fails
            if (!error_handler_can_retry(&sd_card->error_handler)) {
              storage_update_state_machine(sd_card, k_sd_state_failed);
            }
            // Keep state as k_sd_state_error if retries remain
          }
          // State is updated within storage_sd_card_reset on success/failure
        } else {
          log_info(sd_card->tag,
                   "Error Retry Skip",
                   "Condition for retry no longer met after delay/mutex acquisition.");
        }
        storage_release_mutex_if_taken(sd_card, &mutex_taken);
      } else {
        log_error(sd_card->tag,
                  "Error Retry Error",
                  "Failed to acquire mutex for error retry attempt");
      }
    }
    // --- End Error Retry Logic ---

    if (sd_card->mount_task_exit_requested) {
      break;
    }

    // --- Performance Measurement ---
    if (sd_card->state == k_sd_state_interface_ready && atomic_load(&sd_card->card_available) &&
        sd_card->performance.measurement_needed) {
      mutex_taken = false;
      if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
        mutex_taken = true;
        // Check again after acquiring mutex
        if (sd_card->state == k_sd_state_interface_ready && atomic_load(&sd_card->card_available) &&
            sd_card->performance.measurement_needed) {
          storage_measure_card_performance(sd_card); // Clears measurement_needed flag
        }
        storage_release_mutex_if_taken(sd_card, &mutex_taken);
      }
    }
    // --- End Performance Measurement ---

    // --- Periodic Health Check ---
    if (sd_card->state == k_sd_state_interface_ready && atomic_load(&sd_card->card_available)) {
      static uint32_t health_check_counter = 0;
      // Check every ~10 seconds (100 * 100ms task delay)
      if (++health_check_counter >= 100) {
        health_check_counter = 0;
        mutex_taken          = false;
        if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
          mutex_taken = true;
          // Check again after acquiring mutex
          if (sd_card->state == k_sd_state_interface_ready &&
              atomic_load(&sd_card->card_available)) {
            struct stat st;
            if (stat(sd_card->mount_path, &st) != 0) {
              log_warn(
                sd_card->tag,
                "Health Check",
                "Failed to access SD card mount point - possible file system issue. Error: %s",
                strerror(errno));
              RECORD_ERROR(&sd_card->error_handler, errno, "Failed health check (stat failed)");
              storage_update_state_machine(sd_card, k_sd_state_error);
              // Unmount might fail if filesystem is corrupted, but try anyway
              sd_card_unmount(sd_card);
            } else {
              log_debug(sd_card->tag, "Health Check", "SD card mount point accessible.");
            }
          }
          storage_release_mutex_if_taken(sd_card, &mutex_taken);
        }
      }
    }
    // --- End Periodic Health Check ---

  } // End while loop

  log_info(sd_card->tag, "Mount Task Exiting", "SD card mount/unmount task received exit request");

  // Cleanup task resources before exiting
  sd_card->mount_task_handle = NULL; // Clear the handle *before* deleting self
  vTaskDelete(NULL);
}
