/* components/pstar_storage_hal/pstar_storage_sd_card_hal.c */

#include "pstar_storage_hal.h" // Include the main public header first

// Include necessary internal and dependency headers
#include "pstar_bus_config.h"
#include "pstar_bus_event_types.h"
#include "pstar_bus_gpio.h"
#include "pstar_bus_manager.h"
#include "pstar_log_handler.h"
#include "pstar_pin_validator.h"   // Needed for sd_card_init
#include "pstar_storage_common.h"  // Common helper functions
#include "pstar_storage_spi_hal.h" // SPI specific setup

#include "driver/gpio.h"
#include "driver/sdspi_host.h" // Specific SPI host driver for SD cards
#include "driver/spi_common.h" // Needed for SPI host defines
#include "freertos/FreeRTOS.h" // Needed for task functions
#include "freertos/task.h"     // Needed for task functions

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <limits.h> // For ULONG_MAX
#include <stdatomic.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_timer.h"
#include "esp_vfs_fat.h" // FAT filesystem support
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "sdmmc_cmd.h" // SDMMC commands and card structure

/* Constants ******************************************************************/
#define SD_INTERFACE_MAX_ERRORS (3) // Max errors before trying fallback (if enabled)
static const char* TAG = "SD Card HAL";

/* ISR/Task Functions - Forward Declarations **********************************/
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
static void priv_sd_card_detection_isr(void* arg);
#endif
static void priv_sd_card_mount_task(void* arg);

/* Public API Functions (Implementation) **************************************/

esp_err_t sd_card_mount(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) { // Check sd_card first
    log_error(TAG, "Mount Error", "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if already mounted */
  if (atomic_load(&sd_card->card_available)) {
    log_info(sd_card->tag, "Mount Info", "SD card already mounted");
    return ESP_OK;
  }

  /* Check if physically inserted *before* proceeding */
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

  /* Initialize FAT filesystem mount configuration */
  esp_vfs_fat_mount_config_t mount_config = {
    .format_if_mount_failed = false, // Do not format automatically
    .max_files              = sd_card->max_files,
    .allocation_unit_size   = sd_card->allocation_unit_size,
  };

  sdmmc_card_t* card_out = NULL;     // Pointer for the VFS mount function output
  esp_err_t     err      = ESP_FAIL; // Initialize error state

  // Determine mount function based on interface (only SPI for now)
  if (sd_card->current_interface == k_sd_interface_spi) {
    // --- Reconstruct host and device config needed for mount ---
    const char*         spi_bus_name = CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_BUS_NAME;
    pstar_bus_config_t* spi_bus_config =
      pstar_bus_manager_find_bus(&sd_card->bus_manager, spi_bus_name);
    if (spi_bus_config == NULL || spi_bus_config->type != k_pstar_bus_type_spi) {
      log_error(sd_card->tag,
                "Mount Error",
                "Could not find valid SPI bus config '%s' in manager for mount",
                spi_bus_name);
      return ESP_ERR_INVALID_STATE;
    }

    // Re-initialize device to get handle (or confirm it exists)
    sdspi_device_config_t device_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    device_config.host_id               = spi_bus_config->config.spi.host;
    device_config.gpio_cs               = spi_bus_config->config.spi.dev_config.spics_io_num;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
    device_config.gpio_cd = sd_card->pin_config.gpio_det_pin;
#else
    device_config.gpio_cd = SDSPI_SLOT_NO_CD;
#endif
    device_config.gpio_wp  = SDSPI_SLOT_NO_WP;
    device_config.gpio_int = SDSPI_SLOT_NO_INT;

    sdspi_dev_handle_t sdspi_handle;
    esp_err_t          init_dev_err = sdspi_host_init_device(&device_config, &sdspi_handle);
    if (init_dev_err != ESP_OK && init_dev_err != ESP_ERR_INVALID_STATE) {
      log_error(sd_card->tag,
                "Mount Error",
                "Failed to re-init SDSPI device to get handle: %s",
                esp_err_to_name(init_dev_err));
      return init_dev_err; // Propagate error if it wasn't just 'already init'
    }
    // ESP_ERR_INVALID_STATE is okay here, means device already exists, handle is likely valid

    sdmmc_host_t host_config = SDSPI_HOST_DEFAULT();
    host_config.slot         = sdspi_handle; // Use the retrieved/re-initialized handle

    // --- Let VFS allocate the card structure ---
    // sd_card->card should be NULL here or will be replaced.
    err = esp_vfs_fat_sdspi_mount(sd_card->mount_path,
                                  &host_config, // Pass the host config directly
                                  &device_config,
                                  &mount_config,
                                  &card_out); // VFS allocates and returns via card_out
  } else {
    log_error(sd_card->tag,
              "Mount Error",
              "Unsupported interface type for mounting: %d",
              sd_card->current_interface);
    return ESP_ERR_NOT_SUPPORTED;
  }

  if (err != ESP_OK) {
    if (err == ESP_FAIL) {
      log_error(sd_card->tag,
                "Mount Error",
                "Failed to mount filesystem. If card has FAT filesystem, try formatting it.");
    } else if (err == ESP_ERR_INVALID_STATE) {
      log_error(sd_card->tag, "Mount Error", "SD card already mounted or mount point busy.");
    } else if (err == ESP_ERR_NO_MEM) {
      log_error(sd_card->tag, "Mount Error", "Memory allocation failed during mount.");
    } else {
      log_error(sd_card->tag,
                "Mount Error",
                "Failed to initialize/mount SD card: %s (%d)",
                esp_err_to_name(err),
                err);
    }
    // VFS handles cleanup of card_out on error. Do not free sd_card->card.
    return err;
  }

  // --- Mount successful ---

  // --- Correctly handle the returned pointer ---
  if (sd_card->card != NULL && sd_card->card != card_out) {
    // This case should ideally not happen if we didn't pre-allocate.
    // If it does, it indicates a potential leak from a previous failed attempt.
    log_warn(sd_card->tag,
             "Mount Warning",
             "Pointer mismatch: sd_card->card (%p) was not NULL, VFS returned %p. Freeing old one.",
             sd_card->card,
             card_out);
    free(sd_card->card); // Free the old dangling pointer
  }
  sd_card->card = card_out; // Assign the pointer returned by VFS
  // --- End Pointer Handling ---

  if (sd_card->card == NULL) { // Defensive check
    log_error(sd_card->tag, "Mount Error", "Mount OK but VFS returned NULL card pointer!");
    // Attempt to unmount VFS just in case something is half-mounted
    esp_vfs_fat_sdcard_unmount(sd_card->mount_path, NULL); // Pass NULL as card is NULL
    return ESP_FAIL;
  }

  // --- Create /logs directory AFTER successful mount ---
  char logs_path[CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH];
  int  path_len = snprintf(logs_path, sizeof(logs_path), "%s/logs", sd_card->mount_path);
  if (path_len < 0 || (size_t)path_len >= sizeof(logs_path)) {
    log_warn(sd_card->tag, "Mount Warning", "Log path truncated, cannot create /logs directory.");
  } else {
    esp_err_t logs_dir_result = storage_create_directory_if_needed(sd_card, logs_path);
    if (logs_dir_result != ESP_OK) {
      log_warn(sd_card->tag,
               "Mount Warning",
               "Failed to create logs directory '%s': %s",
               logs_path,
               esp_err_to_name(logs_dir_result));
    } else {
      log_info(sd_card->tag, "Mount Info", "Ensured /logs directory exists at '%s'", logs_path);
    }
  }
  // --- End directory creation ---

  sdmmc_card_print_info(stdout, sd_card->card);
  log_info(sd_card->tag,
           "Mount Success",
           "SD card mounted at %s, %lluMB, Interface: %s",
           sd_card->mount_path,
           ((uint64_t)sd_card->card->csd.capacity * sd_card->card->csd.sector_size) / (1024 * 1024),
           sd_card_interface_to_string(sd_card->current_interface));

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

  // Store current interface *before* potentially changing state
  sd_interface_type_t interface_to_clean = sd_card->current_interface;

  if (!atomic_load(&sd_card->card_available)) {
    log_info(sd_card->tag, "Unmount Info", "SD card already unmounted");
    // Ensure card pointer is NULL even if already unmounted
    sd_card->card = NULL; // Just ensure it's NULL, don't free

    // Ensure SPI bus is cleaned up if needed
    if (interface_to_clean == k_sd_interface_spi) { // Check the stored interface
      const char*         spi_bus_name = CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_BUS_NAME;
      pstar_bus_config_t* spi_bus = pstar_bus_manager_find_bus(&sd_card->bus_manager, spi_bus_name);
      if (spi_bus != NULL) {
        log_info(sd_card->tag,
                 "Unmount Cleanup",
                 "Removing SPI bus '%s' (already unmounted).",
                 spi_bus_name);
        pstar_bus_manager_remove_bus(&sd_card->bus_manager, spi_bus_name);
      }
    }
    sd_card->current_interface = k_sd_interface_none; // Ensure interface is none
    return ESP_OK;
  }

  log_info(sd_card->tag,
           "Unmount Started",
           "Unmounting SD card from path: %s",
           sd_card->mount_path);

  esp_err_t vfs_err = ESP_OK;
  esp_err_t bus_err = ESP_OK;

  // Mark unavailable and notify BEFORE freeing resources
  atomic_store(&sd_card->card_available, false);
  storage_notify_availability(sd_card, false);

  // --- Call VFS unmount FIRST ---
  if (sd_card->card != NULL) {
    // --- Store pointer temporarily, call unmount, then NULL the HAL pointer ---
    sdmmc_card_t* card_to_unmount = sd_card->card;
    sd_card->card                 = NULL; // Set HAL pointer to NULL *before* calling unmount
    vfs_err = esp_vfs_fat_sdcard_unmount(sd_card->mount_path, card_to_unmount);
    // --- Assume esp_vfs_fat_sdcard_unmount frees card_to_unmount ---

    if (vfs_err != ESP_OK) {
      // --- FIX: Log the actual error from VFS unmount ---
      log_error(sd_card->tag,
                "VFS Unmount Failed",
                "esp_vfs_fat_sdcard_unmount returned: %s",
                esp_err_to_name(vfs_err));
      // Continue with cleanup even if VFS unmount fails
    } else {
      log_info(sd_card->tag,
               "VFS Unmount OK",
               "VFS unmount successful for %s",
               sd_card->mount_path);
    }
    // --- REMOVED the explicit free here ---
  } else {
    log_warn(sd_card->tag, "Unmount Warning", "Card pointer was NULL during VFS unmount");
    // Attempt VFS unmount with NULL card pointer just in case
    vfs_err = esp_vfs_fat_sdcard_unmount(sd_card->mount_path, NULL);
    if (vfs_err != ESP_OK && vfs_err != ESP_ERR_INVALID_STATE) {
      log_warn(sd_card->tag,
               "Unmount Warning",
               "VFS unmount with NULL card failed: %s",
               esp_err_to_name(vfs_err));
    }
  }
  // --- End VFS Unmount ---

  // --- Bus Cleanup AFTER VFS unmount ---
  if (interface_to_clean == k_sd_interface_spi) {
    const char* spi_bus_name = CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_BUS_NAME;
    log_info(sd_card->tag,
             "Unmount SPI Cleanup",
             "Removing SPI bus '%s' via Bus Manager.",
             spi_bus_name);
    bus_err = pstar_bus_manager_remove_bus(&sd_card->bus_manager, spi_bus_name);
    if (bus_err != ESP_OK && bus_err != ESP_ERR_NOT_FOUND) {
      log_warn(sd_card->tag,
               "Unmount SPI Cleanup",
               "Failed to remove/deinit SPI bus '%s': %s",
               spi_bus_name,
               esp_err_to_name(bus_err));
      // Continue cleanup
    } else if (bus_err == ESP_OK) {
      log_info(sd_card->tag,
               "Unmount SPI Cleanup",
               "Successfully removed and deinitialized SPI bus '%s'.",
               spi_bus_name);
    } else {
      log_info(sd_card->tag,
               "Unmount SPI Cleanup",
               "SPI bus '%s' was not found in manager (already removed?).",
               spi_bus_name);
      bus_err = ESP_OK;
    }
  }
  // --- End Bus Cleanup ---

  sd_card->current_interface = k_sd_interface_none; // Reset interface type

  log_info(sd_card->tag,
           "Unmount Success",
           "SD card unmounted and interface cleaned up successfully");

  // Return the first error encountered (prefer VFS error over bus error if both occur)
  return (vfs_err != ESP_OK) ? vfs_err : bus_err;
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

  // Determine the correct GPIO bus name based on context (needs refinement for multi-instance)
  const char* gpio_bus_name = CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME; // Assume default

  pstar_bus_config_t* gpio_config =
    pstar_bus_manager_find_bus(&sd_card->bus_manager, gpio_bus_name);

  if (gpio_config == NULL) {
    log_info(sd_card->tag,
             "Detection Setup",
             "Creating GPIO bus '%s' for card detection.",
             gpio_bus_name);
    gpio_config = pstar_bus_config_create_gpio(gpio_bus_name, k_pstar_mode_interrupt);
    if (gpio_config == NULL) {
      log_error(sd_card->tag, "Detection Setup Error", "Failed to create GPIO bus configuration");
      return ESP_ERR_NO_MEM;
    }

    gpio_config->config.gpio.config.pin_bit_mask = (1ULL << sd_card->pin_config.gpio_det_pin);
    gpio_config->config.gpio.config.mode         = GPIO_MODE_INPUT;
    gpio_config->config.gpio.config.pull_up_en =
      sd_card->card_detect_low_active ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    gpio_config->config.gpio.config.pull_down_en =
      sd_card->card_detect_low_active ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
    gpio_config->config.gpio.config.intr_type = GPIO_INTR_ANYEDGE;

    pstar_bus_gpio_init_default_ops(&gpio_config->config.gpio.ops);

    esp_err_t err = pstar_bus_manager_add_bus(&sd_card->bus_manager, gpio_config);
    if (err != ESP_OK) {
      log_error(sd_card->tag,
                "Detection Setup Error",
                "Failed to add GPIO bus '%s' to manager: %s",
                gpio_bus_name,
                esp_err_to_name(err));
      pstar_bus_config_destroy(gpio_config); // Destroy config if add fails
      gpio_config = NULL;                    // Avoid dangling pointer
      return err;
    }

    err = pstar_bus_config_init(gpio_config);
    if (err != ESP_OK) {
      log_error(sd_card->tag,
                "Detection Setup Error",
                "Failed to initialize GPIO bus '%s': %s",
                gpio_bus_name,
                esp_err_to_name(err));
      // Remove bus also destroys config
      pstar_bus_manager_remove_bus(&sd_card->bus_manager, gpio_bus_name);
      return err;
    }
  } else {
    if (gpio_config->type != k_pstar_bus_type_gpio) {
      log_error(sd_card->tag,
                "Detection Setup Error",
                "Bus '%s' exists but is not a GPIO bus.",
                gpio_bus_name);
      return ESP_ERR_INVALID_STATE;
    }
    log_info(sd_card->tag,
             "Detection Setup",
             "Using existing GPIO bus '%s' for card detection.",
             gpio_bus_name);
  }

  esp_err_t isr_err = pstar_bus_gpio_isr_add(&sd_card->bus_manager,
                                             gpio_bus_name,
                                             sd_card->pin_config.gpio_det_pin,
                                             priv_sd_card_detection_isr,
                                             (void*)sd_card);

  // ESP_ERR_INVALID_STATE means ISR handler already added for this pin, which is okay if re-initializing
  if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
    log_error(sd_card->tag,
              "Detection Setup Error",
              "Failed to add ISR handler for pin %d on bus '%s': %s",
              sd_card->pin_config.gpio_det_pin,
              gpio_bus_name,
              esp_err_to_name(isr_err));
    // Only remove the bus if we just added it and ISR add failed
    if (pstar_bus_manager_find_bus(&sd_card->bus_manager, gpio_bus_name) == gpio_config &&
        gpio_config != NULL) { // Check if we actually created it this time
      pstar_bus_manager_remove_bus(&sd_card->bus_manager, gpio_bus_name);
    }
    return isr_err;
  } else if (isr_err == ESP_ERR_INVALID_STATE) {
    log_warn(sd_card->tag,
             "Detection Setup Info",
             "ISR handler already added for pin %d.",
             sd_card->pin_config.gpio_det_pin);
  } else {
    log_info(sd_card->tag,
             "ISR Added",
             "Added ISR handler for pin %d on bus '%s'",
             sd_card->pin_config.gpio_det_pin,
             gpio_bus_name);
  }

  log_info(sd_card->tag,
           "Detection Setup Complete",
           "Card detection mechanism configured successfully with %s-active pin %d on bus '%s'",
           sd_card->card_detect_low_active ? "low" : "high",
           sd_card->pin_config.gpio_det_pin,
           gpio_bus_name);
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

  bool spi_supported = false;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  spi_supported = storage_spi_is_supported();
#endif

  if (!spi_supported) {
    log_error(sd_card->tag,
              "Interface Error",
              "SPI interface is not enabled in Kconfig or not supported.");
    return ESP_ERR_NOT_SUPPORTED;
  }

  sd_card->interface_attempt_count++;
  esp_err_t last_error = ESP_FAIL;

  log_info(sd_card->tag,
           "Interface Discovery",
           "Starting SPI interface attempt %lu",
           sd_card->interface_attempt_count);

  // --- Cleanup previous interface attempt if any ---
  // Reset the current interface. Unmount handles freeing card struct if needed.
  if (atomic_load(&sd_card->card_available)) {
    sd_card_unmount(sd_card); // Unmount handles bus cleanup and freeing card
  } else if (sd_card->card != NULL) {
    // If not available but card struct exists (e.g., failed mount), free it
    free(sd_card->card);
    sd_card->card = NULL;
  }
  sd_card->current_interface = k_sd_interface_none;
  // --- End Cleanup ---

  log_info(sd_card->tag, "Interface Try", "Attempting SPI interface");
  sd_card->interface_info[k_sd_interface_spi].attempted = true;

  esp_err_t setup_err = storage_spi_setup(sd_card); // Does NOT allocate sd_card->card anymore

  if (setup_err != ESP_OK) {
    log_warn(sd_card->tag,
             "Interface Failure",
             "Failed to setup SPI: %s",
             esp_err_to_name(setup_err));
    last_error = setup_err;
    sd_card->interface_info[k_sd_interface_spi].error_count++;
    // No card struct to free here
    return last_error;
  }

  // If setup succeeded, try to mount. Mount allocates sd_card->card on success.
  sd_card->current_interface = k_sd_interface_spi;
  esp_err_t mount_err        = sd_card_mount(sd_card); // Uses info from setup, allocates card

  if (mount_err != ESP_OK) {
    log_warn(sd_card->tag,
             "Interface Failure",
             "Failed to mount with SPI: %s",
             esp_err_to_name(mount_err));
    last_error = mount_err;
    sd_card->interface_info[k_sd_interface_spi].error_count++;
    sd_card->current_interface = k_sd_interface_none;
    // Clean up the SPI bus if mount fails (unmount handles this)
    // sd_card->card should be NULL already or freed by mount failure path
    sd_card_unmount(sd_card); // Call unmount to ensure bus is cleaned up
    return last_error;
  }

  log_info(sd_card->tag, "Interface Success", "Successfully mounted SD card with SPI");
  sd_card->interface_info[k_sd_interface_spi].successful        = true;
  sd_card->interface_info[k_sd_interface_spi].last_success_time = esp_timer_get_time();
  sd_card->interface_info[k_sd_interface_spi].error_count       = 0;
  sd_card->interface_discovery_complete                         = true;
  return ESP_OK;
}

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

  // Check if *any* interface is enabled (currently only SPI)
  bool spi_supported = false;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  spi_supported = storage_spi_is_supported();
#endif

  if (!spi_supported) {
    log_error(sd_card->tag,
              "Init Error",
              "Cannot initialize SD Card HAL: No supported interfaces enabled (SPI disabled?).");
    return ESP_ERR_NOT_SUPPORTED;
  }

  log_info(sd_card->tag, "Init Started", "Initializing SD card HAL");

  // --- Create Semaphore FIRST ---
  sd_card->mount_task_exit_sem = xSemaphoreCreateBinary();
  if (sd_card->mount_task_exit_sem == NULL) {
    log_error(TAG, "Init Error", "Failed to create mount task exit semaphore");
    // Cleanup previously created resources (mutex, bus manager, error handler)
    if (sd_card->mutex) {
      vSemaphoreDelete(sd_card->mutex);
      sd_card->mutex = NULL;
    }
    pstar_bus_manager_deinit(&sd_card->bus_manager);
    error_handler_deinit(&sd_card->error_handler);
    return ESP_ERR_NO_MEM;
  }
  // --- End Semaphore Creation ---

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  /* Set up card detection GPIO */
  esp_err_t det_err = sd_card_setup_detection(sd_card);
  if (det_err != ESP_OK) {
    log_error(sd_card->tag,
              "Init Error",
              "Failed to set up card detection: %s",
              esp_err_to_name(det_err));
    // Cleanup semaphore, mutex, bus manager, error handler
    if (sd_card->mount_task_exit_sem) {
      vSemaphoreDelete(sd_card->mount_task_exit_sem);
      sd_card->mount_task_exit_sem = NULL;
    }
    if (sd_card->mutex) {
      vSemaphoreDelete(sd_card->mutex);
      sd_card->mutex = NULL;
    }
    pstar_bus_manager_deinit(&sd_card->bus_manager);
    error_handler_deinit(&sd_card->error_handler);
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
    // Cleanup detection ISR if setup succeeded before pin registration failed
    if (sd_card->pin_config.gpio_det_pin >= 0) {
      const char* gpio_bus_name = CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME;
      pstar_bus_gpio_isr_remove(&sd_card->bus_manager,
                                gpio_bus_name,
                                sd_card->pin_config.gpio_det_pin);
    }
#endif
    // General cleanup including semaphore
    if (sd_card->mount_task_exit_sem) {
      vSemaphoreDelete(sd_card->mount_task_exit_sem);
      sd_card->mount_task_exit_sem = NULL;
    }
    if (sd_card->mutex) {
      vSemaphoreDelete(sd_card->mutex);
      sd_card->mutex = NULL;
    }
    pstar_bus_manager_deinit(&sd_card->bus_manager);
    error_handler_deinit(&sd_card->error_handler);
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
    // Cleanup detection ISR
    if (sd_card->pin_config.gpio_det_pin >= 0) {
      const char* gpio_bus_name = CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME;
      pstar_bus_gpio_isr_remove(&sd_card->bus_manager,
                                gpio_bus_name,
                                sd_card->pin_config.gpio_det_pin);
    }
#endif
    // General cleanup including semaphore
    if (sd_card->mount_task_exit_sem) {
      vSemaphoreDelete(sd_card->mount_task_exit_sem);
      sd_card->mount_task_exit_sem = NULL;
    }
    if (sd_card->mutex) {
      vSemaphoreDelete(sd_card->mutex);
      sd_card->mutex = NULL;
    }
    pstar_bus_manager_deinit(&sd_card->bus_manager);
    error_handler_deinit(&sd_card->error_handler);
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
    // Cannot give semaphore if sd_card is NULL
    vTaskDelete(NULL);
    return;
  }

  // Check if SD card support is enabled globally
#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  log_warn(sd_card->tag,
           "Mount Task Info",
           "SD Card support disabled globally. Exiting mount task.");
  sd_card->mount_task_handle = NULL;
  // Give semaphore before exiting
  if (sd_card->mount_task_exit_sem != NULL) {
    xSemaphoreGive(sd_card->mount_task_exit_sem);
  }
  vTaskDelete(NULL);
  return;
#endif

  // Check if *any* interface is enabled (currently only SPI)
  bool spi_supported = false;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  spi_supported = storage_spi_is_supported();
#endif
  if (!spi_supported) {
    log_warn(sd_card->tag,
             "Mount Task Info",
             "No SD Card interfaces enabled (SPI disabled). Exiting mount task.");
    sd_card->mount_task_handle = NULL;
    // Give semaphore before exiting
    if (sd_card->mount_task_exit_sem != NULL) {
      xSemaphoreGive(sd_card->mount_task_exit_sem);
    }
    vTaskDelete(NULL);
    return;
  }

  log_info(sd_card->tag, "Mount Task Started", "SD card mount/unmount task is running");

  bool mutex_taken = false;

  // Safely initialize state within mutex if not already done
  if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
    mutex_taken = true;
    if (sd_card->state == 0 && sd_card->last_state_change_time == 0) { // Check if uninitialized
      sd_card->state                  = k_sd_state_idle;
      sd_card->last_state_change_time = esp_timer_get_time();
    }
    sd_card->mount_task_exit_requested = false;
    storage_release_mutex_if_taken(sd_card, &mutex_taken);
  } else {
    log_error(sd_card->tag, "Mount Task Error", "Failed to acquire mutex for initial state check.");
    // Proceed cautiously, state might be inconsistent
  }

  // Try loading previous config
  storage_load_working_config(sd_card); // Handles its own mutex

  bool is_inserted;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  is_inserted = storage_sd_card_is_inserted(sd_card);        // Handles its own mutex
  TickType_t last_debounce_check_time = xTaskGetTickCount(); // For debounce timing
#else
  is_inserted = true; // Assume inserted if detection disabled
#endif

  // Initial check on task start
  if (is_inserted) {
    log_info(sd_card->tag, "Initial Detection", "SD card detected at startup");

    // Logic for initial mount
    bool initial_mount_needed = false;
    mutex_taken               = false;
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
      storage_update_state_machine(sd_card,
                                   k_sd_state_error); // Update state to ERROR (takes/gives mutex)
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
          RECORD_ERROR(&sd_card->error_handler, err, "Initial SD card mount failed");
          if (!error_handler_can_retry(&sd_card->error_handler)) {
            storage_update_state_machine(sd_card, k_sd_state_failed);
          }
        } else if (atomic_load(&sd_card->card_available)) {
          storage_update_state_machine(sd_card, k_sd_state_interface_ready);
        } else {
          log_warn(sd_card->tag,
                   "Initial Mount Status",
                   "try_interfaces OK, but card not available.");
          storage_update_state_machine(sd_card, k_sd_state_error);
          RECORD_ERROR(&sd_card->error_handler,
                       ESP_FAIL,
                       "Initial mount OK but card not available");
          if (!error_handler_can_retry(&sd_card->error_handler)) {
            storage_update_state_machine(sd_card, k_sd_state_failed);
          }
        }
        storage_release_mutex_if_taken(sd_card, &mutex_taken);
      } else {
        log_error(sd_card->tag,
                  "Initial Mount State Error",
                  "Failed to acquire mutex to update state after initial mount attempt.");
      }
    }

  } else {
    log_info(sd_card->tag, "Initial Detection", "No SD card detected at startup");
    // Ensure state is idle if no card detected initially
    storage_update_state_machine(sd_card, k_sd_state_idle);
  }

  // Main task loop
  while (!sd_card->mount_task_exit_requested) {
    // Wait for ISR notification or timeout
    uint32_t   notification_value = 0; // Declare the variable
    BaseType_t notification_received =
      xTaskNotifyWait(0, ULONG_MAX, &notification_value, pdMS_TO_TICKS(100));

    if (sd_card->mount_task_exit_requested) {
      break;
    }

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
    // --- Card Detection Logic (Only if enabled) ---
    bool current_inserted_state = storage_sd_card_is_inserted(sd_card); // Handles mutex
    bool previous_available_state =
      atomic_load(&sd_card->card_available); // Get the current mount status

    // Check if the physical state differs from the logical mount state OR if ISR notified
    if (current_inserted_state != previous_available_state || notification_received == pdTRUE) {
      TickType_t now = xTaskGetTickCount();
      // Check if debounce time has passed since last check OR if ISR triggered immediate check
      if (notification_received == pdTRUE ||
          (now - last_debounce_check_time) >=
            pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_SD_CARD_DEBOUNCE_MS)) {
        last_debounce_check_time = now; // Update last check time

        // Re-read state *after* debounce period (or immediately on ISR)
        current_inserted_state = storage_sd_card_is_inserted(sd_card);

        // Only proceed if the debounced state is different from the logical available state
        if (current_inserted_state != previous_available_state) {
          log_info(sd_card->tag,
                   "Debounce Complete",
                   "Confirmed card status change: inserted=%d, previously_available=%d",
                   current_inserted_state,
                   previous_available_state);

          bool mount_needed   = false;
          bool unmount_needed = false;

          mutex_taken = false;
          if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
            mutex_taken = true;

            // Re-check states *after* acquiring mutex
            bool current_inserted_state_locked   = storage_sd_card_is_inserted(sd_card);
            bool previous_available_state_locked = atomic_load(&sd_card->card_available);

            if (current_inserted_state_locked && !previous_available_state_locked) {
              log_info(sd_card->tag, "Mount Trigger", "Card inserted, preparing for mount...");
              storage_update_state_machine(sd_card, k_sd_state_card_inserted);
              storage_update_state_machine(sd_card, k_sd_state_interface_discovery);
              mount_needed = true;
            } else if (!current_inserted_state_locked && previous_available_state_locked) {
              log_info(sd_card->tag, "Unmount Trigger", "Card removed, preparing for unmount...");
              unmount_needed = true;
            } else {
              log_debug(sd_card->tag,
                        "Debounce",
                        "Card status consistent after mutex lock, no action needed.");
            }
            storage_release_mutex_if_taken(sd_card, &mutex_taken);
          } else {
            log_error(sd_card->tag,
                      "Card State Change Error",
                      "Failed to acquire mutex for card state change handling");
          }

          // Perform mount or unmount *outside* the initial mutex lock if needed
          if (mount_needed) {
            esp_err_t err = sd_card_try_interfaces(sd_card); // Handles its own mutex needs

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
              } else if (atomic_load(&sd_card->card_available)) {
                storage_update_state_machine(sd_card, k_sd_state_interface_ready);
                error_handler_reset_state(&sd_card->error_handler); // Reset errors on success
              } else {
                log_warn(sd_card->tag,
                         "Mount Status",
                         "try_interfaces returned OK, but card not available.");
                RECORD_ERROR(&sd_card->error_handler, ESP_FAIL, "Mount OK but card not available");
                storage_update_state_machine(sd_card, k_sd_state_error);
                if (!error_handler_can_retry(&sd_card->error_handler)) {
                  storage_update_state_machine(sd_card, k_sd_state_failed);
                }
              }
              storage_release_mutex_if_taken(sd_card, &mutex_taken);
            } else {
              log_error(sd_card->tag,
                        "Mount State Error",
                        "Failed to acquire mutex to update state after mount attempt.");
            }
          } else if (unmount_needed) {
            esp_err_t err = sd_card_unmount(sd_card); // Handles its own mutex needs

            mutex_taken = false;
            if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
              mutex_taken = true;
              if (err != ESP_OK) {
                log_error(sd_card->tag,
                          "Unmount Error",
                          "Failed to unmount SD card: %s",
                          esp_err_to_name(err));
                // Optionally record unmount error? Depends if it's critical.
              }
              storage_update_state_machine(sd_card, k_sd_state_idle);
              sd_card->interface_discovery_complete = false; // Ready for next insertion
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
        } // End check if debounced state differs from available state
      } // End debounce time check
    } // End check if physical state differs OR notification received
    // --- End Card Detection Logic ---
#endif // CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED

    if (sd_card->mount_task_exit_requested) {
      break;
    }

    // --- Error Retry Logic ---
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED // Only run retry if SD enabled
    bool card_physically_present = storage_sd_card_is_inserted(sd_card); // Handles mutex
    mutex_taken                  = false;
    bool     attempt_retry       = false;
    uint32_t delay_ms            = 0;

    // Check conditions for retry attempt safely
    if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
      mutex_taken = true;
      // Only retry if in ERROR state, card is present, and retries remain
      if (sd_card->state == k_sd_state_error && card_physically_present &&
          error_handler_can_retry(&sd_card->error_handler)) {
        // Get the delay *before* potentially calling the reset function which increments retry count
        delay_ms      = sd_card->error_handler.current_retry_delay;
        attempt_retry = true;
      }
      storage_release_mutex_if_taken(sd_card, &mutex_taken);
    } else {
      log_error(sd_card->tag,
                "Error Retry Error",
                "Failed to acquire mutex to check retry conditions");
    }

    if (attempt_retry) {
      // Apply delay based on the *previous* error's delay calculation
      if (delay_ms > 0) {
        log_info(sd_card->tag,
                 "Error Retry",
                 "Delaying %lu ms before retry %lu/%lu",
                 delay_ms,
                 sd_card->error_handler.current_retry + 1, // Log next attempt number
                 sd_card->error_handler.max_retries);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
      }

      if (sd_card->mount_task_exit_requested)
        break; // Check exit request again

      // Attempt recovery (reset function handles mutex internally and increments retry count via RECORD_ERROR if it fails)
      log_info(sd_card->tag,
               "Error Recovery",
               "Attempting to recover from error (retry %lu/%lu)",
               sd_card->error_handler.current_retry + 1, // Log next attempt number
               sd_card->error_handler.max_retries);

      // The reset function calls try_interfaces which calls mount.
      // Mount failure will call RECORD_ERROR, updating the error handler state.
      esp_err_t reset_err = storage_sd_card_reset(sd_card);

      // Check state AFTER reset attempt
      mutex_taken = false;
      if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
        mutex_taken = true;
        if (reset_err == ESP_OK && atomic_load(&sd_card->card_available)) {
          // Success! State should be INTERFACE_READY (set inside reset)
          // error_handler_reset_state was called inside reset on success path.
        } else {
          // Reset failed or didn't result in mount
          log_error(sd_card->tag, "Reset Failed", "SD card reset/remount failed or did not mount.");
          // Error handler retry count was already incremented by the failure inside reset/mount.
          // Check if retries are now exhausted.
          if (!error_handler_can_retry(&sd_card->error_handler)) {
            storage_update_state_machine(sd_card, k_sd_state_failed);
          } else {
            // Still in error state, but retries remain
            storage_update_state_machine(sd_card, k_sd_state_error);
          }
        }
        storage_release_mutex_if_taken(sd_card, &mutex_taken);
      } else {
        log_error(sd_card->tag,
                  "Reset State Error",
                  "Failed to acquire mutex after reset attempt.");
      }
    }
    // --- End Error Retry Logic ---
#endif // CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED

    if (sd_card->mount_task_exit_requested) {
      break;
    }

    // --- Performance Measurement ---
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED // Only run if SD enabled
    mutex_taken            = false;
    bool needs_measurement = false;
    if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
      mutex_taken = true;
      if (sd_card->state == k_sd_state_interface_ready && atomic_load(&sd_card->card_available) &&
          sd_card->performance.measurement_needed) {
        needs_measurement                       = true;
        sd_card->performance.measurement_needed = false; // Mark as handled
      }
      storage_release_mutex_if_taken(sd_card, &mutex_taken);
    }
    if (needs_measurement) {
      storage_measure_card_performance(sd_card); // Handles its own mutex
    }
    // --- End Performance Measurement ---
#endif // CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED

    // --- Periodic Health Check ---
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED // Only run if SD enabled
    static uint32_t health_check_counter = 0;
    mutex_taken                          = false;
    bool check_health                    = false;
    if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
      mutex_taken = true;
      if (sd_card->state == k_sd_state_interface_ready && atomic_load(&sd_card->card_available)) {
        if (++health_check_counter >= 100) { // Check every ~10 seconds
          health_check_counter = 0;
          check_health         = true;
        }
      } else {
        health_check_counter = 0; // Reset counter if not ready
      }
      storage_release_mutex_if_taken(sd_card, &mutex_taken);
    }

    if (check_health) {
      struct stat st;
      // Construct full path for stat
      char stat_path[CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH];
      snprintf(stat_path, sizeof(stat_path), "%s", sd_card->mount_path);
      stat_path[sizeof(stat_path) - 1] = '\0'; // Ensure null termination

      if (stat(stat_path, &st) != 0) {
        log_warn(sd_card->tag,
                 "Health Check",
                 "Failed to access SD card mount point '%s': %s. Triggering error state.",
                 stat_path,
                 strerror(errno));
        RECORD_ERROR(
          &sd_card->error_handler,
          errno,
          "Failed health check (stat failed)"); // Records error, increments retry count etc.

        // Update state machine AFTER recording error
        mutex_taken = false;
        if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
          mutex_taken = true;
          storage_update_state_machine(sd_card, k_sd_state_error);
          if (!error_handler_can_retry(&sd_card->error_handler)) {
            storage_update_state_machine(sd_card, k_sd_state_failed);
          }
          storage_release_mutex_if_taken(sd_card, &mutex_taken);
        } else {
          log_error(sd_card->tag,
                    "Health Check Error",
                    "Failed to acquire mutex to set error state after health check failure.");
        }
        // Attempt unmount (best effort)
        sd_card_unmount(sd_card); // Handles its own state changes
      } else {
        log_debug(sd_card->tag, "Health Check", "SD card mount point '%s' accessible.", stat_path);
      }
    }
    // --- End Periodic Health Check ---
#endif // CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED

  } // End while loop

  log_info(sd_card->tag, "Mount Task Exiting", "SD card mount/unmount task received exit request");

  // --- Give semaphore before deleting self ---
  if (sd_card->mount_task_exit_sem != NULL) {
    xSemaphoreGive(sd_card->mount_task_exit_sem);
  }
  // --- End Give Semaphore ---

  sd_card->mount_task_handle = NULL; // Clear the handle *before* deleting self
  vTaskDelete(NULL);                 // Delete self LAST
}