/* components/pstar_storage_hal/pstar_storage_hal.c */

#include "pstar_storage_hal.h"
#include "pstar_storage_common.h"
#include "pstar_storage_spi_hal.h"
#include "pstar_storage_sdio_hal.h"

#include "pstar_log_handler.h"
#include "pstar_pin_validator.h"

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

/* Constants ******************************************************************/
static const char* TAG = "Storage HAL";

/* Public Functions ***********************************************************/

const char* sd_card_interface_to_string(sd_interface_type_t interface_type)
{
  switch (interface_type) {
    case k_sd_interface_sdio:
      return "SDIO";
    case k_sd_interface_spi:
      return "SPI";
    case k_sd_interface_none:
      return "None";
    default:
      return "Unknown";
  }
}

esp_err_t sd_card_init_with_pins(sd_card_hal_t*              sd_card,
                                 const char*                 tag,
                                 const char*                 mount_path,
                                 const char*                 component_id,
                                 sd_bus_width_t              bus_width,
                                 const sd_card_pin_config_t* pin_config)
{
  /* Validate arguments */
  if (sd_card == NULL || tag == NULL || mount_path == NULL || component_id == NULL ||
      pin_config == NULL) { /* Use log_error */
    log_error(TAG, "Init Error", "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  /* First initialize with default settings */
  esp_err_t err = sd_card_init_default(sd_card, tag, mount_path, component_id, bus_width);
  if (err != ESP_OK) {
    return err;
  }

  /* Store custom pin configuration */
  memcpy(&sd_card->pin_config, pin_config, sizeof(sd_card_pin_config_t));

/* Check for pin conflicts with custom pins */
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  if (bus_width == k_sd_bus_width_4bit && sd_card->pin_config.gpio_det_pin >= 0 &&
      sd_card->pin_config.gpio_det_pin == sd_card->pin_config.sdio_d3_pin) {
    log_warn(sd_card->tag, /* Use log_warn */
             "Pin Conflict",
             "Card detect pin (GPIO %d) conflicts with SDIO D3 for 4-bit mode. "
             "Will default to 1-bit mode for better compatibility.",
             sd_card->pin_config.gpio_det_pin);

    /* Default to 1-bit mode when there's a pin conflict */
    sd_card->bus_width = k_sd_bus_width_1bit;
  }
#endif

  /* Defer detailed pin validation to registration */

  log_info(sd_card->tag, /* Use log_info */
           "Custom Pins",
           "SD card initialized with custom pin configuration");

  return ESP_OK;
}

esp_err_t sd_card_init_default(sd_card_hal_t* sd_card,
                               const char*    tag,
                               const char*    mount_path,
                               const char*    component_id,
                               sd_bus_width_t bus_width)
{
  esp_err_t err = ESP_OK;
  /* Validate arguments */
  if (sd_card == NULL || tag == NULL || mount_path == NULL ||
      component_id == NULL) { /* Use log_error */
    log_error(TAG,
              "Invalid arguments",
              "sd_card: %p, tag: %s, mount_path: %s, component_id: %s",
              sd_card,
              tag,
              mount_path,
              component_id);
    return ESP_ERR_INVALID_ARG;
  }

  /* Validate bus width */
  if (bus_width != k_sd_bus_width_1bit && bus_width != k_sd_bus_width_4bit) {
    log_error(TAG, /* Use log_error */
              "Invalid bus width",
              "Bus width must be 1 or 4, got: %d",
              bus_width);
    return ESP_ERR_INVALID_ARG;
  }

  /* Validate mount path using the simpler path check */
  if (!storage_path_is_safe_simple(mount_path)) {
    log_error(TAG, /* Use log_error */
              "Invalid mount path",
              "Mount path '%s' is not safe",
              mount_path);
    return ESP_ERR_INVALID_ARG;
  }

  /* Determine preferred interface from Kconfig */
  sd_interface_type_t preferred_interface;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_INTERFACE_SDIO
  preferred_interface = k_sd_interface_sdio;
#elif defined(CONFIG_PSTAR_KCONFIG_SD_CARD_INTERFACE_SPI)
  preferred_interface = k_sd_interface_spi;
#else
#error "No preferred SD card interface selected in Kconfig"
  preferred_interface = k_sd_interface_none; /* Should not happen */
#endif

  /* Cache enabled modes */
  bool sdio_enabled = storage_sdio_is_supported();
  bool spi_enabled = storage_spi_is_supported();

  /* Check if preferred interface is actually enabled */
  if ((preferred_interface == k_sd_interface_sdio && !sdio_enabled) ||
      (preferred_interface == k_sd_interface_spi && !spi_enabled)) {
    log_warn(TAG,
             "Interface Config Warning",
             "Preferred interface (%s) is disabled. Adjusting preference.",
             sd_card_interface_to_string(preferred_interface));
    if (sdio_enabled) {
      preferred_interface = k_sd_interface_sdio;
    } else if (spi_enabled) {
      preferred_interface = k_sd_interface_spi;
    } else {
      preferred_interface = k_sd_interface_none; /* No interfaces enabled */
    }
  }

  /* Default task configuration */
  sd_card_task_config_t task_config = {
    .stack_size    = CONFIG_PSTAR_KCONFIG_SD_CARD_TASK_STACK_SIZE,
    .priority      = CONFIG_PSTAR_KCONFIG_SD_CARD_TASK_PRIORITY,
    .mutex_timeout = pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_SD_CARD_MUTEX_TIMEOUT_MS),
  };

  /* Initialize performance metrics */
  sd_card_performance_t performance = {
    .last_measured      = 0,
    .read_speed_kbps    = 0,
    .write_speed_kbps   = 0,
    .measurement_needed = false,
  };

  /* Default Configuration */
  sd_card_hal_t sd_card_default = {
    .tag                  = tag,
    .mount_path           = mount_path,
    .max_files            = CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_FILES,
    .allocation_unit_size = CONFIG_PSTAR_KCONFIG_SD_CARD_ALLOCATION_UNIT_SIZE,
    .card_detect_low_active =
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
#if CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ACTIVE_LOW
      true,
#else
      false,
#endif
#else
      false,
#endif
    .bus_width           = bus_width, /* Use provided bus width */
    .pin_config          = sd_card_default_pins, /* Use default pin configuration */
    .preferred_interface = preferred_interface,
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLE_INTERFACE_FALLBACK
    .enable_fallback = true,
#else
    .enable_fallback = false,
#endif
    .sdio_mode_enabled            = sdio_enabled,
    .spi_mode_enabled             = spi_enabled,
    .state                        = k_sd_state_idle,      /* Initial state */
    .error_count                  = 0,                    /* No errors initially */
    .last_state_change_time       = 0,                    /* Will be set during init */
    .current_interface            = k_sd_interface_none,
    .interface_info               = {{0}, {0}, {0}}, /* Initialize all interface info */
    .interface_discovery_complete = false,
    .interface_attempt_count      = 0,
    .card                         = NULL, /* Will call sdmmc_card_init() */
    .mutex                        = NULL, /* Will call xSemaphoreCreateMutex() */
    .initialized                  = false,
    .mount_task_exit_requested    = false,
    .mount_task_handle            = NULL, /* Will call xTaskCreate() */
    .error_handler                = {0},  /* Will call error_handler_init() */
    .component_id                 = component_id,
    .bus_manager                  = {0},         /* Will call bus_manager_init() */
    .task_config                  = task_config, /* Use default task configuration */
    .performance                  = performance, /* Initialize performance metrics */
    .availability_callback        = NULL         /* Initialize callback */
  };
  atomic_init(&sd_card_default.card_available, false); /* Initialize atomic bool */

  /* Copy default configuration to sd_card */
  memcpy(sd_card, &sd_card_default, sizeof(sd_card_hal_t));

/* Check for pin conflicts with default pins */
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  if (bus_width == k_sd_bus_width_4bit && sd_card->pin_config.gpio_det_pin >= 0 &&
      sd_card->pin_config.gpio_det_pin == sd_card->pin_config.sdio_d3_pin) {
    log_warn(TAG, /* Use log_warn */
             "Pin Conflict",
             "Card detect pin (GPIO %d) conflicts with SDIO D3 for 4-bit mode. "
             "Will default to 1-bit mode for better compatibility.",
             sd_card->pin_config.gpio_det_pin);

    /* Default to 1-bit mode when there's a pin conflict */
    sd_card->bus_width = k_sd_bus_width_1bit;
  }
#endif

  /* Initialize error handler */
  err = error_handler_init(&sd_card->error_handler,
                           CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_RETRY,
                           CONFIG_PSTAR_KCONFIG_SD_CARD_RETRY_DELAY_MS,
                           CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_RETRY_DELAY_MS,
                           storage_sd_card_reset, /* Reset function */
                           sd_card);           /* Reset context */
  if (err != ESP_OK) {
    log_error(TAG, "Failed to init error handler: %s", esp_err_to_name(err));
    /* No resources allocated yet to clean up */
    return err;
  }

  /* Initialize mutex */
  sd_card->mutex = xSemaphoreCreateMutex();
  if (sd_card->mutex == NULL) {
    log_error(TAG, /* Use log_error */
              "Failed to create mutex",
              "sd_card: %p",
              sd_card);
    error_handler_deinit(&sd_card->error_handler); /* Cleanup handler if mutex fails */
    return ESP_ERR_NO_MEM;
  }

  /* Initialize bus manager */
  err = pstar_bus_manager_init(&sd_card->bus_manager, tag);
  if (err != ESP_OK) {
    log_error(TAG, /* Use log_error */
              "Failed to initialize bus manager",
              "err: %d",
              err);
    error_handler_deinit(&sd_card->error_handler);
    vSemaphoreDelete(sd_card->mutex);
    sd_card->mutex = NULL;
    return err;
  }

  /* Initialize NVS if not already initialized */
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    /* NVS partition was truncated and needs to be erased */
    err = nvs_flash_erase(); /* Use log_warn for NVS issues */
    if (err != ESP_OK) {
      log_warn(sd_card->tag, "NVS Warning", "Failed to erase NVS: %s", esp_err_to_name(err));
    }

    /* Retry initialization */
    err = nvs_flash_init();
  }

  if (err != ESP_OK) {
    log_warn(sd_card->tag, /* Use log_warn for NVS issues */
             "NVS Warning",
             "Failed to initialize NVS: %s",
             esp_err_to_name(err));
    /* Non-critical error, continue without NVS */
    err = ESP_OK; /* Treat NVS failure as non-fatal for HAL init */
  }

  log_info(TAG, /* Use log_info */
           "SD Card Default Init",
           "SD card HAL structure initialized with %s bus width mode",
           storage_bus_width_to_string(sd_card->bus_width));
  return ESP_OK;
}

esp_err_t sd_card_set_task_config(sd_card_hal_t* sd_card,
                                  uint32_t       stack_size,
                                  UBaseType_t    priority,
                                  TickType_t     mutex_timeout)
{
  if (sd_card == NULL) {
    log_error(TAG, /* Use log_error */
              "Set Task Config Error",
              "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

  if (sd_card->initialized) {
    log_warn(TAG, /* Use log_warn */
             "Set Task Config Warning",
             "Cannot change task configuration after initialization");
    return ESP_ERR_INVALID_STATE;
  }

  /* Validate configuration */
  if (stack_size < 2048) {
    log_warn(TAG, "Config Warning", "Stack size %lu too small, using minimum of 2048", stack_size);
    stack_size = 2048;
  }

  if (mutex_timeout == 0) {
    log_warn(TAG, "Config Warning", "Mutex timeout of 0 is not allowed, using default");
    mutex_timeout = pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_SD_CARD_MUTEX_TIMEOUT_MS);
  }

  /* Update task configuration */
  sd_card->task_config.stack_size    = stack_size;
  sd_card->task_config.priority      = priority;
  sd_card->task_config.mutex_timeout = mutex_timeout;

  log_info(sd_card->tag, /* Use log_info */
           "Task Config Updated",
           "SD card task configuration updated: stack=%lu, priority=%u, timeout=%lu",
           stack_size,
           priority,
           (unsigned long)mutex_timeout); /* Cast TickType_t for printf */
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

  /* Check if any interface mode is enabled */
  if (!storage_spi_is_supported() && !storage_sdio_is_supported()) {
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
  /* Use det_err to avoid conflict */
  if (det_err != ESP_OK) {
    log_error(sd_card->tag,
              "Init Error",
              "Failed to set up card detection: %s",
              esp_err_to_name(det_err));
    return det_err;
  }
#else
  /* Card detection disabled */
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
    /* Cleanup detection if pin registration fails */
    bool mutex_taken_cleanup = false;
    if (sd_card->mutex != NULL &&
        xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
      mutex_taken_cleanup = true;
      if (sd_card->pin_config.gpio_det_pin >= 0) {
        /* Use pstar_bus_gpio_isr_remove which checks the bus name */
        pstar_bus_gpio_isr_remove(&sd_card->bus_manager,
                                  CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME,
                                  sd_card->pin_config.gpio_det_pin);
        /* Unregister the pin as well */
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
    /* Cleanup detection setup if needed */
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

bool sd_card_is_available(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    return false;
  }
  return atomic_load(&sd_card->card_available);
}

sd_state_t sd_card_get_state(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    return k_sd_state_idle;
  }

  sd_state_t state       = k_sd_state_idle;
  bool       mutex_taken = false;

  /* Take mutex for thread-safe access */
  if (sd_card->mutex != NULL &&
      xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
    mutex_taken = true;
    state       = sd_card->state;
    storage_release_mutex_if_taken(sd_card, &mutex_taken);
  }

  return state;
}

sd_interface_type_t sd_card_get_current_interface(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    return k_sd_interface_none;
  }

  sd_interface_type_t interface   = k_sd_interface_none;
  bool                mutex_taken = false;

  /* Take mutex for thread-safe access */
  if (sd_card->mutex != NULL &&
      xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
    mutex_taken = true;
    interface   = sd_card->current_interface;
    storage_release_mutex_if_taken(sd_card, &mutex_taken);
  }

  return interface;
}

sd_bus_width_t sd_card_get_bus_width(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    return k_sd_bus_width_1bit;
  }

  sd_bus_width_t width       = k_sd_bus_width_1bit;
  bool           mutex_taken = false;

  /* Take mutex for thread-safe access */
  if (sd_card->mutex != NULL &&
      xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
    mutex_taken = true;
    width       = sd_card->bus_width;
    storage_release_mutex_if_taken(sd_card, &mutex_taken);
  }

  return width;
}

esp_err_t sd_card_get_performance(sd_card_hal_t* sd_card, sd_card_performance_t* performance)
{
  if (sd_card == NULL || performance == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  /* Use log_error for mutex timeout */
  bool mutex_taken = false;

  /* Take mutex for thread-safe access */
  if (sd_card->mutex != NULL &&
      xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
    mutex_taken = true;

    /* Copy performance metrics */
    memcpy(performance, &sd_card->performance, sizeof(sd_card_performance_t));

    /* If performance hasn't been measured yet and card is available, flag for measurement */
    if (performance->last_measured == 0 && atomic_load(&sd_card->card_available)) {
      sd_card->performance.measurement_needed = true;
    }

    storage_release_mutex_if_taken(sd_card, &mutex_taken);
    return ESP_OK;
  }

  log_error(sd_card->tag, "Get Perf Error", "Failed to acquire mutex for get_performance");
  return ESP_ERR_TIMEOUT;
}

esp_err_t sd_card_force_remount(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  bool      mutex_taken = false;
  esp_err_t result      = ESP_OK;

  /* Take mutex for thread-safe access */
  if (sd_card->mutex != NULL &&
      xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
    mutex_taken = true;

    /* Only proceed if card is inserted */
    if (!storage_sd_card_is_inserted(sd_card)) {
      log_warn(sd_card->tag, "Remount Warning", "Cannot remount - no card inserted");
      storage_release_mutex_if_taken(sd_card, &mutex_taken);
      return ESP_ERR_NOT_FOUND;
    }

    /* Unmount if currently mounted */
    if (atomic_load(&sd_card->card_available)) {
      esp_err_t unmount_result = sd_card_unmount(sd_card);
      if (unmount_result != ESP_OK) {
        log_error(sd_card->tag,
                  "Remount Error",
                  "Failed to unmount SD card: %s",
                  esp_err_to_name(unmount_result));
        result = unmount_result;
        storage_release_mutex_if_taken(sd_card, &mutex_taken);
        return result;
      }
    }

    /* Reset interface discovery state */
    sd_card->interface_discovery_complete = false;
    sd_card->interface_attempt_count      = 0;
    /* Reset specific interface info */
    for (int i = 0; i < k_sd_interface_count; i++) {
      sd_card->interface_info[i].attempted = false;
      /* Keep successful flag to try preferred first */
      /* sd_card->interface_info[i].successful = false; */
      sd_card->interface_info[i].error_count = 0;
      /* sd_card->interface_info[i].last_success_time = 0; */
    }

    storage_update_state_machine(sd_card, k_sd_state_card_inserted);
    storage_update_state_machine(sd_card, k_sd_state_interface_discovery);

    /* Try to mount with a fresh interface discovery */
    esp_err_t mount_result = sd_card_try_interfaces(sd_card);
    if (mount_result != ESP_OK) {
      log_error(sd_card->tag,
                "Remount Error",
                "Failed to remount SD card: %s",
                esp_err_to_name(mount_result));
      storage_update_state_machine(sd_card, k_sd_state_error);
      result = mount_result;
    } else if (atomic_load(&sd_card->card_available)) {
      /* Interface discovery successful */
      storage_update_state_machine(sd_card, k_sd_state_interface_ready);
      log_info(sd_card->tag,
               "Remount Success",
               "Successfully remounted SD card with %s interface and %s bus width",
               sd_card_interface_to_string(sd_card->current_interface),
               storage_bus_width_to_string(sd_card->bus_width));
    }

    /* Release mutex */
    storage_release_mutex_if_taken(sd_card, &mutex_taken);
  } else {
    log_error(sd_card->tag, "Remount Error", "Failed to acquire mutex for remount operation");
    return ESP_ERR_TIMEOUT;
  }

  return result;
}

esp_err_t sd_card_set_bus_width(sd_card_hal_t* sd_card, sd_bus_width_t bus_width)
{
  if (sd_card == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Validate bus width */
  if (bus_width != k_sd_bus_width_1bit && bus_width != k_sd_bus_width_4bit) {
    log_error(sd_card->tag, "Invalid bus width", "Bus width must be 1 or 4, got: %d", bus_width);
    return ESP_ERR_INVALID_ARG;
  }

  bool      mutex_taken = false;
  esp_err_t result      = ESP_OK;

  /* Take mutex for thread-safe access */
  if (sd_card->mutex != NULL &&
      xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
    mutex_taken = true;

    /* Check if the new bus width differs from the current one */
    if (sd_card->bus_width == bus_width) {
      log_info(sd_card->tag,
               "Bus Width",
               "Already using %s bus width",
               storage_bus_width_to_string(bus_width));
      storage_release_mutex_if_taken(sd_card, &mutex_taken);
      return ESP_OK;
    }

/* Check for pin conflicts with 4-bit mode */
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
    if (bus_width == k_sd_bus_width_4bit && sd_card->pin_config.gpio_det_pin >= 0 &&
        sd_card->pin_config.gpio_det_pin == sd_card->pin_config.sdio_d3_pin) {
      log_error(sd_card->tag,
                "Pin Conflict",
                "Cannot use 4-bit mode - card detect pin (GPIO %d) conflicts with SDIO D3",
                sd_card->pin_config.gpio_det_pin);
      storage_release_mutex_if_taken(sd_card, &mutex_taken);
      return ESP_ERR_INVALID_STATE;
    }
#endif

    /* Update bus width */
    log_info(sd_card->tag,
             "Bus Width Change",
             "Changing from %s to %s bus width",
             storage_bus_width_to_string(sd_card->bus_width),
             storage_bus_width_to_string(bus_width));

    sd_card->bus_width = bus_width;

    /* Save the new bus width setting to NVS */
    storage_save_working_config(sd_card);

    /* If card is currently mounted, force a remount to apply the new bus width */
    if (atomic_load(&sd_card->card_available)) {
      /* Release mutex since remount will take it again */
      storage_release_mutex_if_taken(sd_card, &mutex_taken);
      return sd_card_force_remount(sd_card);
    }

    /* Release mutex */
    storage_release_mutex_if_taken(sd_card, &mutex_taken);
  } else {
    log_error(sd_card->tag, "Bus Width Error", "Failed to acquire mutex for bus width change");
    return ESP_ERR_TIMEOUT;
  }

  return result;
}

esp_err_t sd_card_cleanup(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    log_error(TAG, "Cleanup Error", "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(sd_card->tag, "Cleanup Started", "Cleaning up SD card HAL resources");

  esp_err_t result      = ESP_OK;
  bool      mutex_taken = false;

  /* Signal mount task to exit */
  if (sd_card->mount_task_handle != NULL) {
    sd_card->mount_task_exit_requested = true;

    /* Wait for task to exit with timeout */
    TickType_t       start_time    = xTaskGetTickCount();
    const TickType_t max_wait_time = pdMS_TO_TICKS(1000); /* 1 second max wait */

    while (sd_card->mount_task_handle != NULL &&
           (xTaskGetTickCount() - start_time) < max_wait_time) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    /* Force delete if still running after timeout */
    if (sd_card->mount_task_handle != NULL) {
      log_warn(sd_card->tag,
               "Cleanup Warning",
               "Mount task did not exit cleanly, forcing deletion.");
      vTaskDelete(sd_card->mount_task_handle);
      sd_card->mount_task_handle = NULL;
    }
  }

  /* Attempt to take mutex with increased timeout for cleanup */
  if (sd_card->mutex != NULL) {
    if (xSemaphoreTake(sd_card->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      mutex_taken = true;
    } else {
      log_error(sd_card->tag,
                "Cleanup Error",
                "Failed to acquire mutex for cleanup - forcing cleanup anyway");
      /* Continue with cleanup even if we can't get the mutex */
    }
  }

  /* Unmount if mounted */
  if (atomic_load(&sd_card->card_available)) {
    esp_err_t err = sd_card_unmount(sd_card);
    if (err != ESP_OK) {
      log_error(sd_card->tag,
                "Cleanup Error",
                "Failed to unmount SD card: %s",
                esp_err_to_name(err));
      result = err;
      /* Continue with cleanup despite error */
    }
  }

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  /* Clean up GPIO ISR */
  if (sd_card->initialized &&
      sd_card->pin_config.gpio_det_pin >= 0) { /* Check initialized flag and valid pin */
    pstar_bus_gpio_isr_remove(&sd_card->bus_manager,
                              CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME,
                              sd_card->pin_config.gpio_det_pin);
    /* Unregister the pin from validator */
    pin_validator_unregister_pin(sd_card->pin_config.gpio_det_pin, "SD Card HAL");
  }
#endif
  /* Clean up bus manager */
  pstar_bus_manager_deinit(&sd_card->bus_manager);

  /* Free card structure */
  if (sd_card->card != NULL) {
    free(sd_card->card);
    sd_card->card = NULL;
  }

  /* Update state */
  sd_card->initialized = false;
  atomic_store(&sd_card->card_available, false);
  sd_card->interface_discovery_complete = false;
  sd_card->current_interface            = k_sd_interface_none;
  sd_card->state                        = k_sd_state_idle;
  sd_card->availability_callback        = NULL; /* Clear callback */

  /* Give mutex before deleting it */
  storage_release_mutex_if_taken(sd_card, &mutex_taken);

  /* Delete mutex */
  if (sd_card->mutex != NULL) {
    vSemaphoreDelete(sd_card->mutex);
    sd_card->mutex = NULL;
  }

  /* Deinit error handler */
  error_handler_deinit(&sd_card->error_handler);

  log_info(sd_card->tag, "Cleanup Complete", "SD card HAL resources cleaned up successfully");
  return result;
}

esp_err_t sd_card_register_availability_callback(sd_card_hal_t* sd_card,
                                                sd_availability_cb_t callback)
{
  if (sd_card == NULL) {
    log_error(TAG, "Callback Error", "SD card HAL pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  if (!sd_card->initialized) {
    log_error(TAG, "Callback Error", "SD card HAL not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  bool mutex_taken = false;
  if (sd_card->mutex != NULL &&
      xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
    mutex_taken = true;
  } else {
    log_error(TAG, "Callback Error", "Failed to acquire mutex");
    return ESP_ERR_TIMEOUT;
  }

  sd_card->availability_callback = callback;
  log_info(sd_card->tag,
           "Callback Registered",
           "Availability callback %s",
           callback ? "registered" : "cleared");

  storage_release_mutex_if_taken(sd_card, &mutex_taken);
  return ESP_OK;
}
