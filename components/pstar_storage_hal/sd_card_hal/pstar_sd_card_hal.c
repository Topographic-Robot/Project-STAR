/* components/pstar_storage_hal/sd_card_hal/pstar_sd_card_hal.c */

#include "pstar_sd_card_hal.h"

#include "pstar_bus_config.h"
#include "pstar_bus_event_types.h"
#include "pstar_bus_gpio.h"
#include "pstar_bus_manager.h"
#include "pstar_bus_sdio.h"
#include "pstar_bus_spi.h"
#include "pstar_log_handler.h"
#include "pstar_pin_validator.h"

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"

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

#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
#warning "Both SDIO and SPI modes for SD Card HAL are disabled. SD Card HAL will be non-functional."
#endif
#endif

#define SD_INTERFACE_MAX_ERRORS (3)

/* Logging and identification */
static const char* TAG = "SD Card HAL";

/* Default pin configurations from Kconfig */
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

/* Function Prototypes ********************************************************/

static void        priv_release_mutex_if_taken(sd_card_hal_t* sd_card, bool* mutex_taken);
static void        priv_update_state_machine(sd_card_hal_t* sd_card, sd_state_t new_state);
static esp_err_t   priv_save_working_config(sd_card_hal_t* sd_card);
static esp_err_t   priv_load_working_config(sd_card_hal_t* sd_card);
static void        priv_measure_card_performance(sd_card_hal_t* sd_card);
static bool        priv_check_directory_traversal(const char* path);
static bool        priv_check_allowed_characters(const char* path);
static bool        priv_path_is_safe(const sd_card_hal_t* sd_card, const char* path);
static bool        priv_path_is_safe_simple(const char* path);
static bool        priv_is_path_within_mount(const sd_card_hal_t* sd_card, const char* path);
static esp_err_t   priv_create_directory_if_needed(const sd_card_hal_t* sd_card, const char* path);
static const char* priv_sd_bus_width_to_string(sd_bus_width_t bus_width);
static esp_err_t   priv_sd_card_reset(void* context);
static bool        priv_sd_card_is_inserted(sd_card_hal_t* sd_card);
static esp_err_t   priv_sd_card_setup_spi(sd_card_hal_t* sd_card);
static esp_err_t   priv_sd_card_setup_sdio(sd_card_hal_t* sd_card);
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
static esp_err_t priv_sd_card_setup_detection(sd_card_hal_t* sd_card);
static void      priv_sd_card_detection_isr(void* arg);
#endif
static esp_err_t priv_sd_card_try_interfaces(sd_card_hal_t* sd_card);
static esp_err_t priv_sd_card_mount(sd_card_hal_t* sd_card);
static esp_err_t priv_sd_card_unmount(sd_card_hal_t* sd_card);
static void      priv_sd_card_mount_task(void* arg);
static esp_err_t priv_register_sd_card_pins(sd_card_hal_t* sd_card);
static void      priv_notify_availability(sd_card_hal_t* sd_card, bool available);

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
  if (!priv_path_is_safe_simple(mount_path)) {
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
  bool sdio_enabled;
  bool spi_enabled;

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
  sdio_enabled = true;
#else
  sdio_enabled = false;
#endif

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  spi_enabled = true;
#else
  spi_enabled = false;
#endif

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
    .preferred_interface = preferred_interface,
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLE_INTERFACE_FALLBACK
    .enable_fallback = true,
#else
    .enable_fallback = false,
#endif
    .sdio_mode_enabled            = sdio_enabled,
    .spi_mode_enabled             = spi_enabled,
    .pin_config                   = sd_card_default_pins, /* Use default pin configuration */
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
                           priv_sd_card_reset, /* Reset function */
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
           priv_sd_bus_width_to_string(sd_card->bus_width));
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
#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  log_error(sd_card->tag,
            "Init Error",
            "Cannot initialize SD Card HAL: Both SDIO and SPI modes are disabled.");
  return ESP_ERR_NOT_SUPPORTED;
#endif
#endif

  log_info(sd_card->tag,
           "Init Started",
           "Initializing SD card HAL with %s bus width",
           priv_sd_bus_width_to_string(sd_card->bus_width));

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  /* Set up card detection GPIO */
  esp_err_t det_err = priv_sd_card_setup_detection(sd_card);
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
  esp_err_t pin_err = priv_register_sd_card_pins(sd_card);
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
      priv_release_mutex_if_taken(sd_card, &mutex_taken_cleanup);
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
      priv_release_mutex_if_taken(sd_card, &mutex_taken);
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
    priv_release_mutex_if_taken(sd_card, &mutex_taken);
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
    priv_release_mutex_if_taken(sd_card, &mutex_taken);
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
    priv_release_mutex_if_taken(sd_card, &mutex_taken);
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

    priv_release_mutex_if_taken(sd_card, &mutex_taken);
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
    if (!priv_sd_card_is_inserted(sd_card)) {
      log_warn(sd_card->tag, "Remount Warning", "Cannot remount - no card inserted");
      priv_release_mutex_if_taken(sd_card, &mutex_taken);
      return ESP_ERR_NOT_FOUND;
    }

    /* Unmount if currently mounted */
    if (atomic_load(&sd_card->card_available)) {
      esp_err_t unmount_result = priv_sd_card_unmount(sd_card);
      if (unmount_result != ESP_OK) {
        log_error(sd_card->tag,
                  "Remount Error",
                  "Failed to unmount SD card: %s",
                  esp_err_to_name(unmount_result));
        result = unmount_result;
        priv_release_mutex_if_taken(sd_card, &mutex_taken);
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

    priv_update_state_machine(sd_card, k_sd_state_card_inserted);
    priv_update_state_machine(sd_card, k_sd_state_interface_discovery);

    /* Try to mount with a fresh interface discovery */
    esp_err_t mount_result = priv_sd_card_try_interfaces(sd_card);
    if (mount_result != ESP_OK) {
      log_error(sd_card->tag,
                "Remount Error",
                "Failed to remount SD card: %s",
                esp_err_to_name(mount_result));
      priv_update_state_machine(sd_card, k_sd_state_error);
      result = mount_result;
    } else if (atomic_load(&sd_card->card_available)) {
      /* Interface discovery successful */
      priv_update_state_machine(sd_card, k_sd_state_interface_ready);
      log_info(sd_card->tag,
               "Remount Success",
               "Successfully remounted SD card with %s interface and %s bus width",
               sd_card_interface_to_string(sd_card->current_interface),
               priv_sd_bus_width_to_string(sd_card->bus_width));
    }

    /* Release mutex */
    priv_release_mutex_if_taken(sd_card, &mutex_taken);
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
               priv_sd_bus_width_to_string(bus_width));
      priv_release_mutex_if_taken(sd_card, &mutex_taken);
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
      priv_release_mutex_if_taken(sd_card, &mutex_taken);
      return ESP_ERR_INVALID_STATE;
    }
#endif

    /* Update bus width */
    log_info(sd_card->tag,
             "Bus Width Change",
             "Changing from %s to %s bus width",
             priv_sd_bus_width_to_string(sd_card->bus_width),
             priv_sd_bus_width_to_string(bus_width));

    sd_card->bus_width = bus_width;

    /* Save the new bus width setting to NVS */
    priv_save_working_config(sd_card);

    /* If card is currently mounted, force a remount to apply the new bus width */
    if (atomic_load(&sd_card->card_available)) {
      /* Release mutex since remount will take it again */
      priv_release_mutex_if_taken(sd_card, &mutex_taken);
      return sd_card_force_remount(sd_card);
    }

    /* Release mutex */
    priv_release_mutex_if_taken(sd_card, &mutex_taken);
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
    esp_err_t err = priv_sd_card_unmount(sd_card);
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
  priv_release_mutex_if_taken(sd_card, &mutex_taken);

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

esp_err_t sd_card_register_availability_callback(sd_card_hal_t*       sd_card,
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

  priv_release_mutex_if_taken(sd_card, &mutex_taken);
  return ESP_OK;
}

/* Private Functions **********************************************************/

/**
 * @brief Calls the registered availability callback, if any.
 */
static void priv_notify_availability(sd_card_hal_t* sd_card, bool available)
{
  /* No mutex needed here, called from within critical sections (mount/unmount) */
  if (sd_card && sd_card->availability_callback) {
    log_info(sd_card->tag,
             "Notify Callback",
             "Notifying listener about availability: %s",
             available ? "Available" : "Unavailable");
    sd_card->availability_callback(available);
  }
}

/**
 * @brief Safely releases a mutex if it was taken
 *
 * @param[in]     sd_card     Pointer to the SD card HAL instance
 * @param[in,out] mutex_taken Pointer to the mutex taken flag to update
 */
static void priv_release_mutex_if_taken(sd_card_hal_t* sd_card, bool* mutex_taken)
{
  if (sd_card != NULL && sd_card->mutex != NULL && *mutex_taken) {
    xSemaphoreGive(sd_card->mutex);
    *mutex_taken = false;
  }
}

/**
 * @brief Updates the state machine state with proper logging
 *
 * @param[in] sd_card   Pointer to the SD card HAL instance
 * @param[in] new_state New state to transition to
 */
static void priv_update_state_machine(sd_card_hal_t* sd_card, sd_state_t new_state)
{
  if (sd_card == NULL) {
    return;
  }

  /* Take mutex for thread safety */
  bool mutex_taken = false;
  if (sd_card->mutex != NULL &&
      xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
    mutex_taken = true;
  } else {
    log_warn(sd_card->tag,
             "State Change Warning",
             "Failed to acquire mutex for state change, proceeding anyway");
    /* Continue without mutex as this is a critical operation */
  }

  /* No state change, nothing to do */
  if (sd_card->state == new_state) {
    priv_release_mutex_if_taken(sd_card, &mutex_taken);
    return;
  }

  /* Log state transition */
  const char* states[] =
    {"IDLE", "CARD_INSERTED", "INTERFACE_DISCOVERY", "INTERFACE_READY", "ERROR", "FAILED"};

  log_info(sd_card->tag,
           "State Change",
           "Transitioning from %s to %s state",
           states[sd_card->state],
           states[new_state]);

  /* Update the state */
  sd_card->state = new_state;

  /* Record state transition time */
  sd_card->last_state_change_time = esp_timer_get_time();

  /* Reset state-specific variables */
  if (new_state == k_sd_state_interface_discovery) {
    sd_card->interface_discovery_complete = false;
    sd_card->interface_attempt_count      = 0;
    /* Reset error counts for all interfaces when starting discovery */
    for (int i = 0; i < k_sd_interface_count; i++) {
      sd_card->interface_info[i].error_count = 0;
      sd_card->interface_info[i].attempted   = false;
      /* Keep successful flag */
    }
  } else if (new_state == k_sd_state_interface_ready) {
    /* Save the working configuration */
    priv_save_working_config(sd_card);
    /* Reset error handler */
    error_handler_reset_state(&sd_card->error_handler);
    /* Reset performance metrics */
    sd_card->performance.last_measured    = 0;
    sd_card->performance.read_speed_kbps  = 0;
    sd_card->performance.write_speed_kbps = 0;
    /* Schedule performance measurement */
    sd_card->performance.measurement_needed = true;
  } else if (new_state == k_sd_state_error) {
    /* Increment error count */
    sd_card->error_count++;
  } else if (new_state == k_sd_state_idle) {
    /* Reset error counts */
    sd_card->error_count       = 0;
    sd_card->current_interface = k_sd_interface_none; /* Clear current interface */
  }

  /* Release mutex */
  priv_release_mutex_if_taken(sd_card, &mutex_taken);
}

/**
 * @brief Saves the current working SD card configuration to NVS
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
static esp_err_t priv_save_working_config(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL || !atomic_load(&sd_card->card_available)) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Only save if we have a valid interface */
  if (sd_card->current_interface == k_sd_interface_none ||
      sd_card->current_interface >= k_sd_interface_count) {
    return ESP_ERR_INVALID_STATE;
  }

  nvs_handle_t nvs_handle;
  esp_err_t    err;

  /* Open NVS namespace */
  err = nvs_open(CONFIG_PSTAR_KCONFIG_SD_CARD_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    log_warn(sd_card->tag, "NVS Warning", "Failed to open NVS namespace: %s", esp_err_to_name(err));
    return err;
  }

  /* Save current interface */
  err = nvs_set_u8(nvs_handle,
                   CONFIG_PSTAR_KCONFIG_SD_CARD_NVS_INTERFACE_KEY,
                   (uint8_t)sd_card->current_interface);
  if (err != ESP_OK) {
    log_warn(sd_card->tag,
             "NVS Warning",
             "Failed to save interface to NVS: %s",
             esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  /* Save bus width */
  err = nvs_set_u8(nvs_handle,
                   CONFIG_PSTAR_KCONFIG_SD_CARD_NVS_BUS_WIDTH_KEY,
                   (uint8_t)sd_card->bus_width);
  if (err != ESP_OK) {
    log_warn(sd_card->tag,
             "NVS Warning",
             "Failed to save bus width to NVS: %s",
             esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  /* Commit changes */
  err = nvs_commit(nvs_handle);
  if (err != ESP_OK) {
    log_warn(sd_card->tag, "NVS Warning", "Failed to commit NVS changes: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  /* Close NVS handle */
  nvs_close(nvs_handle);

  log_info(sd_card->tag,
           "Config Saved",
           "Saved working configuration - Interface: %s, Bus Width: %d-bit",
           sd_card_interface_to_string(sd_card->current_interface),
           sd_card->bus_width);

  return ESP_OK;
}

/**
 * @brief Loads the previously working SD card configuration from NVS
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
static esp_err_t priv_load_working_config(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t nvs_handle;
  esp_err_t    err;

  /* Open NVS namespace */
  err = nvs_open(CONFIG_PSTAR_KCONFIG_SD_CARD_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
  if (err != ESP_OK) {
    /* Not finding the namespace is normal on first boot, just return */
    if (err == ESP_ERR_NVS_NOT_FOUND) {
      log_info(sd_card->tag, "NVS Info", "No saved SD card configuration found");
      return ESP_OK;
    }

    log_warn(sd_card->tag, "NVS Warning", "Failed to open NVS namespace: %s", esp_err_to_name(err));
    return err;
  }

  /* Load interface */
  uint8_t interface_val;
  err = nvs_get_u8(nvs_handle, CONFIG_PSTAR_KCONFIG_SD_CARD_NVS_INTERFACE_KEY, &interface_val);
  if (err == ESP_OK) {
    /* Validate the loaded value */
    if (interface_val < k_sd_interface_count) {
      /* Set the current interface as the loaded one, it will be tried first */
      sd_card->current_interface = (sd_interface_type_t)interface_val;
      /* Mark that we have a previously working interface */
      sd_card->interface_info[interface_val].successful = true;
      sd_card->interface_info[interface_val].last_success_time =
        esp_timer_get_time();                       /* Treat load time as last success */
      sd_card->interface_discovery_complete = true; /* Assume complete if loaded */
    } else {
      log_warn(sd_card->tag, "NVS Warning", "Invalid interface value in NVS: %d", interface_val);
      sd_card->current_interface            = k_sd_interface_none;
      sd_card->interface_discovery_complete = false;
    }
  } else if (err != ESP_ERR_NVS_NOT_FOUND) {
    log_warn(sd_card->tag,
             "NVS Warning",
             "Failed to read interface from NVS: %s",
             esp_err_to_name(err));
    sd_card->current_interface            = k_sd_interface_none;
    sd_card->interface_discovery_complete = false;
  } else {
    /* Interface key not found */
    sd_card->current_interface            = k_sd_interface_none;
    sd_card->interface_discovery_complete = false;
  }

  /* Load bus width */
  uint8_t bus_width_val;
  err = nvs_get_u8(nvs_handle, CONFIG_PSTAR_KCONFIG_SD_CARD_NVS_BUS_WIDTH_KEY, &bus_width_val);
  if (err == ESP_OK) {
    /* Validate the loaded value */
    if (bus_width_val == k_sd_bus_width_1bit || bus_width_val == k_sd_bus_width_4bit) {
      sd_card->bus_width = (sd_bus_width_t)bus_width_val;
    } else {
      log_warn(sd_card->tag, "NVS Warning", "Invalid bus width value in NVS: %d", bus_width_val);
      /* Keep the default/Kconfig bus width if NVS is invalid */
    }
  } else if (err != ESP_ERR_NVS_NOT_FOUND) {
    log_warn(sd_card->tag,
             "NVS Warning",
             "Failed to read bus width from NVS: %s",
             esp_err_to_name(err));
  }
  /* If bus width key not found, keep the default/Kconfig value */

  /* Close NVS handle */
  nvs_close(nvs_handle);

  /* If we successfully loaded a working configuration, log it */
  if (sd_card->current_interface != k_sd_interface_none) {
    log_info(sd_card->tag,
             "Config Loaded",
             "Loaded previous working configuration - Interface: %s, Bus Width: %d-bit",
             sd_card_interface_to_string(sd_card->current_interface),
             sd_card->bus_width);
    return ESP_OK;
  }

  log_info(sd_card->tag,
           "Config Not Found",
           "No valid previous working configuration found, will perform full discovery.");
  return ESP_OK;
}

static void priv_measure_card_performance(sd_card_hal_t* sd_card)
{
  /* Validate input parameters and card availability */
  if (sd_card == NULL || !atomic_load(&sd_card->card_available) || sd_card->card == NULL) {
    return;
  }

  /* Skip measurement if performed recently (< 1 minute) */
  int64_t now = esp_timer_get_time();
  if (sd_card->performance.last_measured > 0 &&
      ((now - sd_card->performance.last_measured) < 60000000)) {
    return;
  }

  log_info(sd_card->tag, "Performance Test", "Measuring SD card performance metrics");

  /* Allocate a temporary DMA-capable buffer for testing (256 KB) */
  const size_t buffer_size = 256 * 1024;
  uint8_t*     buffer      = heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
  if (buffer == NULL) {
    log_error(sd_card->tag, "Performance Error", "Failed to allocate memory for performance test");
    return;
  }

  /* Fill buffer with a test pattern */
  for (size_t i = 0; i < buffer_size; i++) {
    buffer[i] = (uint8_t)(i & 0xFF);
  }

  /* Define system subdirectory and compute its length dynamically */
  const char* system_subdir     = "/.system";
  size_t      system_subdir_len = strlen(system_subdir);

  /* Ensure the combined path (mount path + system_subdir) fits in the buffer */
  size_t mount_path_len = strlen(sd_card->mount_path);
  if (mount_path_len + system_subdir_len + 1 > CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH) {
    log_error(sd_card->tag,
              "Performance Error",
              "Mount path too long for creating system directory");
    free(buffer);
    return;
  }

  /* Construct the full system directory path safely using snprintf */
  char system_dir[CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH];
  int  ret = snprintf(system_dir, sizeof(system_dir), "%s%s", sd_card->mount_path, system_subdir);
  if (ret < 0 || (size_t)ret >= sizeof(system_dir)) {
    log_error(sd_card->tag, "System Dir Error", "Failed to construct system directory path");
    free(buffer);
    return;
  }

  /* Validate the constructed system directory path */
  if (!priv_path_is_safe_simple(system_dir) || !priv_is_path_within_mount(sd_card, system_dir)) {
    log_error(sd_card->tag,
              "Performance Error",
              "System directory path '%s' is not valid",
              system_dir);
    free(buffer);
    return;
  }

  /* Create the system directory if it does not exist */
  esp_err_t dir_err = priv_create_directory_if_needed(sd_card, system_dir);
  if (dir_err != ESP_OK) {
    log_error(sd_card->tag,
              "Performance Error",
              "Failed to create system directory: %s",
              esp_err_to_name(dir_err));
    free(buffer);
    return;
  }

  /* Define test file name and construct its full path dynamically */
  const char* test_filename     = "/perf_test";
  size_t      test_filename_len = strlen(test_filename);
  if (strlen(system_dir) + test_filename_len + 1 > CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH) {
    log_error(sd_card->tag,
              "Performance Error",
              "System directory path too long, cannot append test filename");
    free(buffer);
    return;
  }
  char test_path[CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH];
  ret = snprintf(test_path, sizeof(test_path), "%s%s", system_dir, test_filename);
  if (ret < 0 || (size_t)ret >= sizeof(test_path)) {
    log_error(sd_card->tag, "Test File Path Error", "Failed to construct test file path");
    free(buffer);
    return;
  }

  /* Measure write speed: create test file and write the buffer */
  int64_t write_start = esp_timer_get_time();
  FILE*   f           = fopen(test_path, "wb");
  if (f == NULL) {
    log_error(sd_card->tag, "Performance Error", "Failed to create test file: %s", strerror(errno));
    free(buffer);
    return;
  }

  /* (ORDER MATTERS) - Record write_end after fclose to include flush and close time */
  size_t bytes_written = fwrite(buffer, 1, buffer_size, f);
  fclose(f);
  int64_t write_end = esp_timer_get_time();

  /* Verify write success */
  if (bytes_written != buffer_size) {
    log_error(sd_card->tag,
              "Performance Error",
              "Write test incomplete: %zu/%zu bytes written",
              bytes_written,
              buffer_size);
    free(buffer);
    unlink(test_path);
    return;
  }

  /* Measure read speed: open the test file and read its contents */
  int64_t read_start = esp_timer_get_time();
  f                  = fopen(test_path, "rb");
  if (f == NULL) {
    log_error(sd_card->tag,
              "Performance Error",
              "Failed to open test file for reading: %s",
              strerror(errno));
    free(buffer);
    unlink(test_path);
    return;
  }
  /* Clear buffer before reading */
  memset(buffer, 0, buffer_size);
  size_t bytes_read = fread(buffer, 1, buffer_size, f);
  fclose(f);
  int64_t read_end = esp_timer_get_time(); /* Again, order matters */

  /* Remove test file after measurement */
  unlink(test_path);

  /* Verify read success */
  if (bytes_read != buffer_size) {
    log_error(sd_card->tag,
              "Performance Error",
              "Read test incomplete: %zu/%zu bytes read",
              bytes_read,
              buffer_size);
    free(buffer);
    return;
  }

  /* Free the temporary buffer */
  free(buffer);

  /* Calculate speeds in kilobits per second (kbps) */
  float write_time_sec = (float)(write_end - write_start) / 1000000.0f;
  float read_time_sec  = (float)(read_end - read_start) / 1000000.0f;
  float write_speed_kbps =
    (write_time_sec > 0) ? (buffer_size * 8.0f / 1000.0f) / write_time_sec : 0;
  float read_speed_kbps = (read_time_sec > 0) ? (buffer_size * 8.0f / 1000.0f) / read_time_sec : 0;

  /* Update performance metrics */
  sd_card->performance.write_speed_kbps   = write_speed_kbps;
  sd_card->performance.read_speed_kbps    = read_speed_kbps;
  sd_card->performance.last_measured      = now;
  sd_card->performance.measurement_needed = false;

  log_info(sd_card->tag,
           "Performance Results",
           "Read: %.2f kbps, Write: %.2f kbps",
           read_speed_kbps,
           write_speed_kbps);
}

/**
 * @brief Helper function to check for unsafe directory traversal patterns.
 *
 * This function checks if the given path contains common directory traversal patterns.
 *
 * @param[in] path Path to check.
 * @return true if no unsafe patterns are found, false otherwise.
 */
static bool priv_check_directory_traversal(const char* path)
{
  if (strstr(path, "..") != NULL || /* Standard directory traversal */
      strstr(path, "./") != NULL || /* Potential hiding technique */
      strstr(path, "//") != NULL || /* Multiple slashes can bypass filters */
      strstr(path, "\\") != NULL) { /* Backslashes can be problematic */
    log_error(TAG,
              "Path Validation Error",
              "Path '%s' contains potentially unsafe directory traversal patterns",
              path);
    return false;
  }
  return true;
}

/**
 * @brief Helper function to ensure the path only contains allowed characters.
 *
 * Allowed characters include alphanumerics, underscores, dashes, dots, slashes,
 * spaces, plus signs, equals, commas, at signs, and parentheses.
 *
 * @param[in] path Path to check.
 * @return true if all characters are allowed, false otherwise.
 */
static bool priv_check_allowed_characters(const char* path)
{
  for (const char* c = path; *c != '\0'; c++) {
    if (!(isalnum((unsigned char)*c) || *c == '_' || *c == '-' || *c == '.' || *c == '/' ||
          *c == ' ' || *c == '+' || *c == '=' || *c == ',' || *c == '@' || *c == '(' ||
          *c == ')')) {
      log_error(TAG,
                "Path Validation Error",
                "Path '%s' contains invalid character '%c' (0x%02X)",
                path,
                *c,
                (unsigned char)*c);
      return false;
    }
  }
  return true;
}

/**
 * @brief Checks if a path is safe to use (no directory traversal attacks).
 *
 * This version validates the path and also verifies that, for absolute paths,
 * the path starts with the configured SD card mount point.
 *
 * @param[in] sd_card Pointer to the SD card HAL instance.
 * @param[in] path Path to check.
 * @return true if path is safe, false otherwise.
 */
static bool priv_path_is_safe(const sd_card_hal_t* sd_card, const char* path)
{
  if (sd_card == NULL || path == NULL) {
    log_error(TAG, "Path Validation Error", "NULL parameter provided to path validation");
    return false;
  }

  /* Path length check to prevent buffer overflow attacks */
  if (strlen(path) >= CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH) {
    log_error(TAG,
              "Path Validation Error",
              "Path too long: %zu characters exceeds maximum of %zu",
              strlen(path),
              CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH - 1);
    return false;
  }

  /* Don't allow absolute paths that don't start with the SD card mount point */
  if (path[0] == '/') {
    size_t mount_path_len = strlen(sd_card->mount_path);
    if (strncmp(path, sd_card->mount_path, mount_path_len) != 0 ||
        (strlen(path) > mount_path_len && path[mount_path_len] != '/')) { /* Check next char */
      log_error(TAG,
                "Path Validation Error",
                "Path '%s' does not begin with configured mount point '%s'",
                path,
                sd_card->mount_path);
      return false;
    }
  }

  /* Perform common security checks */
  if (!priv_check_directory_traversal(path)) {
    return false;
  }
  if (!priv_check_allowed_characters(path)) {
    return false;
  }

  return true;
}

/**
 * @brief Simplified path safety check without requiring an SD card instance.
 *
 * This version validates the path for directory traversal patterns and allowed
 * characters without checking against a mount point.
 *
 * @param[in] path Path to check.
 * @return true if path is safe, false otherwise.
 */
static bool priv_path_is_safe_simple(const char* path)
{
  if (path == NULL) {
    return false;
  }

  /* Perform common security checks */
  if (!priv_check_directory_traversal(path)) {
    return false;
  }
  if (!priv_check_allowed_characters(path)) {
    return false;
  }

  return true;
}

/**
 * @brief Validates if a path is within the SD card's mount path
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @param[in] path Path to validate
 * @return true if within mount path, false otherwise
 */
static bool priv_is_path_within_mount(const sd_card_hal_t* sd_card, const char* path)
{
  if (sd_card == NULL || path == NULL) {
    return false;
  }

  /* Don't allow absolute paths that don't start with the SD card mount point */
  if (path[0] == '/') {
    /* Check if the path starts with the configured mount point prefix */
    size_t mount_path_len = strlen(sd_card->mount_path);
    if (strncmp(path, sd_card->mount_path, mount_path_len) != 0 ||
        (strlen(path) > mount_path_len && path[mount_path_len] != '/')) { /* Check next char */
      log_error(TAG,
                "Path Validation Error",
                "Path '%s' does not begin with configured mount point '%s'",
                path,
                sd_card->mount_path);
      return false;
    }
  }

  return true;
}

/**
 * @brief Creates a directory if it doesn't exist
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @param[in] path Directory path to create
 * @return ESP_OK if successful, appropriate error code otherwise
 */
static esp_err_t priv_create_directory_if_needed(const sd_card_hal_t* sd_card, const char* path)
{
  if (sd_card == NULL || path == NULL) {
    log_error(TAG, "Directory Error", "NULL parameter provided to directory creation");
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if path is safe */
  if (!priv_path_is_safe(sd_card, path)) {
    /* priv_path_is_safe already logs the specific error */
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if directory already exists */
  struct stat st;
  if (stat(path, &st) == 0) {
    /* Path exists, check if it's a directory */
    if (S_ISDIR(st.st_mode)) {
      /* Already a directory, nothing to do */
      log_debug(TAG, "Directory Already Exists", "Path exists and is a directory: %s", path);
      return ESP_OK;
    } else {
      /* Path exists but is not a directory */
      log_error(TAG,
                "Directory Error",
                "Path exists but is not a directory: %s (type: %lu)",
                path,
                st.st_mode & S_IFMT);
      return ESP_ERR_INVALID_STATE;
    }
  }

  /* Directory doesn't exist, create it */
  log_info(TAG, "Creating Directory", "Creating directory: %s", path);

  if (mkdir(path, 0755) != 0) {
    /* Check specific error conditions */
    if (errno == ENOENT) {
      log_error(TAG,
                "Directory Error",
                "Failed to create directory %s: Parent directory doesn't exist",
                path);
    } else if (errno == EACCES) {
      log_error(TAG, "Directory Error", "Failed to create directory %s: Permission denied", path);
    } else {
      log_error(TAG,
                "Directory Error",
                "Failed to create directory %s: %s (errno: %d)",
                path,
                strerror(errno),
                errno);
    }
    return ESP_FAIL;
  }

  log_info(TAG, "Directory Created", "Successfully created directory: %s", path);
  return ESP_OK;
}

/**
 * @brief Returns the string representation of the SD bus width
 *
 * @param[in] bus_width The bus width enum value
 * @return Descriptive string of the bus width
 */
static const char* priv_sd_bus_width_to_string(sd_bus_width_t bus_width)
{
  switch (bus_width) {
    case k_sd_bus_width_1bit:
      return "1-bit";
    case k_sd_bus_width_4bit:
      return "4-bit";
    default:
      return "Unknown";
  }
}

/**
 * @brief Resets the SD card hardware and remounts if needed.
 *        Assumes the caller holds the sd_card->mutex.
 *
 * This function is called by the error handler when an error occurs to try
 * to recover from the error by resetting the SD card hardware and remounting.
 *
 * @param[in] context Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
static esp_err_t priv_sd_card_reset(void* context)
{
  esp_err_t      result  = ESP_OK;
  sd_card_hal_t* sd_card = (sd_card_hal_t*)context;

  if (sd_card == NULL) {
    log_error(TAG, "Reset Error", "Invalid context pointer");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(sd_card->tag, "Reset Started", "Resetting SD card hardware");

  /* --- Mutex is assumed to be held by the caller --- */

  /* Unmount first if currently mounted */
  if (atomic_load(&sd_card->card_available)) {
    esp_err_t unmount_result =
      priv_sd_card_unmount(sd_card); /* unmount handles its own state changes/notifications */
    if (unmount_result != ESP_OK) {
      log_error(sd_card->tag,
                "Reset Error",
                "Failed to unmount SD card: %s",
                esp_err_to_name(unmount_result));
      result = unmount_result;
      return result; /* Return early on unmount failure */
    }
    /* If unmount succeeds, card_available is now false, state is idle */
  }

  /* Start interface discovery again */
  sd_card->interface_discovery_complete = false;
  sd_card->interface_attempt_count      = 0;
  /* Reset specific interface info error counts */
  for (int i = 0; i < k_sd_interface_count; i++) {
    sd_card->interface_info[i].error_count = 0;
    sd_card->interface_info[i].attempted   = false;
  }

  /* Set current interface to preferred, will be tried first */
  sd_card->current_interface = sd_card->preferred_interface;

  /* If card is inserted, try to mount */
  if (priv_sd_card_is_inserted(sd_card)) { /* is_inserted takes+gives its own mutex if needed */
    priv_update_state_machine(
      sd_card,
      k_sd_state_interface_discovery); /* update_state takes+gives its own mutex */
    esp_err_t mount_result =
      priv_sd_card_try_interfaces(sd_card); /* try_interfaces handles its own state/mutex */
    if (mount_result != ESP_OK) {
      log_error(sd_card->tag,
                "Reset Error",
                "Failed to initialize SD card interface after reset: %s",
                esp_err_to_name(mount_result));
      result = mount_result;
      /* Update state machine to error after failed reset attempt */
      priv_update_state_machine(sd_card,
                                k_sd_state_error); /* update_state takes+gives its own mutex */
      return result;                               /* Return error */
    }
    /* If mounted successfully, update state */
    if (atomic_load(&sd_card->card_available)) {
      priv_update_state_machine(
        sd_card,
        k_sd_state_interface_ready); /* update_state takes+gives its own mutex */
    } else {
      /* Mount failed even after reset */
      priv_update_state_machine(sd_card,
                                k_sd_state_error); /* update_state takes+gives its own mutex */
    }
  } else {
    priv_update_state_machine(sd_card,
                              k_sd_state_idle); /* update_state takes+gives its own mutex */
  }

  /* --- Mutex is released by the caller --- */

  if (result == ESP_OK && atomic_load(&sd_card->card_available)) {
    log_info(sd_card->tag, "Reset Complete", "SD card reset successful and card mounted");
  } else if (result == ESP_OK) {
    log_info(sd_card->tag,
             "Reset Complete",
             "SD card reset successful but card not mounted (not inserted or mount failed)");
  }

  return result;
}

/**
 * @brief Checks if an SD card is physically inserted
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return true if card is inserted, false otherwise
 */
static bool priv_sd_card_is_inserted(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    return false;
  }

#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  /* If detection is disabled, assume card is always inserted (or rely on mount success) */
  /* For safety, let's assume not inserted unless explicitly mounted */
  /* NOTE: This behavior might need adjustment depending on desired logic when detection is off */
  return atomic_load(&sd_card->card_available);
#else

  /* Check if detection pin is valid */
  if (sd_card->pin_config.gpio_det_pin < 0) {
    log_warn(sd_card->tag,
             "Detection Warning",
             "Card detection enabled but pin is invalid (-1). Assuming card is not inserted.");
    return false;
  }

  bool card_inserted = false;
  bool mutex_taken   = false;

  /* Take mutex for thread-safe access */
  if (sd_card->mutex != NULL &&
      xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
    mutex_taken     = true;
    uint32_t  level = 0;
    esp_err_t err;

    /* Read the card detect pin */
    err = pstar_bus_gpio_get_level(&sd_card->bus_manager,
                                   CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME,
                                   sd_card->pin_config.gpio_det_pin,
                                   &level);

    if (err != ESP_OK) {
      log_error(sd_card->tag,
                "Detection Error",
                "Failed to read card detect pin: %s",
                esp_err_to_name(err));
      priv_release_mutex_if_taken(sd_card, &mutex_taken);
      return false;
    }

    /* Interpret level based on active mode */
    if (sd_card->card_detect_low_active) {
      /* Low-active pin: 0 means card present */
      card_inserted = (level == 0);
    } else {
      /* High-active pin: 1 means card present */
      card_inserted = (level == 1);
    }
  } else { /* Use log_warn */
    log_warn(sd_card->tag,
             "Detection Warning",
             "Failed to acquire mutex for card detection, defaulting to not inserted");
  }

  /* Release mutex if taken */
  priv_release_mutex_if_taken(sd_card, &mutex_taken);

  return card_inserted;
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED */
}

/**
 * @brief Sets up the SPI interface for SD card communication
 *
 * @param[in] sd_card        Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
static esp_err_t priv_sd_card_setup_spi(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    log_error(TAG, "SPI Setup Error", "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  log_error(TAG, "SPI Setup Error", "SPI Mode is disabled in Kconfig");
  return ESP_ERR_NOT_SUPPORTED;
#endif

  log_info(sd_card->tag, "SPI Setup", "Setting up SPI interface for SD card");

  /* Create SPI bus configuration */
  pstar_bus_config_t* spi_config   = NULL;
  sdspi_dev_handle_t  sdspi_handle = 0;
  esp_err_t           err          = ESP_OK;

  spi_config = pstar_bus_config_create_spi(CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_BUS_NAME,
                                           SPI2_HOST,
                                           k_pstar_mode_polling);
  if (spi_config == NULL) {
    log_error(sd_card->tag, "SPI Setup Error", "Failed to create SPI bus configuration");
    return ESP_ERR_NO_MEM;
  }

  /* Configure SPI pins from the pin configuration */
  spi_config->config.spi.bus_config.miso_io_num     = sd_card->pin_config.spi_di_pin;   /* DI */
  spi_config->config.spi.bus_config.mosi_io_num     = sd_card->pin_config.spi_do_pin;   /* DO */
  spi_config->config.spi.bus_config.sclk_io_num     = sd_card->pin_config.spi_sclk_pin; /* CLK */
  spi_config->config.spi.bus_config.quadwp_io_num   = -1;   /* Not used */
  spi_config->config.spi.bus_config.quadhd_io_num   = -1;   /* Not used */
  spi_config->config.spi.bus_config.max_transfer_sz = 4092; /* Max SPI transfer size */

  /* Configure SPI device - Start with lower speed for initialization */
  spi_config->config.spi.dev_config.clock_speed_hz = 400 * 1000; /* 400 kHz initial speed */
  spi_config->config.spi.dev_config.mode           = 0;          /* SPI mode 0 */
  spi_config->config.spi.dev_config.spics_io_num = sd_card->pin_config.spi_cs_pin; /* Chip Select */
  spi_config->config.spi.dev_config.queue_size   = 7; /* Queue size for transactions */
  spi_config->config.spi.dev_config.flags        = 0; /* No special flags */

  /* Check if the bus already exists */
  pstar_bus_config_t* existing_spi =
    pstar_bus_manager_find_bus(&sd_card->bus_manager, CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_BUS_NAME);
  if (existing_spi != NULL) {
    log_info(sd_card->tag, "SPI Info", "Re-using existing SPI bus configuration.");
    /* If re-using, remove the old device attached to the bus handle if it exists */
    if (existing_spi->handle != NULL) {
      sdspi_host_remove_device(
        (sdspi_dev_handle_t)(uintptr_t)existing_spi->handle); /* Cast needed */
      existing_spi->handle = NULL;                            /* Clear the handle */
    }
    /* Free the newly created config as we are using the existing one */
    pstar_bus_config_destroy(spi_config);
    spi_config = existing_spi; /* Use existing config */
  } else {
    /* Add SPI bus to the manager */
    err = pstar_bus_manager_add_bus(&sd_card->bus_manager, spi_config);
    if (err != ESP_OK) {
      log_error(sd_card->tag,
                "SPI Setup Error",
                "Failed to add SPI bus to manager: %s",
                esp_err_to_name(err));
      pstar_bus_config_destroy(spi_config); /* Destroy the newly created config */
      return err;
    }

    /* Initialize the SPI bus */
    err = pstar_bus_config_init(spi_config);
    if (err != ESP_OK) {
      log_error(sd_card->tag,
                "SPI Setup Error",
                "Failed to initialize SPI bus: %s",
                esp_err_to_name(err));
      /* Properly clean up - first remove from manager, then destroy */
      pstar_bus_manager_remove_bus(&sd_card->bus_manager,
                                   CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_BUS_NAME);
      return err;
    }
    /* Initialize default SPI operations */
    pstar_bus_spi_init_default_ops(&spi_config->config.spi.ops);
  }

  /* Create SDSPI host configuration */
  sdspi_device_config_t device_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  device_config.host_id               = spi_config->config.spi.host;
  device_config.gpio_cs               = spi_config->config.spi.dev_config.spics_io_num;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  device_config.gpio_cd = sd_card->pin_config.gpio_det_pin;
#else
  device_config.gpio_cd = SDSPI_SLOT_NO_CD;
#endif
  device_config.gpio_wp  = SDSPI_SLOT_NO_WP;  /* Write protect not used */
  device_config.gpio_int = SDSPI_SLOT_NO_INT; /* Interrupt not used */

  /* Initialize SD SPI driver (only if not already done) */
  err = sdspi_host_init(); /* Store result of init call */
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    log_error(sd_card->tag,
              "SPI Setup Error",
              "Failed to initialize SDSPI host: %s",
              esp_err_to_name(err));
    /* Clean up properly in reverse order */
    if (existing_spi == NULL) { /* Only cleanup if we created it */
      pstar_bus_config_deinit(spi_config);
      pstar_bus_manager_remove_bus(&sd_card->bus_manager,
                                   CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_BUS_NAME);
    }
    return err;
  }

  /* Initialize the device and get the handle */
  err = sdspi_host_init_device(&device_config, &sdspi_handle);
  if (err != ESP_OK) {
    log_error(sd_card->tag,
              "SPI Setup Error",
              "Failed to initialize SDSPI device: %s",
              esp_err_to_name(err));
    /* Clean up properly in reverse order */
    /* No need to deinit host if it was already initialized */
    /* sdspi_host_deinit(); */
    if (existing_spi == NULL) { /* Only cleanup if we created it */
      pstar_bus_config_deinit(spi_config);
      pstar_bus_manager_remove_bus(&sd_card->bus_manager,
                                   CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_BUS_NAME);
    }
    return err;
  }

  /* Mount SDSPI host to SDMMC host */
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot         = sdspi_handle;

  /* Free old card structure if it exists */
  if (sd_card->card != NULL) {
    free(sd_card->card);
    sd_card->card = NULL;
  }

  /* Allocate memory for the card structure */
  sd_card->card = malloc(sizeof(sdmmc_card_t));
  if (sd_card->card == NULL) {
    log_error(sd_card->tag, "SPI Setup Error", "Failed to allocate memory for SD card");
    /* Clean up properly in reverse order */
    sdspi_host_remove_device(sdspi_handle);
    /* sdspi_host_deinit(); */
    if (existing_spi == NULL) { /* Only cleanup if we created it */
      pstar_bus_config_deinit(spi_config);
      pstar_bus_manager_remove_bus(&sd_card->bus_manager,
                                   CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_BUS_NAME);
    }
    return ESP_ERR_NO_MEM;
  }

  /* Copy host configuration to our card structure */
  memcpy(&sd_card->card->host, &host, sizeof(sdmmc_host_t));

  log_info(sd_card->tag, "SPI Setup Complete", "SPI interface configured successfully");
  return ESP_OK;
}

/**
 * @brief Sets up the SDIO interface for SD card communication
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
static esp_err_t priv_sd_card_setup_sdio(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    log_error(TAG, "SDIO Setup Error", "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
  log_error(TAG, "SDIO Setup Error", "SDIO Mode is disabled in Kconfig");
  return ESP_ERR_NOT_SUPPORTED;
#endif

  log_info(sd_card->tag,
           "SDIO Setup",
           "Setting up SDIO interface for SD card with %s bus mode",
           priv_sd_bus_width_to_string(sd_card->bus_width));

  /* Check for pin conflicts when using 4-bit mode */
  bool           pin_conflict       = false;
  sd_bus_width_t original_bus_width = sd_card->bus_width;

  if (sd_card->bus_width == k_sd_bus_width_4bit) {
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
    /* Check if card detect pin conflicts with SDIO D3 */
    if (sd_card->pin_config.gpio_det_pin >= 0 &&
        sd_card->pin_config.gpio_det_pin == sd_card->pin_config.sdio_d3_pin) {
      log_warn(sd_card->tag,
               "Pin Conflict",
               "Card detect pin (GPIO %d) conflicts with SDIO D3 for 4-bit mode. "
               "Falling back to 1-bit mode",
               sd_card->pin_config.gpio_det_pin);

      /* Automatically change to 1-bit mode when conflict is detected */
      sd_card->bus_width = k_sd_bus_width_1bit;
      pin_conflict       = true;
    }
#endif

    /* Also check for other potential pin conflicts or unavailable pins */
    if (sd_card->pin_config.sdio_d1_pin < 0 || sd_card->pin_config.sdio_d2_pin < 0 ||
        sd_card->pin_config.sdio_d3_pin < 0) {
      log_warn(sd_card->tag,
               "Pin Configuration",
               "Missing required pins for 4-bit SDIO mode. Falling back to 1-bit mode");

      /* Automatically change to 1-bit mode when pins are unavailable */
      sd_card->bus_width = k_sd_bus_width_1bit;
      pin_conflict       = true;
    }
  }

  /* Create SDIO bus configuration with all required parameters */
  sdmmc_host_t sdmmc_host = SDMMC_HOST_DEFAULT();

  /* Configure the SDMMC host */
  sdmmc_host.max_freq_khz = SDMMC_FREQ_DEFAULT; /* Use default frequency initially */
  sdmmc_host.flags        = 0;                  /* Start with no flags */
  if (sd_card->bus_width == k_sd_bus_width_4bit && !pin_conflict) {
    sdmmc_host.flags |= SDMMC_HOST_FLAG_4BIT;
  } else {
    sdmmc_host.flags |= SDMMC_HOST_FLAG_1BIT;
  }

  /* Set up standard SDMMC configuration for ESP32 */
  sdmmc_host.command_timeout_ms = 5000; /* Increased timeout */
  /*
   * Set the expected I/O voltage for the SD card communication.
   * This value informs the SDMMC driver about the card's operating voltage
   * standard (typically 3.3V) for correct protocol negotiation and timing.
   * It does NOT change the actual hardware voltage output by the ESP32 pins,
   * which is fixed (usually 3.3V). Setting this incorrectly will likely
   * cause card initialization failures.
   */
  sdmmc_host.io_voltage = 3.3f;

  /* Initialize SDMMC host controller */
  esp_err_t err = sdmmc_host_init();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { /* Allow re-init */
    log_error(sd_card->tag,
              "SDIO Setup Error",
              "Failed to initialize SDMMC host: %s",
              esp_err_to_name(err));
    return err;
  }

  /* Configure slot */
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.clk                 = sd_card->pin_config.sdio_clk_pin;
  slot_config.cmd                 = sd_card->pin_config.sdio_cmd_pin;
  slot_config.d0                  = sd_card->pin_config.sdio_d0_pin;
  slot_config.width               = sd_card->bus_width; /* Set width here */

  if (sd_card->bus_width == k_sd_bus_width_4bit && !pin_conflict) {
    slot_config.d1 = sd_card->pin_config.sdio_d1_pin;
    slot_config.d2 = sd_card->pin_config.sdio_d2_pin;
    slot_config.d3 = sd_card->pin_config.sdio_d3_pin;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP; /* Enable pullups for 4-bit */
  } else {
    /* Ensure 1-bit mode if conflict or configured */
    slot_config.width = 1;
    sdmmc_host.flags &= ~SDMMC_HOST_FLAG_4BIT;
    sdmmc_host.flags |= SDMMC_HOST_FLAG_1BIT;
    sd_card->bus_width = k_sd_bus_width_1bit; /* Update HAL state */
  }

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  slot_config.gpio_cd = sd_card->pin_config.gpio_det_pin;
#else
  slot_config.gpio_cd = SDMMC_SLOT_NO_CD;
#endif
  slot_config.gpio_wp = SDMMC_SLOT_NO_WP; /* Write protect not used */

  /* Initialize SDMMC slot */
  err = sdmmc_host_init_slot(sdmmc_host.slot, &slot_config);
  if (err != ESP_OK) {
    log_error(sd_card->tag,
              "SDIO Setup Error",
              "Failed to initialize SDMMC slot: %s",
              esp_err_to_name(err));
    sdmmc_host_deinit(); /* Clean up host init */
    return err;
  }

  /* Free old card structure if it exists */
  if (sd_card->card != NULL) {
    free(sd_card->card);
    sd_card->card = NULL;
  }

  /* Allocate memory for the card structure */
  sd_card->card = malloc(sizeof(sdmmc_card_t));
  if (sd_card->card == NULL) {
    log_error(sd_card->tag, "SDIO Setup Error", "Failed to allocate memory for SD card");
    sdmmc_host_deinit();
    return ESP_ERR_NO_MEM;
  }

  /* Copy host configuration to our card structure */
  memcpy(&sd_card->card->host, &sdmmc_host, sizeof(sdmmc_host_t));

  /* If we changed bus width due to conflicts, log it */
  if (original_bus_width != sd_card->bus_width) {
    log_info(sd_card->tag,
             "SDIO Bus Width Changed",
             "Changed from %s to %s mode due to pin conflicts or configuration issues",
             priv_sd_bus_width_to_string(original_bus_width),
             priv_sd_bus_width_to_string(sd_card->bus_width));
  }

  log_info(sd_card->tag,
           "SDIO Setup Complete",
           "SDIO interface configured successfully with %s bus mode",
           priv_sd_bus_width_to_string(sd_card->bus_width));
  return ESP_OK;
}

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
/**
 * @brief Sets up card detection mechanism using GPIO
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
static esp_err_t priv_sd_card_setup_detection(sd_card_hal_t* sd_card)
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

/**
 * @brief Try mounting the SD card with available interfaces according to config
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if any interface worked, appropriate error code otherwise
 */
static esp_err_t priv_sd_card_try_interfaces(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    log_error(TAG, "Interface Error", "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  log_error(sd_card->tag,
            "Interface Error",
            "No SD card interfaces (SDIO or SPI) are enabled in Kconfig.");
  return ESP_ERR_NOT_SUPPORTED;
#endif
#endif

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
      setup_err = priv_sd_card_setup_sdio(sd_card);
    } else if (current_try == k_sd_interface_spi) {
      setup_err = priv_sd_card_setup_spi(sd_card);
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
    esp_err_t mount_err = priv_sd_card_mount(sd_card);
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
 * @brief Mounts the SD card
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
static esp_err_t priv_sd_card_mount(sd_card_hal_t* sd_card)
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
  if (!priv_sd_card_is_inserted(sd_card)) {
    log_warn(sd_card->tag, "Mount Warning", "Cannot mount: SD card not inserted");
    return ESP_ERR_NOT_FOUND;
  }
  log_info(sd_card->tag,
           "Mount Started",
           "Mounting SD card at path: %s with %s interface",
           sd_card->mount_path,
           sd_card_interface_to_string(sd_card->current_interface));

  /* Validate mount path using the simpler validation function */
  if (!priv_path_is_safe_simple(sd_card->mount_path)) {
    log_error(sd_card->tag, "Mount Error", "Unsafe mount path: %s", sd_card->mount_path);
    return ESP_ERR_INVALID_ARG;
  }

  /* Ensure mount path directory exists */
  esp_err_t dir_result = priv_create_directory_if_needed(sd_card, sd_card->mount_path);
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
  esp_err_t logs_dir_result = priv_create_directory_if_needed(sd_card, logs_path);
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
           priv_sd_bus_width_to_string(sd_card->bus_width));

  /* Update state */
  atomic_store(&sd_card->card_available, true);

  /* Schedule performance measurement on the new card */
  sd_card->performance.measurement_needed = true;

  /* Notify listeners about availability */
  priv_notify_availability(sd_card, true);

  return ESP_OK;
}

/**
 * @brief Unmounts the SD card
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
static esp_err_t priv_sd_card_unmount(sd_card_hal_t* sd_card)
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
    priv_notify_availability(sd_card, false); /* Notify listeners */
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
  priv_notify_availability(sd_card, false);

  log_info(sd_card->tag, "Unmount Success", "SD card unmounted successfully");
  return ESP_OK;
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

#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  log_warn(sd_card->tag,
           "Mount Task Info",
           "SD Card HAL disabled (no interfaces enabled). Exiting mount task.");
  sd_card->mount_task_handle = NULL; /* Clear handle before exit */
  vTaskDelete(NULL);                 /* Exit task if HAL is disabled */
  return;
#endif
#endif
  log_info(sd_card->tag, "Mount Task Started", "SD card mount/unmount task is running");

  bool mutex_taken = false;

  /* Initialize state machine */
  sd_card->state                  = k_sd_state_idle;
  sd_card->last_state_change_time = esp_timer_get_time();

  /* Initialize exit request flag */
  sd_card->mount_task_exit_requested = false;

  /* Load previous working configuration from NVS if available */
  priv_load_working_config(sd_card);

  /* Initial card check */
  bool is_inserted;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  is_inserted               = priv_sd_card_is_inserted(sd_card);
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
      priv_update_state_machine(sd_card, k_sd_state_card_inserted);
      priv_update_state_machine(sd_card, k_sd_state_interface_discovery);

      /* Try interfaces */
      esp_err_t err = priv_sd_card_try_interfaces(sd_card);
      if (err != ESP_OK) {
        log_error(sd_card->tag,
                  "Initial Mount Error",
                  "Failed to mount SD card at startup: %s",
                  esp_err_to_name(err));
        /* Update state machine to reflect the error */
        priv_update_state_machine(sd_card, k_sd_state_error);
      } else if (atomic_load(&sd_card->card_available)) {
        /* Interface discovery successful, update state */
        priv_update_state_machine(sd_card, k_sd_state_interface_ready);
      }

      /* Always release mutex when done */
      priv_release_mutex_if_taken(sd_card, &mutex_taken);
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
    is_inserted = priv_sd_card_is_inserted(sd_card);
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
            priv_update_state_machine(sd_card, k_sd_state_card_inserted);
            priv_update_state_machine(sd_card, k_sd_state_interface_discovery);

            /* Try interfaces in priority order */
            esp_err_t err = priv_sd_card_try_interfaces(sd_card);
            if (err != ESP_OK) {
              log_error(sd_card->tag,
                        "Mount Error",
                        "Failed to mount SD card: %s",
                        esp_err_to_name(err));

              /* Record the error */
              RECORD_ERROR(&sd_card->error_handler, err, "Failed to mount SD card");

              /* Update state machine to reflect the error */
              priv_update_state_machine(sd_card, k_sd_state_error);

              /* If we can retry, do so after some delay */
              if (error_handler_can_retry(&sd_card->error_handler)) {
                log_info(sd_card->tag, "Retry", "Will retry mounting after delay");
                /* The error handler will call the reset function */
              } else {
                /* Maximum retries reached */
                priv_update_state_machine(sd_card, k_sd_state_failed);
              }
            } else if (atomic_load(&sd_card->card_available)) {
              /* Mount successful - update state machine */
              priv_update_state_machine(sd_card, k_sd_state_interface_ready);
              /* Reset error handler */
              error_handler_reset_state(&sd_card->error_handler);
            }
          } else if (!is_inserted && atomic_load(&sd_card->card_available)) {
            /* Card removed - unmount first */
            esp_err_t err = priv_sd_card_unmount(sd_card);
            if (err != ESP_OK) {
              log_error(sd_card->tag,
                        "Unmount Error",
                        "Failed to unmount SD card: %s",
                        esp_err_to_name(err));

              /* Even if unmount fails, we need to update the state machine */
              /* to reflect that the card is physically removed */
            }

            /* Then update state machine */
            priv_update_state_machine(sd_card, k_sd_state_idle);

            /* Reset interface discovery state */
            sd_card->interface_discovery_complete = false;

            log_info(sd_card->tag,
                     "Card Removed",
                     "SD card has been physically removed and unmounted");
          }

          /* Always release mutex when done */
          priv_release_mutex_if_taken(sd_card, &mutex_taken);
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
          esp_err_t err = priv_sd_card_reset(sd_card);

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
              priv_update_state_machine(sd_card, k_sd_state_failed);
            }
            /* State remains k_sd_state_error if retries remain */
          } else if (atomic_load(&sd_card->card_available)) {
            /* Reset successful, card mounted */
            priv_update_state_machine(sd_card, k_sd_state_interface_ready);
            error_handler_reset_state(&sd_card->error_handler); /* Reset error state on success */
          } else {
            /* Reset OK, but card still not available (e.g. removed) */
            priv_update_state_machine(sd_card, k_sd_state_idle);
            error_handler_reset_state(&sd_card->error_handler); /* Reset error state */
          }
        } else {
          log_info(sd_card->tag,
                   "Error Retry Skip",
                   "Condition for retry no longer met after delay/mutex acquisition.");
        }
        /* Release mutex after operation */
        priv_release_mutex_if_taken(sd_card, &mutex_taken);
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
        priv_measure_card_performance(sd_card);

        /* Release mutex */
        priv_release_mutex_if_taken(sd_card, &mutex_taken);
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
              priv_update_state_machine(sd_card, k_sd_state_error);

              /* Let the error retry logic handle the reset */
            }
          }

          /* Release mutex */
          priv_release_mutex_if_taken(sd_card, &mutex_taken);
        }
      }
    }
  }

  log_info(sd_card->tag, "Mount Task Exiting", "SD card mount/unmount task received exit request");

  sd_card->mount_task_handle = NULL; /* Clear handle before deleting */
  vTaskDelete(NULL);
}

/**
 * @brief Register all SD card pins with the pin validator.
 *        Logs configuration errors but attempts to register all pins regardless
 *        to allow the main validator to catch conflicts later.
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK Always returns ESP_OK in this modified version,
 *         logs errors for configuration issues.
 */
static esp_err_t priv_register_sd_card_pins(sd_card_hal_t* sd_card)
{
  esp_err_t err               = ESP_OK;
  esp_err_t first_reg_err     = ESP_OK; /* Track first REGISTRATION error */
  bool      share_common_pins = false;
  bool      mismatch_found    = false;

  log_info(sd_card->tag, "Pin Registration", "Registering SD Card HAL pins...");

  /* Determine if common pins should be marked as shareable */
  if (sd_card->spi_mode_enabled && sd_card->sdio_mode_enabled) {
    /* Check for mismatches first */
    if (sd_card->pin_config.spi_sclk_pin != sd_card->pin_config.sdio_clk_pin) {
      log_error(sd_card->tag,
                "Shared Pin Mismatch",
                "CONFIG ERROR: SPI CLK (%d) != SDIO CLK (%d)",
                (int)sd_card->pin_config.spi_sclk_pin,
                (int)sd_card->pin_config.sdio_clk_pin);
      mismatch_found = true;
    }
    if (sd_card->pin_config.spi_do_pin != sd_card->pin_config.sdio_cmd_pin) {
      log_error(sd_card->tag,
                "Shared Pin Mismatch",
                "CONFIG ERROR: SPI DO (%d) != SDIO CMD (%d)",
                (int)sd_card->pin_config.spi_do_pin,
                (int)sd_card->pin_config.sdio_cmd_pin);
      mismatch_found = true;
    }
    if (sd_card->pin_config.spi_di_pin != sd_card->pin_config.sdio_d0_pin) {
      log_error(sd_card->tag,
                "Shared Pin Mismatch",
                "CONFIG ERROR: SPI DI (%d) != SDIO D0 (%d)",
                (int)sd_card->pin_config.spi_di_pin,
                (int)sd_card->pin_config.sdio_d0_pin);
      mismatch_found = true;
    }

    if (mismatch_found) {
      log_error(
        sd_card->tag,
        "Configuration Error",
        "Shared pins between SPI and SDIO must be configured to the same GPIO. "
        "Please correct in menuconfig. Pin registration will proceed, but validation will likely fail.");
      share_common_pins = false; /* Treat as non-shared if mismatched */
    } else {
      share_common_pins = true;
      log_info(sd_card->tag, "Pin Sharing", "SPI/SDIO common pins will be marked as shareable.");
    }
  } else {
    log_info(sd_card->tag,
             "Pin Sharing",
             "Only one interface mode enabled, common pins not shared.");
    share_common_pins = false;
  }

  /* --- Register SPI Pins (if enabled) --- */
  if (sd_card->spi_mode_enabled) {
    log_debug(sd_card->tag, "Pin Registration", "Registering SPI pins...");

    /* SPI CS (Unique to SPI) */
    if (sd_card->pin_config.spi_cs_pin >= 0) {
      err = pin_validator_register_pin(sd_card->pin_config.spi_cs_pin,
                                       "SD Card HAL (SPI)",
                                       "SPI Chip Select",
                                       false); /* CS is never shared */
      if (err != ESP_OK && first_reg_err == ESP_OK)
        first_reg_err = err;
    }

    /* SPI CLK (Potentially Shared) */
    if (sd_card->pin_config.spi_sclk_pin >= 0) {
      err = pin_validator_register_pin(
        sd_card->pin_config.spi_sclk_pin,
        "SD Card HAL (SPI)",
        "SPI CLK",
        share_common_pins); /* Mark shareable if both modes enabled & matched */
      if (err != ESP_OK && first_reg_err == ESP_OK)
        first_reg_err = err;
    }

    /* SPI DO (Potentially Shared with SDIO CMD) */
    if (sd_card->pin_config.spi_do_pin >= 0) {
      err = pin_validator_register_pin(
        sd_card->pin_config.spi_do_pin,
        "SD Card HAL (SPI)",
        "SPI DO",
        share_common_pins); /* Mark shareable if both modes enabled & matched */
      if (err != ESP_OK && first_reg_err == ESP_OK)
        first_reg_err = err;
    }

    /* SPI DI (Potentially Shared with SDIO D0) */
    if (sd_card->pin_config.spi_di_pin >= 0) {
      err = pin_validator_register_pin(
        sd_card->pin_config.spi_di_pin,
        "SD Card HAL (SPI)",
        "SPI DI",
        share_common_pins); /* Mark shareable if both modes enabled & matched */
      if (err != ESP_OK && first_reg_err == ESP_OK)
        first_reg_err = err;
    }
  }

  /* --- Register SDIO Pins (if enabled) --- */
  if (sd_card->sdio_mode_enabled) {
    log_debug(sd_card->tag, "Pin Registration", "Registering SDIO pins...");

    /* SDIO CLK (Register ONLY if NOT shared or if mismatched) */
    if (!share_common_pins || mismatch_found) {
      if (sd_card->pin_config.sdio_clk_pin >= 0) {
        err = pin_validator_register_pin(sd_card->pin_config.sdio_clk_pin, /* Use SDIO pin number */
                                         "SD Card HAL (SDIO)",
                                         "SDIO CLK",
                                         false); /* Not shared or mismatched */
        if (err != ESP_OK && first_reg_err == ESP_OK)
          first_reg_err = err;
      }
    } else if (share_common_pins && sd_card->pin_config.sdio_clk_pin >= 0) {
      /* If shared and valid, register usage under SDIO component name but mark shareable */
      err = pin_validator_register_pin(sd_card->pin_config.sdio_clk_pin,
                                       "SD Card HAL (SDIO)",
                                       "SDIO CLK",
                                       true); /* Mark as shareable */
      if (err != ESP_OK && first_reg_err == ESP_OK)
        first_reg_err = err;
    }

    /* SDIO CMD (Register ONLY if NOT shared or if mismatched) */
    if (!share_common_pins || mismatch_found) {
      if (sd_card->pin_config.sdio_cmd_pin >= 0) {
        err = pin_validator_register_pin(sd_card->pin_config.sdio_cmd_pin, /* Use SDIO pin number */
                                         "SD Card HAL (SDIO)",
                                         "SDIO CMD",
                                         false); /* Not shared or mismatched */
        if (err != ESP_OK && first_reg_err == ESP_OK)
          first_reg_err = err;
      }
    } else if (share_common_pins && sd_card->pin_config.sdio_cmd_pin >= 0) {
      err = pin_validator_register_pin(sd_card->pin_config.sdio_cmd_pin,
                                       "SD Card HAL (SDIO)",
                                       "SDIO CMD",
                                       true); /* Mark as shareable */
      if (err != ESP_OK && first_reg_err == ESP_OK)
        first_reg_err = err;
    }

    /* SDIO D0 (Register ONLY if NOT shared or if mismatched) */
    if (!share_common_pins || mismatch_found) {
      if (sd_card->pin_config.sdio_d0_pin >= 0) {
        err = pin_validator_register_pin(sd_card->pin_config.sdio_d0_pin, /* Use SDIO pin number */
                                         "SD Card HAL (SDIO)",
                                         "SDIO D0",
                                         false); /* Not shared or mismatched */
        if (err != ESP_OK && first_reg_err == ESP_OK)
          first_reg_err = err;
      }
    } else if (share_common_pins && sd_card->pin_config.sdio_d0_pin >= 0) {
      err = pin_validator_register_pin(sd_card->pin_config.sdio_d0_pin,
                                       "SD Card HAL (SDIO)",
                                       "SDIO D0",
                                       true); /* Mark as shareable */
      if (err != ESP_OK && first_reg_err == ESP_OK)
        first_reg_err = err;
    }

    /* SDIO D1, D2, D3 (Unique to SDIO 4-bit mode) */
    if (sd_card->bus_width == k_sd_bus_width_4bit) {
      if (sd_card->pin_config.sdio_d1_pin >= 0) {
        err = pin_validator_register_pin(sd_card->pin_config.sdio_d1_pin,
                                         "SD Card HAL (SDIO)",
                                         "SDIO D1 (4-bit)",
                                         false); /* Unique pin */
        if (err != ESP_OK && first_reg_err == ESP_OK)
          first_reg_err = err;
      }
      if (sd_card->pin_config.sdio_d2_pin >= 0) {
        err = pin_validator_register_pin(sd_card->pin_config.sdio_d2_pin,
                                         "SD Card HAL (SDIO)",
                                         "SDIO D2 (4-bit)",
                                         false); /* Unique pin */
        if (err != ESP_OK && first_reg_err == ESP_OK)
          first_reg_err = err;
      }
      if (sd_card->pin_config.sdio_d3_pin >= 0) {
        err = pin_validator_register_pin(sd_card->pin_config.sdio_d3_pin,
                                         "SD Card HAL (SDIO)",
                                         "SDIO D3 (4-bit)",
                                         false); /* Unique pin */
        if (err != ESP_OK && first_reg_err == ESP_OK)
          first_reg_err = err;
      }
    }
  }

  /* Return the first *registration* error encountered, or OK if none. */
  /* Configuration mismatch errors were logged but don't cause an early exit here. */
  if (first_reg_err != ESP_OK) {
    log_error(sd_card->tag,
              "Pin Registration Error",
              "Failed to register one or more pins with validator: %s",
              esp_err_to_name(first_reg_err));
    return first_reg_err;
  }

  log_info(sd_card->tag, "Pin Registration", "Finished registering SD Card HAL pins.");
  return ESP_OK;
}