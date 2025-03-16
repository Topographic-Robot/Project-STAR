/* components/pstar_storage_hal/sd_card_hal/sd_card_hal.c */

#include "sd_card_hal.h"
#include "log_handler.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include <errno.h>
#include <ctype.h>
#include <unistd.h>
#include "bus_config.h"
#include "bus_sdio.h"
#include "bus_spi.h"
#include "bus_gpio.h"
#include "bus_event.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

/* Constants ******************************************************************/

/* Logging and identification */
static const char* const sd_card_tag = "SD Card HAL";

/* Timing and configuration constants */
static const uint32_t    sd_card_default_det_debounce_time     = 100;                /* Debounce time in ms */
static const uint32_t    sd_card_default_mount_task_stack_size = 4096;               /* Stack size for mount task */
static const UBaseType_t sd_card_default_mount_task_priority   = 5;                  /* Priority for mount task */
static const uint32_t    sd_card_default_max_mount_retries     = 3;                  /* Maximum mount retries */
static const uint32_t    sd_card_default_retry_delay_ms        = 1000;               /* Delay between retries in ms */
static const uint32_t    sd_card_default_max_retry_delay_ms    = 5000;               /* Maximum retry delay in ms */
static const uint32_t    sd_card_default_max_files             = 5;                  /* Maximum number of files */
static const uint32_t    sd_card_default_allocation_unit_size  = 16384;              /* Allocation unit size in bytes (16 KB) */
static const TickType_t  sd_card_default_mutex_timeout         = pdMS_TO_TICKS(500); /* Mutex timeout */
static const size_t      sd_card_max_path_len                  = 128;                /* Maximum path length */
static const char* const sd_card_nvs_namespace                 = "sd_card";          /* NVS namespace for SD card configuration */
static const char* const sd_card_nvs_interface_key             = "interface";        /* NVS key for last working interface */
static const char* const sd_card_nvs_bus_width_key             = "bus_width";        /* NVS key for last working bus width */

/* Default pin configurations */
/* Update in sd_card_hal.c */
static const sd_card_pin_config_t sd_card_default_pins = {
  /* GPIO pins */
  .gpio_det_pin  = GPIO_NUM_13, /* Card detect pin (high active) */
  
  /* SPI pins */
  .spi_di_pin    = GPIO_NUM_19, /* SPI DI (Data In) */
  .spi_do_pin    = GPIO_NUM_23, /* SPI DO (Data Out) */
  .spi_sclk_pin  = GPIO_NUM_14, /* SPI SCLK (Clock) */
  .spi_cs_pin    = GPIO_NUM_5,  /* SPI CS (Chip Select) */
  
  /* SDIO pins */
  .sdio_clk_pin  = GPIO_NUM_14, /* SDIO CLK */
  .sdio_cmd_pin  = GPIO_NUM_15, /* SDIO CMD */
  .sdio_d0_pin   = GPIO_NUM_2,  /* SDIO D0 (mandatory for both 1-bit and 4-bit modes) */
  .sdio_d1_pin   = GPIO_NUM_4,  /* SDIO D1 (only for 4-bit mode) */
  .sdio_d2_pin   = GPIO_NUM_12, /* SDIO D2 (only for 4-bit mode) */
  .sdio_d3_pin   = GPIO_NUM_32, /* SDIO D3 (only for 4-bit mode) */
};

/* Bus names */
static const char* const sd_card_default_gpio_bus_name = "sd_card_gpio";
static const char* const sd_card_default_spi_bus_name  = "sd_card_spi";
static const char* const sd_card_default_sdio_bus_name = "sd_card_sdio";

/* Function Prototypes ********************************************************/

static void priv_release_mutex_if_taken(sd_card_hal_t* sd_card, bool* mutex_taken);
static void priv_update_state_machine(sd_card_hal_t* sd_card, sd_state_t new_state);
static esp_err_t priv_save_working_config(sd_card_hal_t* sd_card);
static esp_err_t priv_load_working_config(sd_card_hal_t* sd_card);
static void priv_measure_card_performance(sd_card_hal_t* sd_card);
static bool priv_path_is_safe(const sd_card_hal_t* sd_card, const char* path);
static bool priv_path_is_safe_simple(const char* path);
static bool priv_is_path_within_mount(const sd_card_hal_t* sd_card, const char* path);
static esp_err_t priv_create_directory_if_needed(const sd_card_hal_t* sd_card, const char* path);
static const char* priv_sd_bus_width_to_string(sd_bus_width_t bus_width);
static esp_err_t priv_sd_card_reset(void* context);
static bool priv_sd_card_is_inserted(sd_card_hal_t* sd_card);
static esp_err_t priv_sd_card_setup_spi_with_speed(sd_card_hal_t* sd_card, 
                                                   uint32_t       clock_speed_hz);
static esp_err_t priv_sd_card_setup_sdio(sd_card_hal_t* sd_card);
static esp_err_t priv_sd_card_setup_detection(sd_card_hal_t* sd_card);
static void priv_sd_card_detection_isr(void* arg);
static esp_err_t priv_sd_card_try_interfaces(sd_card_hal_t* sd_card);
static esp_err_t priv_sd_card_mount(sd_card_hal_t* sd_card);
static esp_err_t priv_sd_card_unmount(sd_card_hal_t* sd_card);
static void priv_sd_card_mount_task(void* arg);

/* Public Functions ***********************************************************/

const char* sd_card_interface_to_string(sd_interface_type_t interface_type)
{
  switch (interface_type) {
    case k_sd_interface_sdio:
      return "SDIO (40MHz)";
    case k_sd_interface_fast_spi:
      return "Fast SPI (20MHz)";
    case k_sd_interface_normal_spi:
      return "Normal SPI (4MHz)";
    case k_sd_interface_slow_spi:
      return "Slow SPI (400kHz)";
    default:
      return "Unknown Interface";
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
  if (sd_card == NULL || tag == NULL || mount_path == NULL || 
      component_id == NULL || pin_config == NULL) {
    log_error(sd_card_tag, 
              "Init Error", 
              "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* First initialize with default settings */
  esp_err_t err = sd_card_init_default(sd_card, tag, mount_path, component_id, bus_width);
  if (err != ESP_OK) {
    return err;
  }
  
  /* Store custom pin configuration */
  memcpy(&sd_card->pin_config, pin_config, sizeof(sd_card_pin_config_t));
  
  /* Check for pin conflicts */
  if (bus_width == k_sd_bus_width_4bit && 
      sd_card->pin_config.gpio_det_pin == sd_card->pin_config.sdio_d3_pin) {
    log_warn(sd_card->tag, 
             "Pin Conflict", 
             "Card detect pin (GPIO %d) conflicts with SDIO D3 for 4-bit mode. "
             "Will default to 1-bit mode for better compatibility.", 
             sd_card->pin_config.gpio_det_pin);
    
    /* Default to 1-bit mode when there's a pin conflict */
    sd_card->bus_width = k_sd_bus_width_1bit;
  }
  
  /* Also check for invalid pin assignments */
  bool valid_pins = true;
  if (sd_card->pin_config.gpio_det_pin < 0) {
    log_error(sd_card->tag,
              "Invalid Pins",
              "Card detect pin is invalid: %d",
              sd_card->pin_config.gpio_det_pin);
    valid_pins = false;
  }
  
  if (sd_card->pin_config.sdio_clk_pin < 0 ||
      sd_card->pin_config.sdio_cmd_pin < 0 ||
      sd_card->pin_config.sdio_d0_pin < 0) {
    log_error(sd_card->tag,
              "Invalid Pins",
              "Required SDIO pins are invalid: CLK=%d, CMD=%d, D0=%d",
              sd_card->pin_config.sdio_clk_pin,
              sd_card->pin_config.sdio_cmd_pin,
              sd_card->pin_config.sdio_d0_pin);
    valid_pins = false;
  }
  
  if (sd_card->pin_config.spi_di_pin < 0 ||
      sd_card->pin_config.spi_do_pin < 0 ||
      sd_card->pin_config.spi_sclk_pin < 0 ||
      sd_card->pin_config.spi_cs_pin < 0) {
    log_error(sd_card->tag,
              "Invalid Pins",
              "Required SPI pins are invalid: DI=%d, DO=%d, SCLK=%d, CS=%d",
              sd_card->pin_config.spi_di_pin,
              sd_card->pin_config.spi_do_pin,
              sd_card->pin_config.spi_sclk_pin,
              sd_card->pin_config.spi_cs_pin);
    valid_pins = false;
  }
  
  if (!valid_pins) {
    return ESP_ERR_INVALID_ARG;
  }
  
  log_info(sd_card->tag, 
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
      component_id == NULL) {
    log_error(sd_card_tag, 
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
    log_error(sd_card_tag,
              "Invalid bus width",
              "Bus width must be 1 or 4, got: %d",
              bus_width);
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Validate mount path using the simpler path check */
  if (!priv_path_is_safe_simple(mount_path)) {
    log_error(sd_card_tag,
              "Invalid mount path",
              "Mount path '%s' is not safe",
              mount_path);
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Default task configuration */
  sd_card_task_config_t task_config = {
    .stack_size    = sd_card_default_mount_task_stack_size,
    .priority      = sd_card_default_mount_task_priority,
    .mutex_timeout = sd_card_default_mutex_timeout,
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
    .tag                          = tag,
    .mount_path                   = mount_path,
    .max_files                    = sd_card_default_max_files,
    .allocation_unit_size         = sd_card_default_allocation_unit_size,
    .card_detect_low_active       = false,                                /* Default to high-active (1=card present) */
    .bus_width                    = bus_width,                            /* Use provided bus width */
    .pin_config                   = sd_card_default_pins,                 /* Use default pin configuration */
    .state                        = k_sd_state_idle,                      /* Initial state */
    .error_count                  = 0,                                    /* No errors initially */
    .last_state_change_time       = 0,                                    /* Will be set during init */
    .current_interface            = k_sd_interface_count,
    .interface_info               = { {0}, {0}, {0}, {0} },
    .interface_discovery_complete = false,
    .interface_attempt_count      = 0,
    .card                         = NULL,                                 /* Will call sdmmc_card_init() */
    .mutex                        = NULL,                                 /* Will call xSemaphoreCreateMutex() */
    .card_available               = false,
    .initialized                  = false,
    .mount_task_exit_requested    = false,
    .mount_task_handle            = NULL,                                 /* Will call xTaskCreate() */
    .error_handler                = { 0 },                                /* Will call error_handler_init() */
    .component_id                 = component_id,
    .bus_manager                  = { 0 },                                /* Will call bus_manager_init() */
    .task_config                  = task_config,                          /* Use default task configuration */
    .performance                  = performance                           /* Initialize performance metrics */
  };

  /* Copy default configuration to sd_card */
  memcpy(sd_card, &sd_card_default, sizeof(sd_card_hal_t));

  /* Check for pin conflicts */
  if (bus_width == k_sd_bus_width_4bit && 
      sd_card->pin_config.gpio_det_pin == sd_card->pin_config.sdio_d3_pin) {
    log_warn(sd_card_tag, 
             "Pin Conflict", 
             "Card detect pin (GPIO %d) conflicts with SDIO D3 for 4-bit mode. "
             "Will default to 1-bit mode for better compatibility.", 
             sd_card->pin_config.gpio_det_pin);
    
    /* Default to 1-bit mode when there's a pin conflict */
    sd_card->bus_width = k_sd_bus_width_1bit;
  }

  /* Initialize error handler */
  error_handler_init(&sd_card->error_handler, 
                     sd_card_default_max_mount_retries,
                     sd_card_default_retry_delay_ms,
                     sd_card_default_max_retry_delay_ms,
                     priv_sd_card_reset, /* Reset function */ 
                     sd_card);           /* Reset context */

  /* Initialize mutex */
  sd_card->mutex = xSemaphoreCreateMutex();
  if (sd_card->mutex == NULL) {
    log_error(sd_card_tag, 
              "Failed to create mutex", 
              "sd_card: %p", 
              sd_card);
    return ESP_ERR_NO_MEM;
  }

  /* Initialize bus manager */
  err = pstar_bus_manager_init(&sd_card->bus_manager, tag);
  if (err != ESP_OK) {
    log_error(sd_card_tag, 
              "Failed to initialize bus manager", 
              "err: %d", 
              err);
    vSemaphoreDelete(sd_card->mutex);
    sd_card->mutex = NULL;
    return err;
  }

  /* Initialize NVS if not already initialized */
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    /* NVS partition was truncated and needs to be erased */
    err = nvs_flash_erase();
    if (err != ESP_OK) {
      log_warn(sd_card->tag,
               "NVS Warning",
               "Failed to erase NVS: %s",
               esp_err_to_name(err));
    }
    
    /* Retry initialization */
    err = nvs_flash_init();
  }
  
  if (err != ESP_OK) {
    log_warn(sd_card->tag,
             "NVS Warning",
             "Failed to initialize NVS: %s",
             esp_err_to_name(err));
    /* Non-critical error, continue without NVS */
  }

  log_info(sd_card_tag, 
           "SD Card Default Init", 
           "SD card HAL initialized with %s bus width mode", 
           priv_sd_bus_width_to_string(sd_card->bus_width));
  return ESP_OK;
}

esp_err_t sd_card_set_task_config(sd_card_hal_t* sd_card, 
                                  uint32_t       stack_size,
                                  UBaseType_t    priority,
                                  TickType_t     mutex_timeout)
{
  if (sd_card == NULL) {
    log_error(sd_card_tag, 
              "Set Task Config Error", 
              "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }
  
  if (sd_card->initialized) {
    log_warn(sd_card_tag, 
             "Set Task Config Warning", 
             "Cannot change task configuration after initialization");
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Validate configuration */
  if (stack_size < 2048) {
    log_warn(sd_card_tag,
             "Config Warning",
             "Stack size %lu is too small, using minimum of 2048",
             stack_size);
    stack_size = 2048;
  }
  
  if (mutex_timeout == 0) {
    log_warn(sd_card_tag,
             "Config Warning",
             "Mutex timeout of 0 is not allowed, using default");
    mutex_timeout = sd_card_default_mutex_timeout;
  }
  
  /* Update task configuration */
  sd_card->task_config.stack_size    = stack_size;
  sd_card->task_config.priority      = priority;
  sd_card->task_config.mutex_timeout = mutex_timeout;
  
  log_info(sd_card->tag, 
           "Task Config Updated", 
           "SD card task configuration updated: stack=%lu, priority=%u, timeout=%lu",
           stack_size,
           priority,
           mutex_timeout);
  return ESP_OK;
}

esp_err_t sd_card_init(sd_card_hal_t* sd_card, 
                       const char*    parent_id, 
                       uint32_t       priority)
{
  /* Validate arguments */
  if (sd_card == NULL) {
    log_error(sd_card_tag, 
              "Init Error", 
              "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if already initialized */
  if (sd_card->initialized) {
    log_warn(sd_card->tag, 
             "Init Warning", 
             "SD card HAL already initialized");
    return ESP_OK;
  }

  log_info(sd_card->tag, 
           "Init Started", 
           "Initializing SD card HAL with %s bus width", 
           priv_sd_bus_width_to_string(sd_card->bus_width));

  /* Set up card detection GPIO */
  esp_err_t err = priv_sd_card_setup_detection(sd_card);
  if (err != ESP_OK) {
    log_error(sd_card->tag, 
              "Init Error", 
              "Failed to set up card detection: %s", 
              esp_err_to_name(err));
    return err;
  }

  /* Initialize mount task */
  BaseType_t task_created = xTaskCreate(priv_sd_card_mount_task,
                                        "sd_mount_task",
                                        sd_card->task_config.stack_size,
                                        (void*)sd_card,
                                        sd_card->task_config.priority,
                                        &sd_card->mount_task_handle);

  if (task_created != pdPASS) {
    log_error(sd_card_tag, 
              "Failed to create mount task", 
              "sd_card: %p", 
              sd_card);
    /* Cleanup detection setup */
    bool mutex_taken = false;
    if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
      mutex_taken = true;
      pstar_bus_gpio_isr_remove(&sd_card->bus_manager, 
                          sd_card_default_gpio_bus_name, 
                          sd_card->pin_config.gpio_det_pin);
      priv_release_mutex_if_taken(sd_card, &mutex_taken);
    }
    return ESP_ERR_NO_MEM;
  }

  /* Mark as initialized */
  sd_card->initialized = true;

  log_info(sd_card->tag, 
           "Init Complete", 
           "SD card HAL initialization complete");
  return ESP_OK;
}

bool sd_card_is_available(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    return false;
  }

  bool available   = false;
  bool mutex_taken = false;
  
  /* Take mutex for thread-safe access */
  if (sd_card->mutex != NULL && 
      xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
    mutex_taken = true;
    available   = sd_card->card_available;
    priv_release_mutex_if_taken(sd_card, &mutex_taken);
  }
  
  return available;
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
    return k_sd_interface_count;
  }

  sd_interface_type_t interface   = k_sd_interface_count;
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

esp_err_t sd_card_get_performance(sd_card_hal_t*         sd_card, 
                                  sd_card_performance_t* performance)
{
  if (sd_card == NULL || performance == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  
  bool mutex_taken = false;
  
  /* Take mutex for thread-safe access */
  if (sd_card->mutex != NULL && 
      xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
    mutex_taken = true;
    
    /* Copy performance metrics */
    memcpy(performance, &sd_card->performance, sizeof(sd_card_performance_t));
    
    /* If performance hasn't been measured yet and card is available, flag for measurement */
    if (performance->last_measured == 0 && sd_card->card_available) {
      sd_card->performance.measurement_needed = true;
    }
    
    priv_release_mutex_if_taken(sd_card, &mutex_taken);
    return ESP_OK;
  }
  
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
      log_warn(sd_card->tag,
               "Remount Warning",
               "Cannot remount - no card inserted");
      priv_release_mutex_if_taken(sd_card, &mutex_taken);
      return ESP_ERR_NOT_FOUND;
    }
    
    /* Unmount if currently mounted */
    if (sd_card->card_available) {
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
    } else if (sd_card->card_available) {
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
    log_error(sd_card->tag,
              "Remount Error",
              "Failed to acquire mutex for remount operation");
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
    log_error(sd_card->tag,
              "Invalid bus width",
              "Bus width must be 1 or 4, got: %d",
              bus_width);
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
    if (bus_width == k_sd_bus_width_4bit && 
        sd_card->pin_config.gpio_det_pin == sd_card->pin_config.sdio_d3_pin) {
      log_error(sd_card->tag, 
                "Pin Conflict", 
                "Cannot use 4-bit mode - card detect pin (GPIO %d) conflicts with SDIO D3", 
                sd_card->pin_config.gpio_det_pin);
      priv_release_mutex_if_taken(sd_card, &mutex_taken);
      return ESP_ERR_INVALID_STATE;
    }
    
    /* Update bus width */
    log_info(sd_card->tag,
             "Bus Width Change",
             "Changing from %s to %s bus width",
             priv_sd_bus_width_to_string(sd_card->bus_width),
             priv_sd_bus_width_to_string(bus_width));
    
    sd_card->bus_width = bus_width;
    
    /* If card is currently mounted, force a remount to apply the new bus width */
    if (sd_card->card_available) {
      /* Release mutex since remount will take it again */
      priv_release_mutex_if_taken(sd_card, &mutex_taken);
      return sd_card_force_remount(sd_card);
    }
    
    /* Release mutex */
    priv_release_mutex_if_taken(sd_card, &mutex_taken);
  } else {
    log_error(sd_card->tag,
              "Bus Width Error",
              "Failed to acquire mutex for bus width change");
    return ESP_ERR_TIMEOUT;
  }
  
  return result;
}

esp_err_t sd_card_cleanup(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    log_error(sd_card_tag, 
              "Cleanup Error", 
              "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(sd_card->tag, 
           "Cleanup Started", 
           "Cleaning up SD card HAL resources");
  
  esp_err_t result       = ESP_OK;
  bool      mutex_taken = false;

  /* Signal mount task to exit */
  if (sd_card->mount_task_handle != NULL) {
    sd_card->mount_task_exit_requested = true;
    
    /* Wait for task to exit with timeout */
    TickType_t start_time = xTaskGetTickCount();
    const TickType_t max_wait_time = pdMS_TO_TICKS(1000); /* 1 second max wait */
    
    while (sd_card->mount_task_handle != NULL && 
           (xTaskGetTickCount() - start_time) < max_wait_time) {
      vTaskDelay(pdMS_TO_TICKS(10));
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
  if (sd_card->card_available) {
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

  /* Delete mount task if it's still running */
  if (sd_card->mount_task_handle != NULL) {
    vTaskDelete(sd_card->mount_task_handle);
    sd_card->mount_task_handle = NULL;
  }

  /* Clean up GPIO ISR */
  if (sd_card->initialized) {
    pstar_bus_gpio_isr_remove(&sd_card->bus_manager, 
                        sd_card_default_gpio_bus_name, 
                        sd_card->pin_config.gpio_det_pin);
  }

  /* Clean up bus manager */
  pstar_bus_manager_deinit(&sd_card->bus_manager);

  /* Free card structure */
  if (sd_card->card != NULL) {
    free(sd_card->card);
    sd_card->card = NULL;
  }

  /* Update state */
  sd_card->initialized                  = false;
  sd_card->card_available               = false;
  sd_card->interface_discovery_complete = false;
  sd_card->current_interface            = k_sd_interface_count;
  sd_card->state                        = k_sd_state_idle;

  /* Give mutex before deleting it */
  priv_release_mutex_if_taken(sd_card, &mutex_taken);

  /* Delete mutex */
  if (sd_card->mutex != NULL) {
    vSemaphoreDelete(sd_card->mutex);
    sd_card->mutex = NULL;
  }

  log_info(sd_card->tag, 
          "Cleanup Complete", 
          "SD card HAL resources cleaned up successfully");
  return result;
}

/* Private Functions **********************************************************/

/**
 * @brief Safely releases a mutex if it was taken
 * 
 * @param[in] sd_card     Pointer to the SD card HAL instance
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
  const char* states[] = {
    "IDLE",
    "CARD_INSERTED",
    "INTERFACE_DISCOVERY",
    "INTERFACE_READY",
    "ERROR",
    "FAILED"
  };
  
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
    sd_card->interface_attempt_count = 0;
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
    sd_card->error_count = 0;
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
  if (sd_card == NULL || !sd_card->card_available) {
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Only save if we have a valid interface */
  if (sd_card->current_interface >= k_sd_interface_count) {
    return ESP_ERR_INVALID_STATE;
  }
  
  nvs_handle_t nvs_handle;
  esp_err_t    err;
  
  /* Open NVS namespace */
  err = nvs_open(sd_card_nvs_namespace, NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    log_warn(sd_card->tag,
             "NVS Warning",
             "Failed to open NVS namespace: %s",
             esp_err_to_name(err));
    return err;
  }
  
  /* Save current interface */
  err = nvs_set_u8(nvs_handle, sd_card_nvs_interface_key, 
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
  err = nvs_set_u8(nvs_handle, sd_card_nvs_bus_width_key, 
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
    log_warn(sd_card->tag,
             "NVS Warning",
             "Failed to commit NVS changes: %s",
             esp_err_to_name(err));
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
  err = nvs_open(sd_card_nvs_namespace, NVS_READONLY, &nvs_handle);
  if (err != ESP_OK) {
    /* Not finding the namespace is normal on first boot, just return */
    if (err == ESP_ERR_NVS_NOT_FOUND) {
      log_info(sd_card->tag,
               "NVS Info",
               "No saved SD card configuration found");
      return ESP_OK;
    }
    
    log_warn(sd_card->tag,
             "NVS Warning",
             "Failed to open NVS namespace: %s",
             esp_err_to_name(err));
    return err;
  }
  
  /* Load interface */
  uint8_t interface_val;
  err = nvs_get_u8(nvs_handle, sd_card_nvs_interface_key, &interface_val);
  if (err == ESP_OK) {
    /* Validate the loaded value */
    if (interface_val < k_sd_interface_count) {
      sd_card->current_interface = (sd_interface_type_t)interface_val;
      /* Mark that we have a previously working interface */
      sd_card->interface_info[interface_val].successful        = true;
      sd_card->interface_info[interface_val].last_success_time = esp_timer_get_time();
    } else {
      log_warn(sd_card->tag,
               "NVS Warning",
               "Invalid interface value in NVS: %d",
               interface_val);
    }
  } else if (err != ESP_ERR_NVS_NOT_FOUND) {
    log_warn(sd_card->tag,
             "NVS Warning",
             "Failed to read interface from NVS: %s",
             esp_err_to_name(err));
  }
  
  /* Load bus width */
  uint8_t bus_width_val;
  err = nvs_get_u8(nvs_handle, sd_card_nvs_bus_width_key, &bus_width_val);
  if (err == ESP_OK) {
    /* Validate the loaded value */
    if (bus_width_val == k_sd_bus_width_1bit || 
        bus_width_val == k_sd_bus_width_4bit) {
      sd_card->bus_width = (sd_bus_width_t)bus_width_val;
    } else {
      log_warn(sd_card->tag,
               "NVS Warning",
               "Invalid bus width value in NVS: %d",
               bus_width_val);
    }
  } else if (err != ESP_ERR_NVS_NOT_FOUND) {
    log_warn(sd_card->tag,
             "NVS Warning",
             "Failed to read bus width from NVS: %s",
             esp_err_to_name(err));
  }
  
  /* Close NVS handle */
  nvs_close(nvs_handle);
  
  /* If we successfully loaded a working configuration, log it */
  if (sd_card->current_interface < k_sd_interface_count) {
    log_info(sd_card->tag,
             "Config Loaded",
             "Loaded previous working configuration - Interface: %s, Bus Width: %d-bit",
             sd_card_interface_to_string(sd_card->current_interface),
             sd_card->bus_width);
    return ESP_OK;
  }
  
  log_info(sd_card->tag,
           "Config Not Found",
           "No valid previous working configuration found");
  return ESP_OK;
}

static void priv_measure_card_performance(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL || !sd_card->card_available || sd_card->card == NULL) {
    return;
  }
  
  /* Skip if performance was measured recently (less than 1 minute ago) */
  int64_t now = esp_timer_get_time();
  if (sd_card->performance.last_measured > 0 && 
      ((now - sd_card->performance.last_measured) < 60000000)) {
    return;
  }
  
  log_info(sd_card->tag,
           "Performance Test",
           "Measuring SD card performance metrics");
  
  /* Create a temporary buffer for testing */
  const size_t buffer_size = 256 * 1024; /* 256 KB test size */
  uint8_t*     buffer      = heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
  if (buffer == NULL) {
    log_error(sd_card->tag,
              "Performance Error",
              "Failed to allocate memory for performance test");
    return;
  }
  
  /* Fill buffer with test pattern */
  for (size_t i = 0; i < buffer_size; i++) {
    buffer[i] = (uint8_t)(i & 0xFF);
  }
  
  /* Create a system directory for test files if it doesn't exist */
  char system_dir[sd_card_max_path_len];
  
  /* Safely construct system directory path */
  if (strlen(sd_card->mount_path) > sd_card_max_path_len - 9) { /* 9 = length of "/.system" + null terminator */
    log_error(sd_card->tag,
              "Performance Error",
              "Mount path too long for creating system directory");
    free(buffer);
    return;
  }
  
  strcpy(system_dir, sd_card->mount_path);
  strcat(system_dir, "/.system");
  
  /* Check if path is safe before proceeding */
  if (!priv_path_is_safe_simple(system_dir) || !priv_is_path_within_mount(sd_card, system_dir)) {
    log_error(sd_card->tag,
              "Performance Error",
              "System directory path '%s' is not valid",
              system_dir);
    free(buffer);
    return;
  }
  
  /* Create the system directory if it doesn't exist */
  esp_err_t dir_err = priv_create_directory_if_needed(sd_card, system_dir);
  if (dir_err != ESP_OK) {
    log_error(sd_card->tag,
              "Performance Error",
              "Failed to create system directory: %s",
              esp_err_to_name(dir_err));
    free(buffer);
    return;
  }
  
  /* Create test file path in the system directory using safe string operations */
  char test_path[sd_card_max_path_len];
  
  /* Safe copy and concatenation */
  if (strlen(system_dir) > sd_card_max_path_len - 11) { /* 11 = length of "/perf_test" + null terminator */
    log_error(sd_card->tag,
              "Performance Error",
              "System directory path too long, cannot append test filename");
    free(buffer);
    return;
  }
  
  strcpy(test_path, system_dir);
  strcat(test_path, "/perf_test");
  
  /* Measure write speed */
  int64_t write_start = esp_timer_get_time();
  
  /* Create test file and write to it */
  FILE* f = fopen(test_path, "wb");
  if (f == NULL) {
    log_error(sd_card->tag,
              "Performance Error",
              "Failed to create test file: %s", 
              strerror(errno));
    free(buffer);
    return;
  }
  
  /* Write buffer to file */
  size_t bytes_written = fwrite(buffer, 1, buffer_size, f);
  fclose(f);
  
  int64_t write_end = esp_timer_get_time();
  
  /* Check if write was successful */
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
  
  /* Measure read speed */
  int64_t read_start = esp_timer_get_time();
  
  /* Open test file for reading */
  f = fopen(test_path, "rb");
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
  
  /* Read file into buffer */
  size_t bytes_read = fread(buffer, 1, buffer_size, f);
  fclose(f);
  
  int64_t read_end = esp_timer_get_time();
  
  /* Clean up test file */
  unlink(test_path);
  
  /* Check if read was successful */
  if (bytes_read != buffer_size) {
    log_error(sd_card->tag,
              "Performance Error",
              "Read test incomplete: %zu/%zu bytes read",
              bytes_read,
              buffer_size);
    free(buffer);
    return;
  }
  
  /* Free buffer */
  free(buffer);
  
  /* Calculate speeds */
  float write_time_sec = (float)(write_end - write_start) / 1000000.0f;
  float read_time_sec  = (float)(read_end - read_start) / 1000000.0f;
  
  float write_speed_kbps = (buffer_size * 8.0f / 1000.0f) / write_time_sec;
  float read_speed_kbps  = (buffer_size * 8.0f / 1000.0f) / read_time_sec;
  
  /* Update metrics */
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
 * @brief Checks if a path is safe to use (no directory traversal attacks)
 * 
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @param[in] path Path to check
 * @return true if path is safe, false otherwise
 */
static bool priv_path_is_safe(const sd_card_hal_t* sd_card, const char* path)
{
  if (sd_card == NULL || path == NULL) {
    log_error(sd_card_tag, 
              "Path Validation Error", 
              "NULL parameter provided to path validation");
    return false;
  }
  
  /* Path length check to prevent buffer overflow attacks */
  if (strlen(path) >= sd_card_max_path_len) {
    log_error(sd_card_tag, 
              "Path Validation Error", 
              "Path too long: %zu characters exceeds maximum of %zu", 
              strlen(path), 
              sd_card_max_path_len - 1);
    return false;
  }
  
  /* Don't allow absolute paths that don't start with the SD card mount point */
  if (path[0] == '/') {
    /* Check if the path starts with the configured mount point prefix */
    size_t mount_path_len = strlen(sd_card->mount_path);
    if (strncmp(path, sd_card->mount_path, mount_path_len) != 0) {
      log_error(sd_card_tag, 
                "Path Validation Error", 
                "Path '%s' does not begin with configured mount point '%s'", 
                path, 
                sd_card->mount_path);
      return false;
    }
  }
  
  /* Check for directory traversal attempts using various common patterns */
  if (strstr(path, "..") != NULL ||   /* Standard directory traversal */
      strstr(path, "./") != NULL ||   /* Potential hiding technique */
      strstr(path, "//") != NULL ||   /* Multiple slashes can bypass filters */
      strstr(path, "\\") != NULL) {   /* Backslashes can be problematic */
    log_error(sd_card_tag, 
              "Path Validation Error", 
              "Path '%s' contains potentially unsafe directory traversal patterns", 
              path);
    return false;
  }
  
  /* Ensure the path only contains allowed characters */
  for (const char* c = path; *c != '\0'; c++) {
    /* Allow alphanumeric characters, common punctuation, slashes and dots */
    if (!(isalnum((unsigned char)*c) || *c == '_' || *c == '-' || *c == '.' || 
          *c == '/' || *c == ' ' || *c == '+' || *c == '=' || *c == ',' || 
          *c == '@' || *c == '(' || *c == ')')) {
      log_error(sd_card_tag, 
                "Path Validation Error", 
                "Path '%s' contains invalid character '%c' (0x%02X)",
                path, *c, (unsigned char)*c);
      return false;
    }
  }
  
  return true;
}

/**
 * @brief Updates the path safety check with the current mount point
 * 
 * This simpler version validates paths but doesn't require an sd_card instance
 * 
 * @param[in] path Path to check
 * @return true if path is safe, false otherwise
 */
static bool priv_path_is_safe_simple(const char* path)
{
  if (path == NULL) {
    return false;
  }
  
  /* Check for directory traversal attempts using various common patterns */
  if (strstr(path, "..") != NULL ||   /* Standard directory traversal */
      strstr(path, "./") != NULL ||   /* Potential hiding technique */
      strstr(path, "//") != NULL ||   /* Multiple slashes can bypass filters */
      strstr(path, "\\") != NULL) {   /* Backslashes can be problematic */
    log_error(sd_card_tag, 
              "Path Validation Error", 
              "Path '%s' contains potentially unsafe directory traversal patterns", 
              path);
    return false;
  }
  
  /* Ensure the path only contains allowed characters */
  for (const char* c = path; *c != '\0'; c++) {
    /* Allow alphanumeric characters, common punctuation, slashes and dots */
    if (!(isalnum((unsigned char)*c) || *c == '_' || *c == '-' || *c == '.' || 
          *c == '/' || *c == ' ' || *c == '+' || *c == '=' || *c == ',' || 
          *c == '@' || *c == '(' || *c == ')')) {
      log_error(sd_card_tag, 
                "Path Validation Error", 
                "Path '%s' contains invalid character '%c' (0x%02X)",
                path, *c, (unsigned char)*c);
      return false;
    }
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
    if (strncmp(path, sd_card->mount_path, mount_path_len) != 0) {
      log_error(sd_card_tag, 
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
    log_error(sd_card_tag, 
              "Directory Error", 
              "NULL parameter provided to directory creation");
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
      log_debug(sd_card_tag, 
                "Directory Already Exists", 
                "Path exists and is a directory: %s", 
                path);
      return ESP_OK;
    } else {
      /* Path exists but is not a directory */
      log_error(sd_card_tag, 
                "Directory Error", 
                "Path exists but is not a directory: %s (type: %lu)", 
                path, 
                st.st_mode & S_IFMT);
      return ESP_ERR_INVALID_STATE;
    }
  }
  
  /* Directory doesn't exist, create it */
  log_info(sd_card_tag, 
           "Creating Directory", 
           "Creating directory: %s", 
           path);
  
  if (mkdir(path, 0755) != 0) {
    /* Check specific error conditions */
    if (errno == ENOENT) {
      log_error(sd_card_tag, 
                "Directory Error", 
                "Failed to create directory %s: Parent directory doesn't exist", 
                path);
    } else if (errno == EACCES) {
      log_error(sd_card_tag, 
                "Directory Error", 
                "Failed to create directory %s: Permission denied", 
                path);
    } else {
      log_error(sd_card_tag, 
                "Directory Error", 
                "Failed to create directory %s: %s (errno: %d)", 
                path, 
                strerror(errno), 
                errno);
    }
    return ESP_FAIL;
  }
  
  log_info(sd_card_tag, 
           "Directory Created", 
           "Successfully created directory: %s", 
           path);
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
 * @brief Resets the SD card hardware and remounts if needed
 * 
 * This function is called by the error handler when an error occurs to try
 * to recover from the error by resetting the SD card hardware and remounting.
 * 
 * @param[in] context Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
static esp_err_t priv_sd_card_reset(void* context)
{
  esp_err_t      result      = ESP_OK;
  bool           mutex_taken = false;
  sd_card_hal_t* sd_card     = (sd_card_hal_t*)context;
  
  if (sd_card == NULL) {
    log_error(sd_card_tag, 
              "Reset Error", 
              "Invalid context pointer");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(sd_card->tag, 
           "Reset Started", 
           "Resetting SD card hardware");

  /* Take mutex for thread safety */
  if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) != pdTRUE) {
    log_error(sd_card->tag, 
              "Reset Error", 
              "Failed to take mutex during reset");
    return ESP_ERR_TIMEOUT;
  }
  mutex_taken = true;

  /* Unmount first if currently mounted */
  if (sd_card->card_available) {
    esp_err_t unmount_result = priv_sd_card_unmount(sd_card);
    if (unmount_result != ESP_OK) {
      log_error(sd_card->tag,
                "Reset Error",
                "Failed to unmount SD card: %s",
                esp_err_to_name(unmount_result));
      result = unmount_result;
      priv_release_mutex_if_taken(sd_card, &mutex_taken);
      return result;
    }
  }

  /* Start interface discovery again */
  sd_card->interface_discovery_complete = false;
  
  /* Set current interface to the last successful one, if any */
  sd_card->current_interface = k_sd_interface_count;
  for (int i = 0; i < k_sd_interface_count; i++) {
    if (sd_card->interface_info[i].successful) {
      if (sd_card->current_interface == k_sd_interface_count || 
          sd_card->interface_info[i].last_success_time > 
          sd_card->interface_info[sd_card->current_interface].last_success_time) {
        sd_card->current_interface = (sd_interface_type_t)i;
      }
    }
  }

  /* If card is inserted, try to mount with preferred interface */
  if (priv_sd_card_is_inserted(sd_card)) {
    priv_update_state_machine(sd_card, k_sd_state_interface_discovery);
    esp_err_t mount_result = priv_sd_card_try_interfaces(sd_card);
    if (mount_result != ESP_OK) {
      log_error(sd_card->tag, 
                "Reset Error", 
                "Failed to initialize SD card interface: %s", 
                esp_err_to_name(mount_result));
      result = mount_result;
      priv_release_mutex_if_taken(sd_card, &mutex_taken);
      return result;
    }
    /* If mounted successfully, update state */
    if (sd_card->card_available) {
      priv_update_state_machine(sd_card, k_sd_state_interface_ready);
    }
  } else {
    priv_update_state_machine(sd_card, k_sd_state_idle);
  }

  /* Release mutex */
  priv_release_mutex_if_taken(sd_card, &mutex_taken);

  if (result == ESP_OK) {
    log_info(sd_card->tag, 
             "Reset Complete", 
             "SD card reset successful");
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

  bool card_inserted = false;
  bool mutex_taken   = false;
  
  /* Take mutex for thread-safe access */
  if (sd_card->mutex != NULL && 
      xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
    mutex_taken = true;
    uint32_t  level = 0;
    esp_err_t err;
    
    /* Read the card detect pin */
    err = pstar_bus_gpio_get_level(&sd_card->bus_manager, 
                             sd_card_default_gpio_bus_name, 
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
  } else {
    log_warn(sd_card->tag,
             "Detection Warning",
             "Failed to acquire mutex for card detection, defaulting to not inserted");
  }
  
  /* Release mutex if taken */
  priv_release_mutex_if_taken(sd_card, &mutex_taken);
  
  return card_inserted;
}

/**
 * @brief Sets up the SPI interface for SD card communication with specified speed
 * 
 * @param[in] sd_card       Pointer to the SD card HAL instance
 * @param[in] clock_speed_hz SPI clock speed in Hz
 * @return ESP_OK if successful, appropriate error code otherwise
 */
static esp_err_t priv_sd_card_setup_spi_with_speed(sd_card_hal_t* sd_card, 
                                                   uint32_t       clock_speed_hz)
{
  if (sd_card == NULL) {
    log_error(sd_card_tag, 
              "SPI Setup Error", 
              "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(sd_card->tag, 
           "SPI Setup", 
           "Setting up SPI interface for SD card at %lu Hz", 
           clock_speed_hz);

  /* Create SPI bus configuration */
  pstar_bus_config_t* spi_config   = NULL;
  sdspi_dev_handle_t  sdspi_handle = 0;
  esp_err_t           err          = ESP_OK;

  spi_config = pstar_bus_config_create_spi(sd_card_default_spi_bus_name, 
                                     SPI2_HOST, 
                                     k_pstar_mode_polling);
  if (spi_config == NULL) {
    log_error(sd_card->tag, 
              "SPI Setup Error", 
              "Failed to create SPI bus configuration");
    return ESP_ERR_NO_MEM;
  }

  /* Configure SPI pins from the pin configuration */
  spi_config->config.spi.bus_config.miso_io_num     = sd_card->pin_config.spi_di_pin;   /* Data In */
  spi_config->config.spi.bus_config.mosi_io_num     = sd_card->pin_config.spi_do_pin;   /* Data Out */
  spi_config->config.spi.bus_config.sclk_io_num     = sd_card->pin_config.spi_sclk_pin; /* Clock */
  spi_config->config.spi.bus_config.quadwp_io_num   = -1;                               /* Not used */
  spi_config->config.spi.bus_config.quadhd_io_num   = -1;                               /* Not used */
  spi_config->config.spi.bus_config.max_transfer_sz = 4092;                             /* Max SPI transfer size */

  /* Configure SPI device */
  spi_config->config.spi.dev_config.clock_speed_hz = clock_speed_hz;
  spi_config->config.spi.dev_config.mode           = 0;                                 /* SPI mode 0 */
  spi_config->config.spi.dev_config.spics_io_num   = sd_card->pin_config.spi_cs_pin;    /* Chip Select */
  spi_config->config.spi.dev_config.queue_size     = 7;                                 /* Queue size for transactions */
  spi_config->config.spi.dev_config.flags          = 0;                                 /* No special flags */

  /* Check if the bus already exists */
  pstar_bus_config_t* existing_spi = pstar_bus_manager_find_bus(&sd_card->bus_manager, sd_card_default_spi_bus_name);
  if (existing_spi != NULL) {
    /* Clean up the existing configuration first */
    if (existing_spi->initialized) {
      err = pstar_bus_config_deinit(existing_spi);
      if (err != ESP_OK) {
        log_warn(sd_card->tag,
                 "SPI Cleanup Warning",
                 "Failed to deinitialize existing SPI bus: %s",
                 esp_err_to_name(err));
        /* Proceed despite warning, but be aware this might cause issues */
      }
    }
    err = pstar_bus_manager_remove_bus(&sd_card->bus_manager, sd_card_default_spi_bus_name);
    if (err != ESP_OK) {
      log_warn(sd_card->tag,
               "SPI Cleanup Warning",
               "Failed to remove existing SPI bus from manager: %s",
               esp_err_to_name(err));
      /* Proceed despite warning, but be aware this might cause issues */
    }
  }

  /* Add SPI bus to the manager */
  err = pstar_bus_manager_add_bus(&sd_card->bus_manager, spi_config);
  if (err != ESP_OK) {
    log_error(sd_card->tag, 
              "SPI Setup Error", 
              "Failed to add SPI bus to manager: %s", 
              esp_err_to_name(err));
    pstar_bus_config_destroy(spi_config);
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
    esp_err_t remove_err = pstar_bus_manager_remove_bus(&sd_card->bus_manager, sd_card_default_spi_bus_name);
    if (remove_err != ESP_OK) {
      log_warn(sd_card->tag,
               "SPI Cleanup Warning",
               "Failed to remove SPI bus from manager during cleanup: %s",
               esp_err_to_name(remove_err));
      /* Continue with cleanup despite the warning */
    }
    pstar_bus_config_destroy(spi_config);
    return err;
  }

  /* Initialize default SPI operations */
  pstar_bus_spi_init_default_ops(&spi_config->config.spi.ops);

  /* Create SDSPI host configuration */
  sdspi_device_config_t device_config = {
    .host_id  = spi_config->config.spi.host,
    .gpio_cs  = spi_config->config.spi.dev_config.spics_io_num,
    .gpio_cd  = sd_card->pin_config.gpio_det_pin,
    .gpio_wp  = -1, /* Write protect not used */
    .gpio_int = -1, /* Interrupt not used */
  };

  /* Deinitialize SDSPI host if already initialized */
  sdspi_host_deinit();

  /* Initialize SD SPI driver */
  err = sdspi_host_init();
  if (err != ESP_OK) {
    log_error(sd_card->tag, 
              "SPI Setup Error", 
              "Failed to initialize SDSPI host: %s", 
              esp_err_to_name(err));
    /* Clean up properly in reverse order */
    pstar_bus_config_deinit(spi_config);
    pstar_bus_manager_remove_bus(&sd_card->bus_manager, sd_card_default_spi_bus_name);
    pstar_bus_config_destroy(spi_config);
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
    sdspi_host_deinit();
    pstar_bus_config_deinit(spi_config);
    pstar_bus_manager_remove_bus(&sd_card->bus_manager, sd_card_default_spi_bus_name);
    pstar_bus_config_destroy(spi_config);
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
    log_error(sd_card->tag, 
              "SPI Setup Error", 
              "Failed to allocate memory for SD card");
    /* Clean up properly in reverse order */
    sdspi_host_remove_device(sdspi_handle);
    sdspi_host_deinit();
    pstar_bus_config_deinit(spi_config);
    pstar_bus_manager_remove_bus(&sd_card->bus_manager, sd_card_default_spi_bus_name);
    pstar_bus_config_destroy(spi_config);
    return ESP_ERR_NO_MEM;
  }

  /* Copy host configuration to our card structure */
  memcpy(&sd_card->card->host, &host, sizeof(sdmmc_host_t));

  log_info(sd_card->tag, 
           "SPI Setup Complete", 
           "SPI interface configured successfully at %lu Hz", 
           clock_speed_hz);
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
    log_error(sd_card_tag, 
              "SDIO Setup Error", 
              "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(sd_card->tag, 
           "SDIO Setup", 
           "Setting up SDIO interface for SD card with %s bus mode", 
           priv_sd_bus_width_to_string(sd_card->bus_width));
           
  /* Check for pin conflicts when using 4-bit mode */
  bool           pin_conflict       = false;
  sd_bus_width_t original_bus_width = sd_card->bus_width;
  
  if (sd_card->bus_width == k_sd_bus_width_4bit) {
    /* Check if card detect pin conflicts with SDIO D3 */
    if (sd_card->pin_config.gpio_det_pin == sd_card->pin_config.sdio_d3_pin) {
      log_warn(sd_card->tag, 
              "Pin Conflict", 
              "Card detect pin (GPIO %d) conflicts with SDIO D3 for 4-bit mode. "
              "Falling back to 1-bit mode", 
              sd_card->pin_config.gpio_det_pin);
      
      /* Automatically change to 1-bit mode when conflict is detected */
      sd_card->bus_width = k_sd_bus_width_1bit;
      pin_conflict = true;
    }
    
    /* Also check for other potential pin conflicts or unavailable pins */
    if (sd_card->pin_config.sdio_d1_pin < 0 || 
        sd_card->pin_config.sdio_d2_pin < 0 || 
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
  
  /* Configure GPIO pins for SDIO */
  gpio_config_t io_conf = {0};
  
  if (sd_card->bus_width == k_sd_bus_width_4bit && !pin_conflict) {
    /* Set 4-bit mode flag */
    sdmmc_host.flags |= SDMMC_HOST_FLAG_4BIT;
    
    /* Configure GPIO pins for 4-bit mode (CMD, CLK, D0-D3) */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode      = GPIO_MODE_INPUT_OUTPUT_OD; /* Open drain for SDIO pins */
    
    /* Create bit mask for all SDIO pins */
    io_conf.pin_bit_mask = (1ULL << sd_card->pin_config.sdio_cmd_pin) | 
                           (1ULL << sd_card->pin_config.sdio_clk_pin) | 
                           (1ULL << sd_card->pin_config.sdio_d0_pin)  | 
                           (1ULL << sd_card->pin_config.sdio_d1_pin)  | 
                           (1ULL << sd_card->pin_config.sdio_d2_pin)  | 
                           (1ULL << sd_card->pin_config.sdio_d3_pin);
    
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    
    /* Configure all GPIO pins for SDIO */
    esp_err_t gpio_err = gpio_config(&io_conf);
    if (gpio_err != ESP_OK) {
      log_error(sd_card->tag, 
                "SDIO Setup Error", 
                "Failed to configure GPIO pins for 4-bit SDIO: %s", 
                esp_err_to_name(gpio_err));
      
      /* If we failed to configure 4-bit mode, try falling back to 1-bit mode */
      if (original_bus_width == k_sd_bus_width_4bit) {
        log_warn(sd_card->tag,
                "SDIO Setup Fallback",
                "Falling back to 1-bit mode after GPIO configuration failed");
        sd_card->bus_width = k_sd_bus_width_1bit;
        pin_conflict = true;
      } else {
        return gpio_err;
      }
    } else {
      log_info(sd_card->tag, 
               "SDIO Pin Config", 
               "Configured pins for 4-bit mode: CMD=%d, CLK=%d, D0=%d, D1=%d, D2=%d, D3=%d", 
               sd_card->pin_config.sdio_cmd_pin, 
               sd_card->pin_config.sdio_clk_pin, 
               sd_card->pin_config.sdio_d0_pin, 
               sd_card->pin_config.sdio_d1_pin, 
               sd_card->pin_config.sdio_d2_pin, 
               sd_card->pin_config.sdio_d3_pin);
    }
  }
  
  /* If 1-bit mode is selected or we fell back to it */
  if (sd_card->bus_width == k_sd_bus_width_1bit) {
    /* 1-bit mode */
    sdmmc_host.flags &= ~SDMMC_HOST_FLAG_4BIT; /* Clear 4-bit flag */
    
    /* Configure GPIO pins for 1-bit mode (CMD, CLK, D0 only) */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode      = GPIO_MODE_INPUT_OUTPUT_OD;  /* Open drain for SDIO pins */
    
    /* Create bit mask for 1-bit SDIO pins */
    io_conf.pin_bit_mask = (1ULL << sd_card->pin_config.sdio_cmd_pin) | 
                           (1ULL << sd_card->pin_config.sdio_clk_pin) | 
                           (1ULL << sd_card->pin_config.sdio_d0_pin);
    
    io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    
    /* Configure GPIO pins for SDIO */
    esp_err_t gpio_err = gpio_config(&io_conf);
    if (gpio_err != ESP_OK) {
      log_error(sd_card->tag, 
                "SDIO Setup Error", 
                "Failed to configure GPIO pins for 1-bit SDIO: %s", 
                esp_err_to_name(gpio_err));
      return gpio_err;
    }
    
    log_info(sd_card->tag, 
             "SDIO Pin Config", 
             "Configured pins for 1-bit mode: CMD=%d, CLK=%d, D0=%d", 
             sd_card->pin_config.sdio_cmd_pin, 
             sd_card->pin_config.sdio_clk_pin, 
             sd_card->pin_config.sdio_d0_pin);
  }
  
  /* Configure the SDMMC host */
  sdmmc_host.max_freq_khz = k_sd_interface_sdio / 1000; /* Convert to kHz */
  
  /* Set up standard SDMMC configuration for ESP32 */
  sdmmc_host.command_timeout_ms = 1000;
  sdmmc_host.io_voltage         = 3.3f;
  
  /* Check if the bus already exists */
  pstar_bus_config_t* existing_sdio = pstar_bus_manager_find_bus(&sd_card->bus_manager, 
                                                                 sd_card_default_sdio_bus_name);
  if (existing_sdio != NULL) {
    /* Clean up the existing configuration first */
    if (existing_sdio->initialized) {
      esp_err_t err = pstar_bus_config_deinit(existing_sdio);
      if (err != ESP_OK) {
        log_warn(sd_card->tag,
                 "SDIO Cleanup Warning",
                 "Failed to deinitialize existing SDIO bus: %s",
                 esp_err_to_name(err));
      }
    }
    esp_err_t err = pstar_bus_manager_remove_bus(&sd_card->bus_manager, 
                                                sd_card_default_sdio_bus_name);
    if (err != ESP_OK) {
      log_warn(sd_card->tag,
               "SDIO Cleanup Warning",
               "Failed to remove existing SDIO bus from manager: %s",
               esp_err_to_name(err));
    }
  }
  
  /* Create SDIO bus configuration */
  uint32_t host_flags = sdmmc_host.flags;
  
  pstar_bus_config_t* sdio_config = pstar_bus_config_create_sdio(sd_card_default_sdio_bus_name, 
                                                                 host_flags,
                                                                 SDMMC_HOST_SLOT_1, 
                                                                 k_pstar_mode_polling);
  if (sdio_config == NULL) {
    log_error(sd_card->tag, 
              "SDIO Setup Error", 
              "Failed to create SDIO bus configuration");
    return ESP_ERR_NO_MEM;
  }

  /* Configure SDIO host */
  sdio_config->config.sdio.host = sdmmc_host;
  
  /* Create slot configuration using the macro */
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  
  /* Customize the slot configuration */
  slot_config.gpio_cd = sd_card->pin_config.gpio_det_pin;
  slot_config.gpio_wp = -1; /* Write protect not used */
  
  /* Set the bus width */
  slot_config.width = sd_card->bus_width;
  
  /* Configure D3 as the card detect pin if 1-bit mode (only if not same as the custom card detect) */
  if (sd_card->bus_width == k_sd_bus_width_1bit && 
      sd_card->pin_config.sdio_d3_pin != sd_card->pin_config.gpio_det_pin) {
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
  }
  
  /* Assign the configured slot_config to the SDIO configuration */
  sdio_config->config.sdio.slot_config = slot_config;

  /* Add SDIO bus to the manager */
  esp_err_t err = pstar_bus_manager_add_bus(&sd_card->bus_manager, sdio_config);
  if (err != ESP_OK) {
    log_error(sd_card->tag, 
              "SDIO Setup Error", 
              "Failed to add SDIO bus to manager: %s", 
              esp_err_to_name(err));
    pstar_bus_config_destroy(sdio_config);
    return err;
  }

  /* Initialize the SDIO bus */
  err = pstar_bus_config_init(sdio_config);
  if (err != ESP_OK) {
    log_error(sd_card->tag, 
              "SDIO Setup Error", 
              "Failed to initialize SDIO bus: %s", 
              esp_err_to_name(err));
    pstar_bus_manager_remove_bus(&sd_card->bus_manager, sd_card_default_sdio_bus_name);
    pstar_bus_config_destroy(sdio_config);
    return err;
  }

  /* Initialize default SDIO operations */
  pstar_bus_sdio_init_default_ops(&sdio_config->config.sdio.ops);

  /* Free old card structure if it exists */
  if (sd_card->card != NULL) {
    free(sd_card->card);
    sd_card->card = NULL;
  }

  /* Allocate memory for the card structure */
  sd_card->card = malloc(sizeof(sdmmc_card_t));
  if (sd_card->card == NULL) {
    log_error(sd_card->tag, 
              "SDIO Setup Error", 
              "Failed to allocate memory for SD card");
    pstar_bus_config_deinit(sdio_config);
    pstar_bus_manager_remove_bus(&sd_card->bus_manager, sd_card_default_sdio_bus_name);
    pstar_bus_config_destroy(sdio_config);
    return ESP_ERR_NO_MEM;
  }

  /* Copy host configuration to our card structure */
  memcpy(&sd_card->card->host, &sdio_config->config.sdio.host, sizeof(sdmmc_host_t));

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

/**
 * @brief Sets up card detection mechanism using GPIO
 * 
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
static esp_err_t priv_sd_card_setup_detection(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    log_error(sd_card_tag, 
              "Detection Setup Error", 
              "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(sd_card->tag, 
           "Detection Setup", 
           "Setting up card detection mechanism");

  /* Check if the card detect pin conflicts with SDIO D3 pin when using 4-bit mode */
  if (sd_card->bus_width == k_sd_bus_width_4bit && 
      sd_card->pin_config.gpio_det_pin == sd_card->pin_config.sdio_d3_pin) {
    log_warn(sd_card->tag, 
             "Pin Conflict", 
             "Card detect pin (GPIO %d) conflicts with SDIO D3 for 4-bit mode. "
             "Detection may be unreliable and will default to 1-bit mode.", 
             sd_card->pin_config.gpio_det_pin);
    
    /* Note: We'll let the SDIO setup handle the actual mode change,
     * but we log the warning here during detection setup. */
  }

  /* Check if the bus already exists */
  pstar_bus_config_t* existing_gpio = pstar_bus_manager_find_bus(&sd_card->bus_manager, 
                                                                 sd_card_default_gpio_bus_name);
  if (existing_gpio != NULL) {
    /* Clean up the existing configuration first */
    if (existing_gpio->initialized) {
      /* Remove ISR handler if it was registered */
      pstar_bus_gpio_isr_remove(&sd_card->bus_manager, 
                          sd_card_default_gpio_bus_name, 
                          sd_card->pin_config.gpio_det_pin);
      
      esp_err_t err = pstar_bus_config_deinit(existing_gpio);
      if (err != ESP_OK) {
        log_warn(sd_card->tag,
                 "GPIO Cleanup Warning",
                 "Failed to deinitialize existing GPIO bus: %s",
                 esp_err_to_name(err));
      }
    }
    esp_err_t err = pstar_bus_manager_remove_bus(&sd_card->bus_manager, 
                                                 sd_card_default_gpio_bus_name);
    if (err != ESP_OK) {
      log_warn(sd_card->tag,
               "GPIO Cleanup Warning",
               "Failed to remove existing GPIO bus from manager: %s",
               esp_err_to_name(err));
    }
  }

  /* Create GPIO bus configuration */
  pstar_bus_config_t* gpio_config = pstar_bus_config_create_gpio(sd_card_default_gpio_bus_name, 
                                                                 k_pstar_mode_interrupt);
  if (gpio_config == NULL) {
    log_error(sd_card->tag, 
              "Detection Setup Error", 
              "Failed to create GPIO bus configuration");
    return ESP_ERR_NO_MEM;
  }

  /* Configure detection pin */
  gpio_config->config.gpio.config.pin_bit_mask = (1ULL << sd_card->pin_config.gpio_det_pin);
  gpio_config->config.gpio.config.mode         = GPIO_MODE_INPUT;
  gpio_config->config.gpio.config.pull_up_en   = GPIO_PULLUP_ENABLE;
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
    return err;
  }

  /* Initialize the GPIO bus */
  err = pstar_bus_config_init(gpio_config);
  if (err != ESP_OK) {
    log_error(sd_card->tag, 
              "Detection Setup Error", 
              "Failed to initialize GPIO bus: %s", 
              esp_err_to_name(err));
    pstar_bus_manager_remove_bus(&sd_card->bus_manager, sd_card_default_gpio_bus_name);
    pstar_bus_config_destroy(gpio_config);
    return err;
  }

  /* Register ISR handler for the detection pin */
  err = pstar_bus_gpio_isr_add(&sd_card->bus_manager, 
                         sd_card_default_gpio_bus_name, 
                         sd_card->pin_config.gpio_det_pin, 
                         priv_sd_card_detection_isr, 
                         sd_card);
  if (err != ESP_OK) {
    log_error(sd_card->tag, 
              "Detection Setup Error", 
              "Failed to register ISR for card detection pin: %s", 
              esp_err_to_name(err));
    pstar_bus_config_deinit(gpio_config);
    pstar_bus_manager_remove_bus(&sd_card->bus_manager, sd_card_default_gpio_bus_name);
    pstar_bus_config_destroy(gpio_config);
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

/**
 * @brief Try mounting the SD card with all available interfaces in priority order
 * 
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if any interface worked, appropriate error code otherwise
 */
static esp_err_t priv_sd_card_try_interfaces(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    log_error(sd_card_tag, 
              "Interface Error", 
              "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

  sd_card->interface_attempt_count++;
  esp_err_t last_error = ESP_OK;
  
  /* Keep track of loaded hosts/buses to ensure proper cleanup */
  bool sdio_host_loaded = false;
  bool spi_host_loaded = false;
  sdmmc_card_t* temp_card = NULL;

  /* If we have a previously successful interface, try it first */
  if (sd_card->interface_discovery_complete && 
      sd_card->current_interface < k_sd_interface_count) {
    sd_interface_type_t interface_type = sd_card->current_interface;
    
    log_info(sd_card->tag, 
             "Interface", 
             "Using previous successful interface: %s with %s bus width", 
             sd_card_interface_to_string(interface_type),
             priv_sd_bus_width_to_string(sd_card->bus_width));

    /* Try using the known good interface */
    esp_err_t err;
    switch (interface_type) {
      case k_sd_interface_sdio:
        err = priv_sd_card_setup_sdio(sd_card);
        if (err == ESP_OK) {
          sdio_host_loaded = true;
        }
        break;
      case k_sd_interface_fast_spi:
      case k_sd_interface_normal_spi:
      case k_sd_interface_slow_spi:
        err = priv_sd_card_setup_spi_with_speed(sd_card, (uint32_t)interface_type);
        if (err == ESP_OK) {
          spi_host_loaded = true;
        }
        break;
      default:
        log_error(sd_card->tag, 
                  "Interface Error", 
                  "Invalid interface type: %d", 
                  (int)interface_type);
        return ESP_ERR_INVALID_ARG;
    }

    if (err == ESP_OK) {
      err = priv_sd_card_mount(sd_card);
      if (err == ESP_OK) {
        /* Update interface info */
        sd_card->interface_info[interface_type].successful        = true;
        sd_card->interface_info[interface_type].last_success_time = esp_timer_get_time();
        sd_card->interface_info[interface_type].error_count       = 0; /* Reset error count on success */
        return ESP_OK;
      } else {
        /* Failed to mount with known good interface, increment error count */
        sd_card->interface_info[interface_type].error_count++;
        log_warn(sd_card->tag, 
                 "Interface Warning", 
                 "Failed to mount with previous successful interface: %s (error %s)", 
                 sd_card_interface_to_string(interface_type),
                 esp_err_to_name(err));
        
        last_error = err;
        
        /* Need to rediscover interfaces if error count is too high */
        if (sd_card->interface_info[interface_type].error_count > 3) {
          sd_card->interface_discovery_complete = false;
        }
        
        /* Clean up resources before trying next interface */
        if (spi_host_loaded) {
          /* Clean up SPI resources with error checking */
          if (sd_card->card != NULL && sd_card->card->host.flags & SDMMC_HOST_FLAG_SPI) {
            if (sd_card->card->host.slot != 0) {
              esp_err_t remove_err = sdspi_host_remove_device(sd_card->card->host.slot);
              if (remove_err != ESP_OK) {
                log_warn(sd_card->tag,
                         "Interface Cleanup Warning",
                         "Failed to remove SDSPI device: %s",
                         esp_err_to_name(remove_err));
              }
            }
          }
          esp_err_t deinit_err = sdspi_host_deinit();
          if (deinit_err != ESP_OK) {
            log_warn(sd_card->tag,
                     "Interface Cleanup Warning",
                     "Failed to deinitialize SDSPI host: %s",
                     esp_err_to_name(deinit_err));
          }
          spi_host_loaded = false;
        }
        
        if (sdio_host_loaded) {
          /* Clean up SDIO resources */
          esp_err_t sdio_err = sdmmc_host_deinit();
          if (sdio_err != ESP_OK) {
            log_warn(sd_card->tag,
                     "Interface Cleanup Warning",
                     "Failed to deinitialize SDMMC host: %s",
                     esp_err_to_name(sdio_err));
          }
          sdio_host_loaded = false;
        }
        
        /* Free card structure if it was allocated */
        if (sd_card->card != NULL) {
          temp_card = sd_card->card;
          sd_card->card = NULL;
          free(temp_card);
          temp_card = NULL;
        }
      }
    } else {
      /* Failed to setup the interface, increment error count */
      sd_card->interface_info[interface_type].error_count++;
      log_warn(sd_card->tag, 
               "Interface Warning", 
               "Failed to setup previously successful interface: %s (error %s)", 
               sd_card_interface_to_string(interface_type),
               esp_err_to_name(err));
      
      last_error = err;
      
      /* Need to rediscover interfaces if error count is too high */
      if (sd_card->interface_info[interface_type].error_count > 3) {
        sd_card->interface_discovery_complete = false;
      }
    }
  }

  /* Perform full discovery if needed */
  if (!sd_card->interface_discovery_complete) {
    /* Discovery needed - try each interface in order of preference */
    const sd_interface_type_t interface_order[] = {
      k_sd_interface_sdio,
      k_sd_interface_fast_spi,
      k_sd_interface_normal_spi,
      k_sd_interface_slow_spi
    };

    log_info(sd_card->tag, 
             "Interface Discovery", 
             "Starting interface discovery for SD card with %s bus width (attempt %lu)", 
             priv_sd_bus_width_to_string(sd_card->bus_width),
             sd_card->interface_attempt_count);

    /* Keep track of the error for each interface attempt */
    esp_err_t last_errors[k_sd_interface_count] = { ESP_OK };
    
    for (int i = 0; i < k_sd_interface_count; i++) {
      sd_interface_type_t interface_type = interface_order[i];
      
      /* Ensure we've cleaned up any previous interface resources */
      if (spi_host_loaded) {
        /* Clean up SPI resources with error checking */
        if (sd_card->card != NULL && sd_card->card->host.flags & SDMMC_HOST_FLAG_SPI) {
          if (sd_card->card->host.slot != 0) {
            esp_err_t remove_err = sdspi_host_remove_device(sd_card->card->host.slot);
            if (remove_err != ESP_OK) {
              log_warn(sd_card->tag,
                       "Interface Cleanup Warning",
                       "Failed to remove SDSPI device: %s",
                       esp_err_to_name(remove_err));
            }
          }
        }
        esp_err_t deinit_err = sdspi_host_deinit();
        if (deinit_err != ESP_OK) {
          log_warn(sd_card->tag,
                   "Interface Cleanup Warning",
                   "Failed to deinitialize SDSPI host: %s",
                   esp_err_to_name(deinit_err));
        }
        spi_host_loaded = false;
      }
      
      if (sdio_host_loaded) {
        /* Clean up SDIO resources with error checking */
        esp_err_t sdio_err = sdmmc_host_deinit();
        if (sdio_err != ESP_OK) {
          log_warn(sd_card->tag,
                   "Interface Cleanup Warning",
                   "Failed to deinitialize SDMMC host: %s",
                   esp_err_to_name(sdio_err));
        }
        sdio_host_loaded = false;
      }
      
      /* Free card structure if it was allocated */
      if (temp_card != NULL) {
        free(temp_card);
        temp_card = NULL;
      }
      
      /* Mark this interface as attempted */
      sd_card->interface_info[interface_type].attempted = true;
      
      /* Skip if we've had too many errors with this interface */
      if (sd_card->interface_info[interface_type].error_count > 3) {
        log_warn(sd_card->tag, 
                 "Interface Skip", 
                 "Skipping %s due to %lu previous errors", 
                 sd_card_interface_to_string(interface_type),
                 sd_card->interface_info[interface_type].error_count);
        continue;
      }
      
      log_info(sd_card->tag, 
               "Interface Try", 
               "Trying interface: %s", 
               sd_card_interface_to_string(interface_type));
      
      /* Try to set up this interface */
      esp_err_t err;
      switch (interface_type) {
        case k_sd_interface_sdio:
          err = priv_sd_card_setup_sdio(sd_card);
          if (err == ESP_OK) {
            sdio_host_loaded = true;
          }
          break;
        case k_sd_interface_fast_spi:
        case k_sd_interface_normal_spi:
        case k_sd_interface_slow_spi:
          err = priv_sd_card_setup_spi_with_speed(sd_card, (uint32_t)interface_type);
          if (err == ESP_OK) {
            spi_host_loaded = true;
          }
          break;
        default:
          log_warn(sd_card->tag,
                   "Interface Warning", 
                   "Skipping invalid interface type: %d", 
                   (int)interface_type);
          continue; /* Skip invalid interface types */
      }

      if (err != ESP_OK) {
        log_warn(sd_card->tag, 
                 "Interface Failure", 
                 "Failed to setup %s: %s", 
                 sd_card_interface_to_string(interface_type),
                 esp_err_to_name(err));
        
        /* Store the error for this interface */
        last_errors[interface_type] = err;
        last_error = err;
        
        /* Increment error count for this interface */
        sd_card->interface_info[interface_type].error_count++;
        continue;
      }

      /* Try to mount with this interface */
      err = priv_sd_card_mount(sd_card);
      if (err != ESP_OK) {
        log_warn(sd_card->tag, 
                 "Interface Failure", 
                 "Failed to mount with %s: %s", 
                 sd_card_interface_to_string(interface_type),
                 esp_err_to_name(err));
        
        /* Store the error for this interface */
        last_errors[interface_type] = err;
        last_error = err;
        
        /* Increment error count for this interface */
        sd_card->interface_info[interface_type].error_count++;
        continue;
      }

      /* Success! */
      log_info(sd_card->tag, 
               "Interface Success", 
               "Successfully mounted SD card with %s and %s bus width", 
               sd_card_interface_to_string(interface_type),
               priv_sd_bus_width_to_string(sd_card->bus_width));
      
      /* Update interface info */
      sd_card->interface_info[interface_type].successful        = true;
      sd_card->interface_info[interface_type].last_success_time = esp_timer_get_time();
      sd_card->interface_info[interface_type].error_count       = 0; /* Reset error count on success */
      sd_card->current_interface                                = interface_type;
      sd_card->interface_discovery_complete                     = true;
      
      /* Save the working configuration to NVS */
      priv_save_working_config(sd_card);
      
      /* Clear any temporary resources */
      temp_card = NULL;
      
      return ESP_OK;
    }

    /* Clean up resources if all interfaces failed */
    if (spi_host_loaded) {
      /* Clean up SPI resources with error checking */
      if (sd_card->card != NULL && sd_card->card->host.flags & SDMMC_HOST_FLAG_SPI) {
        if (sd_card->card->host.slot != 0) {
          esp_err_t remove_err = sdspi_host_remove_device(sd_card->card->host.slot);
          if (remove_err != ESP_OK) {
            log_warn(sd_card->tag,
                     "Interface Cleanup Warning",
                     "Failed to remove SDSPI device: %s",
                     esp_err_to_name(remove_err));
          }
        }
      }
      esp_err_t deinit_err = sdspi_host_deinit();
      if (deinit_err != ESP_OK) {
        log_warn(sd_card->tag,
                 "Interface Cleanup Warning",
                 "Failed to deinitialize SDSPI host: %s",
                 esp_err_to_name(deinit_err));
      }
      spi_host_loaded = false;
    }
    
    if (sdio_host_loaded) {
      /* Clean up SDIO resources with error checking */
      esp_err_t sdio_err = sdmmc_host_deinit();
      if (sdio_err != ESP_OK) {
        log_warn(sd_card->tag,
                 "Interface Cleanup Warning",
                 "Failed to deinitialize SDMMC host: %s",
                 esp_err_to_name(sdio_err));
      }
      sdio_host_loaded = false;
    }
    
    /* Free card structure if it was temporarily allocated */
    if (temp_card != NULL) {
      free(temp_card);
      temp_card = NULL;
    }
    
    /* If we get here, all interfaces failed */
    /* Find the least-failing interface to return its error code */
    sd_interface_type_t least_error_interface = k_sd_interface_count;
    uint32_t            min_error_count       = UINT32_MAX;
    
    for (int i = 0; i < k_sd_interface_count; i++) {
      if (sd_card->interface_info[i].attempted && 
          sd_card->interface_info[i].error_count < min_error_count) {
        min_error_count       = sd_card->interface_info[i].error_count;
        least_error_interface = (sd_interface_type_t)i;
      }
    }
    
    if (least_error_interface < k_sd_interface_count) {
      log_error(sd_card->tag, 
                "Interface Error", 
                "All interfaces failed during discovery. Best interface was %s with %lu errors", 
                sd_card_interface_to_string(least_error_interface),
                min_error_count);
      return last_errors[least_error_interface];
    }
    
    log_error(sd_card->tag, 
              "Interface Error", 
              "All interfaces failed during discovery and none were attempted");
    return ESP_ERR_NOT_FOUND;
  }

  /* If we get here, previous interface failed and rediscovery also failed */
  log_error(sd_card->tag,
            "Interface Error",
            "Failed to find a working interface after %lu attempts",
            sd_card->interface_attempt_count);
  return last_error != ESP_OK ? last_error : ESP_ERR_NOT_FOUND;
}

/**
 * @brief Mounts the SD card
 * 
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
static esp_err_t priv_sd_card_mount(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    log_error(sd_card_tag, 
              "Mount Error", 
              "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if already mounted */
  if (sd_card->card_available) {
    log_info(sd_card->tag, 
             "Mount Info", 
             "SD card already mounted");
    return ESP_OK;
  }

  /* Check if physically inserted */
  if (!priv_sd_card_is_inserted(sd_card)) {
    log_warn(sd_card->tag, 
             "Mount Warning", 
             "Cannot mount: SD card not inserted");
    return ESP_ERR_NOT_FOUND;
  }

  log_info(sd_card->tag, 
           "Mount Started", 
           "Mounting SD card at path: %s with %s bus width", 
           sd_card->mount_path,
           priv_sd_bus_width_to_string(sd_card->bus_width));

  /* Validate mount path using the simpler validation function */
  if (!priv_path_is_safe_simple(sd_card->mount_path)) {
    log_error(sd_card->tag, 
              "Mount Error", 
              "Unsafe mount path: %s", 
              sd_card->mount_path);
    return ESP_ERR_INVALID_ARG;
  }

  /* Ensure mount path directory exists */
  esp_err_t dir_result = priv_create_directory_if_needed(sd_card, sd_card->mount_path);
  if (dir_result != ESP_OK) {
    log_error(sd_card_tag, 
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

  /* Get slot config based on host flags */
  sdmmc_slot_config_t default_slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  if (!(sd_card->card->host.flags & SDMMC_HOST_FLAG_SPI)) {
    /* SDIO mode - configure the slot */
    default_slot_config.gpio_cd = sd_card->pin_config.gpio_det_pin;
    default_slot_config.gpio_wp = -1; /* Write protect not used */
    
    /* Set the bus width */
    default_slot_config.width = sd_card->bus_width;
    
    /* Add flags for 4-bit mode */
    if (sd_card->bus_width == k_sd_bus_width_4bit) {
      default_slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    }
  }

  /* Try to mount the SD card */
  sdmmc_card_t* card = NULL;
  esp_err_t     err;
  
  /* For SPI mode, slot_config should be NULL */
  if (sd_card->card->host.flags & SDMMC_HOST_FLAG_SPI) {
    err = esp_vfs_fat_sdmmc_mount(sd_card->mount_path, 
                                  &sd_card->card->host, 
                                  NULL, /* NULL slot config for SPI mode */
                                  &mount_config, 
                                  &card);
  } else {
    err = esp_vfs_fat_sdmmc_mount(sd_card->mount_path, 
                                  &sd_card->card->host, 
                                  &default_slot_config,
                                  &mount_config, 
                                  &card);
  }
                                  
  /* Handle potential bus width issues */
  if (err != ESP_OK && sd_card->bus_width == k_sd_bus_width_4bit && 
      !(sd_card->card->host.flags & SDMMC_HOST_FLAG_SPI)) {
    /* If 4-bit mode failed, try falling back to 1-bit mode */
    log_warn(sd_card->tag, 
             "Mount Warning", 
             "Failed to mount with 4-bit bus width, trying 1-bit mode");
    
    /* Create a new slot config with 1-bit mode */
    sdmmc_slot_config_t one_bit_slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    one_bit_slot_config.gpio_cd = sd_card->pin_config.gpio_det_pin;
    one_bit_slot_config.gpio_wp = -1; /* Write protect not used */
    one_bit_slot_config.width = k_sd_bus_width_1bit;
    
    /* Clear 4-bit flag from host */
    sd_card->card->host.flags &= ~SDMMC_HOST_FLAG_4BIT;
    
    /* Try mounting again with 1-bit mode */
    err = esp_vfs_fat_sdmmc_mount(sd_card->mount_path, 
                                  &sd_card->card->host, 
                                  &one_bit_slot_config,
                                  &mount_config, 
                                  &card);
                                  
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
      log_error(sd_card->tag, 
                "Mount Error", 
                "Failed to mount FAT filesystem on SD card");
    } else if (err == ESP_ERR_INVALID_STATE) {
      log_error(sd_card->tag, 
                "Mount Error", 
                "SD card already mounted");
    } else if (err == ESP_ERR_NO_MEM) {
      log_error(sd_card->tag, 
                "Mount Error", 
                "Memory allocation failed");
    } else {
      log_error(sd_card->tag, 
                "Mount Error", 
                "Failed to initialize SD card: %s", 
                esp_err_to_name(err));
    }
    return err;
  }

  /* Save the card structure returned from the mount function */
  if (sd_card->card != NULL) {
    free(sd_card->card);
  }
  sd_card->card = card;

  /* Create logs directory if not exists */
  char logs_path[sd_card_max_path_len];
  if (snprintf(logs_path, sizeof(logs_path), "%s/logs", sd_card->mount_path) >= 
      (int)sizeof(logs_path)) {
    log_warn(sd_card->tag, 
             "Mount Warning", 
             "Log path truncated");
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

  /* Get and log SD card info */
  sdmmc_card_print_info(stdout, sd_card->card);
  
  log_info(sd_card->tag, 
           "Mount Success", 
           "SD card mounted at %s, %lluMB, %s bus width", 
           sd_card->mount_path,
           ((uint64_t)sd_card->card->csd.capacity * sd_card->card->csd.sector_size) / (1024 * 1024),
           priv_sd_bus_width_to_string(sd_card->bus_width));

  /* Update state */
  sd_card->card_available = true;
  
  /* Schedule performance measurement on the new card */
  sd_card->performance.measurement_needed = true;

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
    log_error(sd_card_tag, 
              "Unmount Error", 
              "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if already unmounted */
  if (!sd_card->card_available) {
    log_info(sd_card->tag, 
             "Unmount Info", 
             "SD card already unmounted");
    return ESP_OK;
  }

  log_info(sd_card->tag, 
           "Unmount Started", 
           "Unmounting SD card from path: %s", 
           sd_card->mount_path);

  /* Unmount the SD card */
  esp_err_t err = esp_vfs_fat_sdcard_unmount(sd_card->mount_path, sd_card->card);
  if (err != ESP_OK) {
    log_error(sd_card->tag, 
              "Unmount Error", 
              "Failed to unmount SD card: %s", 
              esp_err_to_name(err));
    return err;
  }

  /* Card pointer is now invalid */
  sd_card->card = NULL;

  /* Update state */
  sd_card->card_available = false;

  log_info(sd_card->tag, 
           "Unmount Success", 
           "SD card unmounted successfully");
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
    log_error(sd_card_tag, 
              "Mount Task Error", 
              "Invalid task parameter");
    vTaskDelete(NULL);
    return;
  }

  log_info(sd_card->tag, 
           "Mount Task Started", 
           "SD card mount/unmount task is running");

  bool       was_inserted     = false;
  uint32_t   debounce_counter = 0;
  TickType_t last_change_time = xTaskGetTickCount();
  bool       mutex_taken      = false;
  
  /* Initialize state machine */
  sd_card->state                  = k_sd_state_idle;
  sd_card->last_state_change_time = esp_timer_get_time();
  
  /* Initialize exit request flag */
  sd_card->mount_task_exit_requested = false;

  /* Load previous working configuration from NVS if available */
  priv_load_working_config(sd_card);

  /* Initial card check */
  bool is_inserted = priv_sd_card_is_inserted(sd_card);
  if (is_inserted) {
    log_info(sd_card->tag, 
             "Initial Detection", 
             "SD card detected at startup");
    
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
      } else if (sd_card->card_available) {
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
    
    was_inserted = sd_card->card_available;
  } else {
    log_info(sd_card->tag, 
             "Initial Detection", 
             "No SD card detected at startup");
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
    is_inserted = priv_sd_card_is_inserted(sd_card);
    
    /* Debounce the detection */
    TickType_t current_time = xTaskGetTickCount();
    if (is_inserted != was_inserted) {
      if (debounce_counter == 0) {
        /* First detection of change */
        last_change_time = current_time;
        debounce_counter++;
      } else if ((current_time - last_change_time) >= 
                 pdMS_TO_TICKS(sd_card_default_det_debounce_time)) {
        /* Debounce period expired, consider it a stable change */
        log_info(sd_card->tag, 
                 "Card Status Change", 
                 "SD card %s", 
                 is_inserted ? "inserted" : "removed");
        
        /* Take mutex for thread safety */
        mutex_taken = false;
        if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
          mutex_taken = true;
          if (is_inserted && !sd_card->card_available) {
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
                log_info(sd_card->tag, 
                         "Retry", 
                         "Will retry mounting after delay");
                /* The error handler will call the reset function */
              } else {
                /* Maximum retries reached */
                priv_update_state_machine(sd_card, k_sd_state_failed);
              }
            } else if (sd_card->card_available) {
              /* Mount successful - update state machine */
              priv_update_state_machine(sd_card, k_sd_state_interface_ready);
              /* Reset error handler */
              error_handler_reset_state(&sd_card->error_handler);
            }
          } else if (!is_inserted && sd_card->card_available) {
            /* Card removed - unmount first */
            esp_err_t err = priv_sd_card_unmount(sd_card);
            if (err != ESP_OK) {
              log_error(sd_card->tag, 
                        "Unmount Error", 
                        "Failed to unmount SD card: %s", 
                        esp_err_to_name(err));
                        
              /* Even if unmount fails, we need to update the state machine
               * to reflect that the card is physically removed */
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
        was_inserted     = is_inserted;
        debounce_counter = 0;
      } else {
        /* Still within debounce period, increment counter */
        debounce_counter++;
      }
    } else {
      /* No change in state, reset debounce counter */
      debounce_counter = 0;
    }
    
    /* Check exit flag before potentially long operation */
    if (sd_card->mount_task_exit_requested) {
      break;
    }
    
    /* Handle card in error state - check for retries */
    if (sd_card->state == k_sd_state_error && is_inserted &&
        error_handler_can_retry(&sd_card->error_handler)) {
      /* Take mutex for thread safety */
      mutex_taken = false;
      if (xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
        mutex_taken = true;
        
        log_info(sd_card->tag,
                "Error Recovery",
                "Attempting to recover from error (retry %lu/%lu)",
                sd_card->error_handler.current_retry + 1,
                sd_card->error_handler.max_retries);
                
        /* Try to reset and mount */
        esp_err_t err = priv_sd_card_reset(sd_card);
        if (err != ESP_OK) {
          log_error(sd_card->tag, 
                    "Reset Error", 
                    "Failed to reset SD card: %s", 
                    esp_err_to_name(err));
          
          /* Record the error */
          RECORD_ERROR(&sd_card->error_handler, err, "Failed to reset SD card");
          
          /* Check if we've reached maximum retries */
          if (!error_handler_can_retry(&sd_card->error_handler)) {
            /* Update state machine to reflect permanent failure */
            priv_update_state_machine(sd_card, k_sd_state_failed);
          }
        } else if (sd_card->card_available) {
          /* Reset successful, we're mounted again */
          priv_update_state_machine(sd_card, k_sd_state_interface_ready);
          error_handler_reset_state(&sd_card->error_handler);
        }
        
        /* Always release mutex when done */
        priv_release_mutex_if_taken(sd_card, &mutex_taken);
      } else {
        log_error(sd_card->tag,
                  "Error Retry Error",
                  "Failed to acquire mutex for error retry");
      }
    }
    
    /* Perform performance measurements if needed and card is ready */
    if (sd_card->state == k_sd_state_interface_ready && 
        sd_card->card_available && 
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
    if (sd_card->state == k_sd_state_interface_ready && sd_card->card_available) {
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
                      
              /* Reset the card */
              esp_err_t err = priv_sd_card_reset(sd_card);
              if (err != ESP_OK) {
                log_error(sd_card->tag,
                          "Health Reset Error",
                          "Failed to reset SD card during health check: %s",
                          esp_err_to_name(err));
                          
                /* Record the error */
                RECORD_ERROR(&sd_card->error_handler, err, "Failed to reset SD card during health check");
                
                /* Update state */
                priv_update_state_machine(sd_card, k_sd_state_error);
              }
            }
          }
          
          /* Release mutex */
          priv_release_mutex_if_taken(sd_card, &mutex_taken);
        }
      }
    }
  }

  log_info(sd_card->tag, 
           "Mount Task Exiting", 
           "SD card mount/unmount task received exit request");
  
  vTaskDelete(NULL);
}