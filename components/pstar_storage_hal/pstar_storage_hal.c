/* components/pstar_storage_hal/pstar_storage_hal.c */

#include "pstar_storage_hal.h"

#include "pstar_bus_gpio.h"    // Include Bus GPIO header
#include "pstar_bus_manager.h" // Include Bus Manager header
#include "pstar_log_handler.h"
#include "pstar_pin_validator.h"
#include "pstar_storage_common.h"
#include "pstar_storage_spi_hal.h" // Include SPI HAL header

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_err.h"
#include "nvs_flash.h"

/* Constants ******************************************************************/
static const char* TAG = "Storage HAL";

/* Default pin configurations (moved from pstar_storage_sd_card_hal.c) */
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

  /* SDIO pins - keeping for future but not using now */
  .sdio_clk_pin = -1,
  .sdio_cmd_pin = -1,
  .sdio_d0_pin  = -1,
  .sdio_d1_pin  = -1,
  .sdio_d2_pin  = -1,
  .sdio_d3_pin  = -1,
};

/* Global instance for simplified API ******************************************/
static sd_card_hal_t g_sd_card;
static bool          g_sd_card_initialized = false;

/* Simplified API Functions ***************************************************/

esp_err_t sd_card_init_simple(void)
{
  if (g_sd_card_initialized) {
    return ESP_OK;
  }

  esp_err_t err = sd_card_init_default(&g_sd_card,
                                       "SD_Card",
                                       CONFIG_PSTAR_KCONFIG_SD_CARD_MOUNT_POINT,
                                       "storage_hal",
                                       k_sd_bus_width_1bit);
  if (err != ESP_OK) {
    log_error(TAG, "Init Simple Error", "Failed to initialize SD card: %s", esp_err_to_name(err));
    return err;
  }

  err = sd_card_init(&g_sd_card);
  if (err != ESP_OK) {
    log_error(TAG, "Init Simple Error", "Failed to start SD card: %s", esp_err_to_name(err));
    return err;
  }

  g_sd_card_initialized = true;
  log_info(TAG, "Init Simple", "SD card initialized successfully");
  return ESP_OK;
}

esp_err_t sd_card_write(const char* filename, const void* data, size_t len, bool append)
{
  if (!g_sd_card_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!sd_card_is_available(&g_sd_card)) {
    return ESP_ERR_NOT_FOUND;
  }

  if (filename == NULL || data == NULL || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  // Construct full path
  char full_path[CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH];
  if (filename[0] == '/') {
    if (snprintf(full_path, sizeof(full_path), "%s%s", g_sd_card.mount_path, filename) >=
        (int)sizeof(full_path)) {
      return ESP_ERR_INVALID_ARG;
    }
  } else {
    if (snprintf(full_path, sizeof(full_path), "%s/%s", g_sd_card.mount_path, filename) >=
        (int)sizeof(full_path)) {
      return ESP_ERR_INVALID_ARG;
    }
  }

  // Check if path is safe
  if (!storage_path_is_safe(&g_sd_card, full_path)) {
    return ESP_ERR_INVALID_ARG;
  }

  // Create parent directory if needed
  char dir_path[CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH];
  strncpy(dir_path, full_path, sizeof(dir_path));

  // Find last slash to get directory path
  char* last_slash = strrchr(dir_path, '/');
  if (last_slash != NULL) {
    *last_slash = '\0'; // Terminate string at last slash

    // Create directory if it doesn't exist
    esp_err_t err = storage_create_directory_if_needed(&g_sd_card, dir_path);
    if (err != ESP_OK) {
      log_error(TAG, "Write Error", "Failed to create directory: %s", esp_err_to_name(err));
      return err;
    }
  }

  // Open file
  FILE* f = fopen(full_path, append ? "ab" : "wb");
  if (f == NULL) {
    log_error(TAG, "Write Error", "Failed to open file: %s (%s)", full_path, strerror(errno));
    return ESP_ERR_NOT_FOUND;
  }

  // Write data
  size_t written = fwrite(data, 1, len, f);
  fclose(f);

  if (written != len) {
    log_error(TAG, "Write Error", "Failed to write all data (%zu/%zu bytes written)", written, len);
    return ESP_ERR_INVALID_STATE;
  }

  return ESP_OK;
}

esp_err_t sd_card_read(const char* filename, void* data, size_t max_len, size_t* bytes_read)
{
  if (!g_sd_card_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!sd_card_is_available(&g_sd_card)) {
    return ESP_ERR_NOT_FOUND;
  }

  if (filename == NULL || data == NULL || max_len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  // Construct full path
  char full_path[CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH];
  if (filename[0] == '/') {
    if (snprintf(full_path, sizeof(full_path), "%s%s", g_sd_card.mount_path, filename) >=
        (int)sizeof(full_path)) {
      return ESP_ERR_INVALID_ARG;
    }
  } else {
    if (snprintf(full_path, sizeof(full_path), "%s/%s", g_sd_card.mount_path, filename) >=
        (int)sizeof(full_path)) {
      return ESP_ERR_INVALID_ARG;
    }
  }

  // Check if path is safe
  if (!storage_path_is_safe(&g_sd_card, full_path)) {
    return ESP_ERR_INVALID_ARG;
  }

  // Open file
  FILE* f = fopen(full_path, "rb");
  if (f == NULL) {
    log_error(TAG, "Read Error", "Failed to open file: %s (%s)", full_path, strerror(errno));
    return ESP_ERR_NOT_FOUND;
  }

  // Read data
  size_t read_count = fread(data, 1, max_len, f);
  fclose(f);

  // Update bytes read if pointer provided
  if (bytes_read != NULL) {
    *bytes_read = read_count;
  }

  return ESP_OK;
}

bool sd_card_file_exists(const char* filename)
{
  if (!g_sd_card_initialized || !sd_card_is_available(&g_sd_card)) {
    return false;
  }

  if (filename == NULL) {
    return false;
  }

  // Construct full path
  char full_path[CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH];
  if (filename[0] == '/') {
    if (snprintf(full_path, sizeof(full_path), "%s%s", g_sd_card.mount_path, filename) >=
        (int)sizeof(full_path)) {
      return false;
    }
  } else {
    if (snprintf(full_path, sizeof(full_path), "%s/%s", g_sd_card.mount_path, filename) >=
        (int)sizeof(full_path)) {
      return false;
    }
  }

  // Check if path is safe
  if (!storage_path_is_safe(&g_sd_card, full_path)) {
    return false;
  }

  // Check if file exists
  struct stat st;
  return (stat(full_path, &st) == 0);
}

esp_err_t sd_card_delete_file(const char* filename)
{
  if (!g_sd_card_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!sd_card_is_available(&g_sd_card)) {
    return ESP_ERR_NOT_FOUND;
  }

  if (filename == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Construct full path
  char full_path[CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH];
  if (filename[0] == '/') {
    if (snprintf(full_path, sizeof(full_path), "%s%s", g_sd_card.mount_path, filename) >=
        (int)sizeof(full_path)) {
      return ESP_ERR_INVALID_ARG;
    }
  } else {
    if (snprintf(full_path, sizeof(full_path), "%s/%s", g_sd_card.mount_path, filename) >=
        (int)sizeof(full_path)) {
      return ESP_ERR_INVALID_ARG;
    }
  }

  // Check if path is safe
  if (!storage_path_is_safe(&g_sd_card, full_path)) {
    return ESP_ERR_INVALID_ARG;
  }

  // Delete file
  if (unlink(full_path) != 0) {
    log_error(TAG, "Delete Error", "Failed to delete file: %s (%s)", full_path, strerror(errno));
    return ESP_ERR_NOT_FOUND;
  }

  return ESP_OK;
}

esp_err_t sd_card_create_dir(const char* path)
{
  if (!g_sd_card_initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!sd_card_is_available(&g_sd_card)) {
    return ESP_ERR_NOT_FOUND;
  }

  if (path == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Construct full path
  char full_path[CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH];
  if (path[0] == '/') {
    if (snprintf(full_path, sizeof(full_path), "%s%s", g_sd_card.mount_path, path) >=
        (int)sizeof(full_path)) {
      return ESP_ERR_INVALID_ARG;
    }
  } else {
    if (snprintf(full_path, sizeof(full_path), "%s/%s", g_sd_card.mount_path, path) >=
        (int)sizeof(full_path)) {
      return ESP_ERR_INVALID_ARG;
    }
  }

  // Create directory
  return storage_create_directory_if_needed(&g_sd_card, full_path);
}

const char* sd_card_get_mount_path(void)
{
  if (!g_sd_card_initialized) {
    return NULL;
  }

  return g_sd_card.mount_path;
}

bool sd_card_is_ready(void)
{
  if (!g_sd_card_initialized) {
    return false;
  }

  return sd_card_is_available(&g_sd_card);
}

/* Public Functions ***********************************************************/

const char* sd_card_interface_to_string(sd_interface_type_t interface_type)
{
  switch (interface_type) {
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
      pin_config == NULL) {
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

  /* Defer detailed pin validation to registration */

  log_info(sd_card->tag, "Custom Pins", "SD card initialized with custom pin configuration");

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
  if (sd_card == NULL || tag == NULL || mount_path == NULL || component_id == NULL) {
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
    log_error(TAG, "Invalid bus width", "Bus width must be 1 or 4, got: %d", bus_width);
    return ESP_ERR_INVALID_ARG;
  }

  /* Validate mount path using the simpler path check */
  if (!storage_path_is_safe_simple(mount_path)) {
    log_error(TAG, "Invalid mount path", "Mount path '%s' is not safe", mount_path);
    return ESP_ERR_INVALID_ARG;
  }

  /* For SPI-only mode, always use SPI as preferred interface */
  sd_interface_type_t preferred_interface = k_sd_interface_spi;

  /* Cache enabled modes - we only support SPI now */
  bool spi_enabled = false;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  // Call function only ifdef'd
  spi_enabled = storage_spi_is_supported();
#endif

  /* Check if SPI is enabled */
  if (!spi_enabled) {
    log_error(TAG, "No interfaces enabled", "SPI mode must be enabled");
    return ESP_ERR_NOT_SUPPORTED;
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
  sd_card_hal_t sd_card_default = {.tag        = tag,
                                   .mount_path = mount_path,
                                   .max_files  = CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_FILES,
                                   .allocation_unit_size =
                                     CONFIG_PSTAR_KCONFIG_SD_CARD_ALLOCATION_UNIT_SIZE,
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
                                   .bus_width              = bus_width,
                                   .pin_config             = sd_card_default_pins,
                                   .preferred_interface    = preferred_interface,
                                   .enable_fallback        = false, // No fallback in SPI-only mode
                                   .sdio_mode_enabled      = false, // SDIO disabled for now
                                   .spi_mode_enabled       = spi_enabled,
                                   .state                  = k_sd_state_idle,
                                   .error_count            = 0,
                                   .last_state_change_time = 0,
                                   .current_interface      = k_sd_interface_none,
                                   .interface_info         = {{0}, {0}},
                                   .interface_discovery_complete = false,
                                   .interface_attempt_count      = 0,
                                   .card                         = NULL,
                                   .mutex                        = NULL,
                                   .initialized                  = false,
                                   .mount_task_exit_requested    = false,
                                   .mount_task_handle            = NULL,
                                   .error_handler                = {0},
                                   .component_id                 = component_id,
                                   .bus_manager                  = {0},
                                   .task_config                  = task_config,
                                   .performance                  = performance,
                                   .availability_callback        = NULL};
  atomic_init(&sd_card_default.card_available, false);

  /* Copy default configuration to sd_card */
  memcpy(sd_card, &sd_card_default, sizeof(sd_card_hal_t));

  /* Initialize error handler */
  err = error_handler_init(&sd_card->error_handler,
                           CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_RETRY,
                           CONFIG_PSTAR_KCONFIG_SD_CARD_RETRY_DELAY_MS,
                           CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_RETRY_DELAY_MS,
                           storage_sd_card_reset,
                           sd_card);
  if (err != ESP_OK) {
    log_error(TAG, "Failed to init error handler: %s", esp_err_to_name(err));
    return err;
  }

  /* Initialize mutex */
  sd_card->mutex = xSemaphoreCreateMutex();
  if (sd_card->mutex == NULL) {
    log_error(TAG, "Failed to create mutex", "sd_card: %p", sd_card);
    error_handler_deinit(&sd_card->error_handler);
    return ESP_ERR_NO_MEM;
  }

  /* Initialize bus manager */
  err = pstar_bus_manager_init(&sd_card->bus_manager, tag);
  if (err != ESP_OK) {
    log_error(TAG, "Failed to initialize bus manager", "err: %d", err);
    error_handler_deinit(&sd_card->error_handler);
    vSemaphoreDelete(sd_card->mutex);
    sd_card->mutex = NULL;
    return err;
  }

  /* Initialize NVS if not already initialized */
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    err = nvs_flash_erase();
    if (err != ESP_OK) {
      log_warn(sd_card->tag, "NVS Warning", "Failed to erase NVS: %s", esp_err_to_name(err));
    }
    err = nvs_flash_init();
  }

  if (err != ESP_OK) {
    log_warn(sd_card->tag, "NVS Warning", "Failed to initialize NVS: %s", esp_err_to_name(err));
    err = ESP_OK; /* Treat NVS failure as non-fatal for HAL init */
  }

  log_info(TAG,
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
    log_error(TAG, "Set Task Config Error", "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

  if (sd_card->initialized) {
    log_warn(TAG,
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

  log_info(sd_card->tag,
           "Task Config Updated",
           "SD card task configuration updated: stack=%lu, priority=%u, timeout=%lu",
           stack_size,
           priority,
           (unsigned long)mutex_timeout);
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
    sd_card->interface_discovery_complete                   = false;
    sd_card->interface_attempt_count                        = 0;
    sd_card->interface_info[k_sd_interface_spi].attempted   = false;
    sd_card->interface_info[k_sd_interface_spi].error_count = 0;

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
      storage_update_state_machine(sd_card, k_sd_state_interface_ready);
      log_info(sd_card->tag,
               "Remount Success",
               "Successfully remounted SD card with %s interface",
               sd_card_interface_to_string(sd_card->current_interface));
    }

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

  if (bus_width != k_sd_bus_width_1bit && bus_width != k_sd_bus_width_4bit) {
    log_error(sd_card->tag, "Invalid bus width", "Bus width must be 1 or 4, got: %d", bus_width);
    return ESP_ERR_INVALID_ARG;
  }

  bool      mutex_taken = false;
  esp_err_t result      = ESP_OK;

  if (sd_card->mutex != NULL &&
      xSemaphoreTake(sd_card->mutex, sd_card->task_config.mutex_timeout) == pdTRUE) {
    mutex_taken = true;

    if (sd_card->bus_width == bus_width) {
      log_info(sd_card->tag,
               "Bus Width",
               "Already using %s bus width",
               storage_bus_width_to_string(bus_width));
      storage_release_mutex_if_taken(sd_card, &mutex_taken);
      return ESP_OK;
    }

    // For SPI, bus width doesn't really matter, but keep the setting for future SDIO support
    log_info(sd_card->tag,
             "Bus Width Change",
             "Changing from %s to %s bus width (SPI mode still active)",
             storage_bus_width_to_string(sd_card->bus_width),
             storage_bus_width_to_string(bus_width));

    sd_card->bus_width = bus_width;
    storage_save_working_config(sd_card);

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

  if (sd_card->mount_task_handle != NULL) {
    sd_card->mount_task_exit_requested = true;
    TickType_t       start_time        = xTaskGetTickCount();
    const TickType_t max_wait_time     = pdMS_TO_TICKS(1000);
    while (sd_card->mount_task_handle != NULL &&
           (xTaskGetTickCount() - start_time) < max_wait_time) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (sd_card->mount_task_handle != NULL) {
      log_warn(sd_card->tag,
               "Cleanup Warning",
               "Mount task did not exit cleanly, forcing deletion.");
      vTaskDelete(sd_card->mount_task_handle);
      sd_card->mount_task_handle = NULL;
    }
  }

  if (sd_card->mutex != NULL) {
    if (xSemaphoreTake(sd_card->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      mutex_taken = true;
    } else {
      log_error(sd_card->tag,
                "Cleanup Error",
                "Failed to acquire mutex for cleanup - forcing cleanup anyway");
    }
  }

  if (atomic_load(&sd_card->card_available)) {
    esp_err_t err = sd_card_unmount(sd_card);
    if (err != ESP_OK) {
      log_error(sd_card->tag,
                "Cleanup Error",
                "Failed to unmount SD card: %s",
                esp_err_to_name(err));
      result = err;
    }
  }

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  if (sd_card->initialized && sd_card->pin_config.gpio_det_pin >= 0) {
    pstar_bus_gpio_isr_remove(&sd_card->bus_manager,
                              CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME,
                              sd_card->pin_config.gpio_det_pin);
    pin_validator_unregister_pin(sd_card->pin_config.gpio_det_pin, "SD Card HAL");
  }
#endif
  pstar_bus_manager_deinit(&sd_card->bus_manager);

  if (sd_card->card != NULL) {
    free(sd_card->card);
    sd_card->card = NULL;
  }

  sd_card->initialized = false;
  atomic_store(&sd_card->card_available, false);
  sd_card->interface_discovery_complete = false;
  sd_card->current_interface            = k_sd_interface_none;
  sd_card->state                        = k_sd_state_idle;
  sd_card->availability_callback        = NULL;

  storage_release_mutex_if_taken(sd_card, &mutex_taken);

  if (sd_card->mutex != NULL) {
    vSemaphoreDelete(sd_card->mutex);
    sd_card->mutex = NULL;
  }

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

  storage_release_mutex_if_taken(sd_card, &mutex_taken);
  return ESP_OK;
}
