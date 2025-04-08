/* components/pstar_storage_hal/pstar_storage_common.c */

#include "pstar_storage_common.h"
#include "pstar_storage_hal.h"  /* Add for access to interface string function */
#include "pstar_log_handler.h"
#include "pstar_pin_validator.h"
#include "pstar_bus_gpio.h"
#include "pstar_bus_manager.h"

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <string.h>  /* Add for memcpy */
#include <sys/stat.h>
#include <unistd.h>

#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

/* Constants ******************************************************************/
static const char* TAG = "Storage Common";

/* Functions ******************************************************************/

void storage_release_mutex_if_taken(sd_card_hal_t* sd_card, bool* mutex_taken)
{
  if (sd_card != NULL && sd_card->mutex != NULL && *mutex_taken) {
    xSemaphoreGive(sd_card->mutex);
    *mutex_taken = false;
  }
}

void storage_update_state_machine(sd_card_hal_t* sd_card, sd_state_t new_state)
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
    storage_release_mutex_if_taken(sd_card, &mutex_taken);
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
    storage_save_working_config(sd_card);
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
  storage_release_mutex_if_taken(sd_card, &mutex_taken);
}

esp_err_t storage_save_working_config(sd_card_hal_t* sd_card)
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

esp_err_t storage_load_working_config(sd_card_hal_t* sd_card)
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

bool storage_sd_card_is_inserted(sd_card_hal_t* sd_card)
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
      storage_release_mutex_if_taken(sd_card, &mutex_taken);
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
  storage_release_mutex_if_taken(sd_card, &mutex_taken);

  return card_inserted;
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED */
}

void storage_notify_availability(sd_card_hal_t* sd_card, bool available)
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

const char* storage_bus_width_to_string(sd_bus_width_t bus_width)
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

/* Path validation helpers ****************************************************/

static bool check_directory_traversal(const char* path)
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

static bool check_allowed_characters(const char* path)
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

bool storage_path_is_safe(const sd_card_hal_t* sd_card, const char* path)
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
  if (!check_directory_traversal(path)) {
    return false;
  }
  if (!check_allowed_characters(path)) {
    return false;
  }

  return true;
}

bool storage_path_is_safe_simple(const char* path)
{
  if (path == NULL) {
    return false;
  }

  /* Perform common security checks */
  if (!check_directory_traversal(path)) {
    return false;
  }
  if (!check_allowed_characters(path)) {
    return false;
  }

  return true;
}

bool storage_is_path_within_mount(const sd_card_hal_t* sd_card, const char* path)
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

esp_err_t storage_create_directory_if_needed(const sd_card_hal_t* sd_card, const char* path)
{
  if (sd_card == NULL || path == NULL) {
    log_error(TAG, "Directory Error", "NULL parameter provided to directory creation");
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if path is safe */
  if (!storage_path_is_safe(sd_card, path)) {
    /* storage_path_is_safe already logs the specific error */
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

void storage_measure_card_performance(sd_card_hal_t* sd_card)
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
  if (!storage_path_is_safe_simple(system_dir) || !storage_is_path_within_mount(sd_card, system_dir)) {
    log_error(sd_card->tag,
              "Performance Error",
              "System directory path '%s' is not valid",
              system_dir);
    free(buffer);
    return;
  }

  /* Create the system directory if it does not exist */
  esp_err_t dir_err = storage_create_directory_if_needed(sd_card, system_dir);
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

esp_err_t storage_sd_card_reset(void* context)
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
      sd_card_unmount(sd_card); /* unmount handles its own state changes/notifications */
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
  if (storage_sd_card_is_inserted(sd_card)) { /* is_inserted takes+gives its own mutex if needed */
    storage_update_state_machine(
      sd_card,
      k_sd_state_interface_discovery); /* update_state takes+gives its own mutex */
    esp_err_t mount_result =
      sd_card_try_interfaces(sd_card); /* try_interfaces handles its own state/mutex */
    if (mount_result != ESP_OK) {
      log_error(sd_card->tag,
                "Reset Error",
                "Failed to initialize SD card interface after reset: %s",
                esp_err_to_name(mount_result));
      result = mount_result;
      /* Update state machine to error after failed reset attempt */
      storage_update_state_machine(sd_card,
                                k_sd_state_error); /* update_state takes+gives its own mutex */
      return result;                               /* Return error */
    }
    /* If mounted successfully, update state */
    if (atomic_load(&sd_card->card_available)) {
      storage_update_state_machine(
        sd_card,
        k_sd_state_interface_ready); /* update_state takes+gives its own mutex */
    } else {
      /* Mount failed even after reset */
      storage_update_state_machine(sd_card,
                                k_sd_state_error); /* update_state takes+gives its own mutex */
    }
  } else {
    storage_update_state_machine(sd_card,
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

esp_err_t storage_register_sd_card_pins(sd_card_hal_t* sd_card)
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
