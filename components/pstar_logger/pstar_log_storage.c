// ===================== ./components/pstar_logger/pstar_log_storage.c ========================

/* components/pstar_logging/log_storage.c */

#include "pstar_log_storage.h"
#include "esp_system.h"
#include "pstar_log_macros.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <unistd.h>
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED && CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED
#include <zlib.h>
#endif
#include "esp_log.h" // Use ESP_LOGx directly for internal storage messages
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdatomic.h>
#include "sdkconfig.h" // Ensure sdkconfig is included for Kconfig defines

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
#include "pstar_file_write_manager.h" /* Need this for file operations */
#include "pstar_sd_card_hal.h" /* Need this for SD availability check */
#endif


static const char* TAG = "Log Storage";

/* Constants ******************************************************************/

#define LOG_STORAGE_MUTEX_TIMEOUT_MS (1000)

/* Globals (Static) ***********************************************************/

// Buffer for logs when SD card is unavailable or storage not fully initialized
static log_entry_t s_log_buffer[CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE] = {0};
static uint32_t    s_log_buffer_index                                     = 0;
// Mutex protects s_log_buffer, s_log_buffer_index, s_current_log_file, and s_file_manager
static SemaphoreHandle_t s_log_mutex = NULL;
// Atomic flags for checking state without needing mutex immediately
static _Atomic bool s_log_storage_initialized = false;
static _Atomic bool s_sd_card_available       = false;

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
static file_write_manager_t* s_file_manager = NULL; /* File write manager instance */
static char s_current_log_file[PSTAR_LOGGING_CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH + 64] = {0}; /* Current log file path (relative) */
static sd_card_hal_t*        s_sd_card_hal_instance = NULL; // Store HAL instance for availability checks
#endif

/* Private Helper Functions (Declarations) ************************************/

static inline time_t priv_timestamp_us_to_seconds(uint64_t timestamp_us);
static inline uint64_t priv_timestamp_us_to_milliseconds(uint64_t timestamp_us);
static inline int priv_format_date_string(char* buffer, size_t buffer_size, const struct tm *const timeinfo);
// static inline int priv_format_log_entry(char* buffer, size_t buffer_size, const struct tm* const timeinfo, uint64_t milliseconds, const char* const level_str, const char* const message); // Unused

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
// Declare these only if SD is enabled, as they are only defined/used then
static inline int priv_format_log_filepath(char* buffer, size_t buffer_size, const struct tm* const timeinfo, const char* const extension);
static void priv_generate_log_file_path(char* file_path, size_t file_path_len);
static bool priv_check_log_rotation(void);
static esp_err_t priv_rotate_log_file(void);
#if CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED
static esp_err_t priv_compress_data(const char* const input, size_t input_len, char* output, size_t* output_len);
#endif
static esp_err_t priv_write_log_data(const char* const file_path, const char* const data);
static esp_err_t priv_flush_log_buffer(void);
#endif // CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED

/* Public Functions ***********************************************************/

// Conditional function signature definition
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
esp_err_t log_storage_init(file_write_manager_t* manager, sd_card_hal_t* sd_card)
#else
esp_err_t log_storage_init(void* manager_arg, void* sd_card_arg)
#endif
{
  // Use ESP_LOGI for initial messages
  ESP_LOGI(TAG, "log_storage_init called...");

  // Check if already initialized (atomically)
  if (atomic_load(&s_log_storage_initialized)) {
    ESP_LOGW(TAG, "Log storage already initialized.");
    return ESP_OK;
  }

  // Create mutex regardless of SD card status
  if (s_log_mutex == NULL) { // Check if mutex already exists (shouldn't happen if init flag is correct)
      s_log_mutex = xSemaphoreCreateMutex();
      if (s_log_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create log storage mutex"); // Use ESP_LOGE for critical internal errors
        return ESP_FAIL;
      }
  }

#if !CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  // Mark arguments as unused when SD is disabled
  (void)manager_arg;
  (void)sd_card_arg;
  ESP_LOGI(TAG, "Initializing log storage (SD card support DISABLED). Buffering only."); // Use ESP_LOGI
  // Minimal initialization: set flag, ensure SD available is false
  atomic_store(&s_sd_card_available, false);
  atomic_store(&s_log_storage_initialized, true);
  return ESP_OK;

#else
  // SD Card logging is enabled in Kconfig
  ESP_LOGI(TAG, "Initializing log storage (SD card support ENABLED)."); // Use ESP_LOGI

  // Use the correct types for the enabled case
  file_write_manager_t* actual_manager = (file_write_manager_t*) manager;
  sd_card_hal_t* actual_sd_card = (sd_card_hal_t*) sd_card;

  if (actual_manager == NULL || actual_sd_card == NULL) {
    ESP_LOGE(TAG, "Init Error: manager and sd_card pointers are required when SD logging is enabled."); // Use ESP_LOGE
    // Don't delete mutex here, might be needed for buffer access
    return ESP_ERR_INVALID_ARG;
  }

  // --- Full Initialization ---
  // Store the file manager and SD HAL instance
  s_file_manager = actual_manager;
  s_sd_card_hal_instance = actual_sd_card;

  // Get initial SD card availability status
  atomic_store(&s_sd_card_available, sd_card_is_available(actual_sd_card));

  // Register for SD card availability notifications
  esp_err_t reg_err = sd_card_register_availability_callback(actual_sd_card, log_storage_set_sd_available);
  if (reg_err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register SD availability callback: %s", esp_err_to_name(reg_err)); // Use ESP_LOGE
    // Don't delete mutex, cleanup will handle it
    return reg_err;
  }

  // Mark storage as fully initialized
  atomic_store(&s_log_storage_initialized, true);

  ESP_LOGI(TAG, "Log storage initialized successfully (SD Available: %s)", atomic_load(&s_sd_card_available) ? "Yes" : "No"); // Use ESP_LOGI
  return ESP_OK;
#endif
}


bool log_storage_is_initialized(void) {
    return atomic_load(&s_log_storage_initialized);
}


void log_storage_set_sd_available(bool available)
{
#if !CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  // ESP_LOGW(TAG, "SD card availability change ignored: SD card support is disabled"); // Reduce noise
  (void)available; // Mark as unused
  return;
#else
  // Check init status first (atomically)
  if (!atomic_load(&s_log_storage_initialized)) {
    ESP_LOGW(TAG, "Ignoring SD availability change: Log storage not initialized."); // Use ESP_LOGW
    return;
  }

  // Atomically update the flag and check if the state actually changed
  bool old_value = atomic_exchange(&s_sd_card_available, available);

  if (old_value == available) {
    return; // No change
  }

  if (available) {
    ESP_LOGI(TAG, "SD card became available, attempting to flush buffered logs."); // Use ESP_LOGI
    // Attempt flush - it will handle mutex and check init status again internally
    esp_err_t flush_err = log_storage_flush();
    if (flush_err != ESP_OK && flush_err != ESP_ERR_TIMEOUT && flush_err != ESP_ERR_INVALID_STATE) {
         ESP_LOGE(TAG, "Error flushing buffer after SD card became available: %s", esp_err_to_name(flush_err)); // Use ESP_LOGE
    }
  } else {
    ESP_LOGW(TAG, "SD card became unavailable, logs will be buffered."); // Use ESP_LOGW
  }
#endif
}

esp_err_t log_storage_write(esp_log_level_t level, const char* const message)
{
  // Storage doesn't need init check here, as it only affects SD writing. Buffering should always work if mutex exists.
  if (message == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // Ensure mutex is created
  if (s_log_mutex == NULL) {
      ESP_LOGE(TAG, "Log storage mutex not initialized!"); // Use ESP_LOGE
      return ESP_ERR_INVALID_STATE;
  }

  // Acquire mutex to access buffer
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(LOG_STORAGE_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to acquire mutex for log storage write"); // Use ESP_LOGE
    return ESP_ERR_TIMEOUT;
  }

  esp_err_t result = ESP_OK;

  // Store log in buffer
  if (s_log_buffer_index < CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE) {
    s_log_buffer[s_log_buffer_index].level     = level;
    s_log_buffer[s_log_buffer_index].timestamp = esp_timer_get_time();
    strlcpy(s_log_buffer[s_log_buffer_index].buffer, message, CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH);
    s_log_buffer[s_log_buffer_index].buffer[CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH - 1] = '\0'; // Ensure null termination
    s_log_buffer_index++;
  } else {
    // Buffer is full

    #if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
      // If SD is enabled, try flushing ONLY if storage is fully initialized and SD is available
      if (atomic_load(&s_log_storage_initialized) && atomic_load(&s_sd_card_available)) {
        ESP_LOGW(TAG, "Log buffer full, attempting to flush..."); // Use ESP_LOGW
        // We hold the mutex, call private flush directly
        result = priv_flush_log_buffer(); // This function handles rotation/write

        if (result != ESP_OK || s_log_buffer_index >= CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE) {
          // Flush failed or didn't clear space, drop oldest log
          ESP_LOGE(TAG, "Buffer Overflow: SD flush failed/insufficient, dropping oldest log."); // Use ESP_LOGE
          // Shift buffer content
          memmove(&s_log_buffer[0], &s_log_buffer[1], sizeof(log_entry_t) * (CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1));
          s_log_buffer_index = CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1; // Make space for new log
        }
      } else {
        // SD not ready or storage not init, drop oldest log
        ESP_LOGE(TAG, "Buffer Overflow: SD not available/initialized, dropping oldest log."); // Use ESP_LOGE
        memmove(&s_log_buffer[0], &s_log_buffer[1], sizeof(log_entry_t) * (CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1));
        s_log_buffer_index = CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1;
      }
    #else
      // SD Disabled: Buffer is full, drop oldest log
      ESP_LOGE(TAG, "Buffer Overflow: Log buffer full (SD disabled), dropping oldest log."); // Use ESP_LOGE
      memmove(&s_log_buffer[0], &s_log_buffer[1], sizeof(log_entry_t) * (CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1));
      s_log_buffer_index = CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1;
    #endif

    // Store the current log in the now available last slot (or newly freed slot)
    s_log_buffer[s_log_buffer_index].level     = level;
    s_log_buffer[s_log_buffer_index].timestamp = esp_timer_get_time();
    strlcpy(s_log_buffer[s_log_buffer_index].buffer, message, CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH);
    s_log_buffer[s_log_buffer_index].buffer[CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH - 1] = '\0';
    s_log_buffer_index++; // Increment index for the newly added log
  }

  // If buffer is now full AND SD storage is fully ready, try to flush again (relevant if initial write filled the buffer)
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  if (s_log_buffer_index >= CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE &&
      atomic_load(&s_log_storage_initialized) &&
      atomic_load(&s_sd_card_available))
  {
      // Call private flush directly as we hold the mutex
      esp_err_t flush_ret = priv_flush_log_buffer();
       if (flush_ret != ESP_OK) {
           result = flush_ret; // Report flush error if it occurred
       }
  }
#endif

  xSemaphoreGive(s_log_mutex);
  return result; // Return ESP_OK or error from flush attempt
}


esp_err_t log_storage_flush(void)
{
#if !CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  ESP_LOGI(TAG, "Flush called but SD logging is disabled."); // Use ESP_LOGI
  return ESP_OK; // Nothing to flush
#else
  // Check if storage component is fully initialized
  if (!atomic_load(&s_log_storage_initialized)) {
    ESP_LOGW(TAG, "Flush called but log storage is not fully initialized."); // Use ESP_LOGW
    return ESP_ERR_INVALID_STATE;
  }

  // Check if SD card is available (atomically)
   if (!atomic_load(&s_sd_card_available)) {
       ESP_LOGW(TAG, "Flush skipped: SD card not available."); // Use ESP_LOGW
       return ESP_OK; // Not an error, just can't flush now
   }

  // Ensure mutex is created
  if (s_log_mutex == NULL) {
      ESP_LOGE(TAG, "Flush Error: Log storage mutex not initialized!"); // Use ESP_LOGE
      return ESP_ERR_INVALID_STATE;
  }

  // Acquire mutex
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(LOG_STORAGE_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to acquire mutex for log flush"); // Use ESP_LOGE
    return ESP_ERR_TIMEOUT;
  }

  // Call the internal flush function
  esp_err_t ret = priv_flush_log_buffer();

  xSemaphoreGive(s_log_mutex);
  return ret;
#endif
}

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED && CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED
esp_err_t log_storage_set_compression(bool enabled)
{
  (void)enabled; // Mark as unused
  ESP_LOGW(TAG, "Log compression is controlled by Kconfig and cannot be changed at runtime."); // Use ESP_LOGW
  return ESP_OK; // No-op
}

bool log_storage_is_compression_enabled(void)
{
  return true; // Kconfig enabled it
}
#endif // Compression enabled checks

esp_err_t log_storage_cleanup(void)
{
  ESP_LOGI(TAG, "Beginning log storage cleanup..."); // Use ESP_LOGI
  esp_err_t final_ret = ESP_OK;

  // Atomically check and set initialized flag
  bool was_initialized __attribute__((unused)) = atomic_exchange(&s_log_storage_initialized, false); // Mark unused if SD disabled

  // If mutex wasn't created, there's nothing else to clean
  if (s_log_mutex == NULL) {
      ESP_LOGW(TAG, "Cleanup: Mutex was not initialized."); // Use ESP_LOGW
      atomic_store(&s_sd_card_available, false); // Ensure flag is false
      return ESP_OK;
  }

  // Attempt to acquire the mutex with a generous timeout for cleanup
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(LOG_STORAGE_MUTEX_TIMEOUT_MS * 2)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to acquire mutex during cleanup. Resources may leak!"); // Use ESP_LOGE
    // Cannot safely proceed further without mutex, but try deleting it anyway
    final_ret = ESP_ERR_TIMEOUT;
  } else {
    // Mutex acquired

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
    // Attempt final flush only if it was fully initialized and SD might be available
    if (was_initialized && atomic_load(&s_sd_card_available)) {
      ESP_LOGI(TAG, "Cleanup: Attempting final buffer flush."); // Use ESP_LOGI
      esp_err_t flush_ret = priv_flush_log_buffer();
      if (flush_ret != ESP_OK) {
        ESP_LOGW(TAG, "Final flush failed during cleanup: %s", esp_err_to_name(flush_ret)); // Use ESP_LOGW
        if (final_ret == ESP_OK) final_ret = flush_ret;
      }
    }

    // Reset file manager pointer and current log file path
    s_file_manager = NULL;
    memset(s_current_log_file, 0, sizeof(s_current_log_file));
    s_sd_card_hal_instance = NULL; // Clear HAL instance pointer
#endif

    // Clear log buffer state
    s_log_buffer_index = 0;
    memset(s_log_buffer, 0, sizeof(s_log_buffer));

    // Give the mutex back before deleting it
    xSemaphoreGive(s_log_mutex);
  } // End mutex handling block

  // Delete the mutex
  vSemaphoreDelete(s_log_mutex);
  s_log_mutex = NULL;

  // Reset atomic SD availability flag
  atomic_store(&s_sd_card_available, false);

  ESP_LOGI(TAG, "Log storage cleanup %s.", (final_ret == ESP_OK) ? "successful" : "completed with warnings/errors"); // Use ESP_LOGI
  return final_ret;
}


/* Private Helper Functions (Definitions) *************************************/

static inline time_t priv_timestamp_us_to_seconds(uint64_t timestamp_us)
{
  return timestamp_us / 1000000;
}

static inline uint64_t priv_timestamp_us_to_milliseconds(uint64_t timestamp_us)
{
  return (timestamp_us % 1000000) / 1000;
}

static inline int priv_format_date_string(char* buffer, size_t buffer_size, const struct tm *const timeinfo)
{
  if (!buffer || buffer_size == 0 || !timeinfo) return -1;
  return snprintf(buffer, buffer_size, "%04d-%02d-%02d", timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday);
}

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
// Only define these if SD card is enabled
static inline int priv_format_log_filepath(char* buffer, size_t buffer_size, const struct tm* const timeinfo, const char* const extension)
{
  if (!buffer || buffer_size == 0 || !timeinfo || !extension) return -1;

  char date_str[CONFIG_PSTAR_KCONFIG_LOGGING_DATE_STRING_BUFFER_SIZE];
  int date_written = priv_format_date_string(date_str, sizeof(date_str), timeinfo);
  if (date_written < 0 || (size_t)date_written >= sizeof(date_str)) {
    ESP_LOGE(TAG, "Date Format Error: Failed to format date string for log file path"); // Use ESP_LOGE
    return -1;
  }

  // Path is relative to the mount point
  return snprintf(buffer, buffer_size, "%s/%s/%04d-%02d-%02d_%02d-%02d-%02d%s",
                  CONFIG_PSTAR_KCONFIG_LOGGING_BASE_DIR, // e.g., "logs"
                  date_str,                              // e.g., "2023-10-27"
                  timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
                  timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
                  extension);
}

static void priv_generate_log_file_path(char* file_path, size_t file_path_len)
{
  if (file_path == NULL || file_path_len == 0) {
    ESP_LOGE(TAG, "File Path Error: Invalid buffer provided for log file path generation"); // Use ESP_LOGE
    return;
  }

  struct tm timeinfo;
  time_t    now = time(NULL);
  localtime_r(&now, &timeinfo);

  const char* const extension = log_storage_is_compression_enabled() ?
                                CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSED_EXTENSION :
                                ".log"; // Use .log for uncompressed
  int written = priv_format_log_filepath(file_path, file_path_len, &timeinfo, extension);

  if (written < 0 || (size_t)written >= file_path_len) {
    ESP_LOGE(TAG, "File Path Error: Failed to format log file path or buffer too small"); // Use ESP_LOGE
    file_path[0] = '\0'; // Ensure null termination on error
  }
}

static bool priv_check_log_rotation(void)
{
    // Assumes SD card is available and storage is initialized
    // Assumes mutex is held

    // Check if a log file path exists
    if (strlen(s_current_log_file) == 0) {
        ESP_LOGI(TAG, "Rotation needed: No current log file path."); // Use ESP_LOGI
        return true; // Need to generate the first file path
    }

    // Construct the full path for stat
    char full_stat_path[PSTAR_LOGGING_CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH * 2]; // Extra space
    if (!s_sd_card_hal_instance || !s_sd_card_hal_instance->mount_path) {
         ESP_LOGE(TAG, "Rotation Check Error: SD HAL instance or mount path is NULL."); // Use ESP_LOGE
         return false; // Cannot check without mount path
    }
    snprintf(full_stat_path, sizeof(full_stat_path), "%s/%s", s_sd_card_hal_instance->mount_path, s_current_log_file);
    full_stat_path[sizeof(full_stat_path) - 1] = '\0';


    /* Check file size */
    struct stat st;
    if (stat(full_stat_path, &st) != 0) {
        if (errno == ENOENT) {
            ESP_LOGW(TAG, "Rotation needed: Current log file '%s' not found.", full_stat_path); // Use ESP_LOGW
            return true; // File doesn't exist, need a new one
        } else {
            ESP_LOGE(TAG, "Stat Error: Failed to stat current log file '%s': %s", full_stat_path, strerror(errno)); // Use ESP_LOGE
            return false; // Avoid rotation if stat fails for other reasons
        }
    }

    if (st.st_size >= PSTAR_LOGGING_MAX_FILE_SIZE) {
        ESP_LOGI(TAG, "Rotation needed: File size %ld >= max %d for %s", (long)st.st_size, PSTAR_LOGGING_MAX_FILE_SIZE, full_stat_path); // Use ESP_LOGI
        return true;
    }

    /* Check if date has changed */
    struct tm timeinfo;
    time_t    now = time(NULL);
    localtime_r(&now, &timeinfo);

    char date_str[CONFIG_PSTAR_KCONFIG_LOGGING_DATE_STRING_BUFFER_SIZE];
    int date_written = priv_format_date_string(date_str, sizeof(date_str), &timeinfo);
    if (date_written < 0 || (size_t)date_written >= sizeof(date_str)) {
        ESP_LOGE(TAG, "Date Format Error: Failed to format date string for rotation check"); // Use ESP_LOGE
        return false;
    }

    // Extract date part from the current RELATIVE log file path's directory component
    // Expected relative format: logs/YYYY-MM-DD/YYYY-MM-DD_HH-MM-SS.ext
    char current_date_part[CONFIG_PSTAR_KCONFIG_LOGGING_DATE_STRING_BUFFER_SIZE] = {0};
    const char* dir_part_start = strchr(s_current_log_file, '/'); // Find first slash after base dir ("logs/")
     if (dir_part_start) {
        dir_part_start++; // Move past the slash
        const char* dir_part_end = strchr(dir_part_start, '/');
        if (dir_part_end) {
            size_t len = dir_part_end - dir_part_start;
            if (len < sizeof(current_date_part)) {
                strncpy(current_date_part, dir_part_start, len);
                current_date_part[len] = '\0';
            }
        }
    }


    if (strlen(current_date_part) == 0 || strcmp(current_date_part, date_str) != 0) {
        ESP_LOGI(TAG, "Rotation needed: Date changed. Current: '%s', New: '%s'", current_date_part, date_str); // Use ESP_LOGI
        return true;
    }

    return false; // No rotation needed
}


static esp_err_t priv_rotate_log_file(void)
{
  // Assumes mutex is held
  if (!priv_check_log_rotation()) {
    return ESP_OK; // No rotation needed
  }

  // Generate new relative log file path
  priv_generate_log_file_path(s_current_log_file, sizeof(s_current_log_file));

  if (strlen(s_current_log_file) == 0) {
      ESP_LOGE(TAG, "Log Rotation Error: Failed to generate new log file path during rotation"); // Use ESP_LOGE
      return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Rotating to new log file: %s (relative)", s_current_log_file); // Use ESP_LOGI
  return ESP_OK;
}

#if CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED
static esp_err_t priv_compress_data(const char* const input, size_t input_len, char* output, size_t* output_len)
{
  if (input == NULL || input_len == 0 || output == NULL || output_len == NULL || *output_len == 0) {
    ESP_LOGE(TAG, "Compression Error: Invalid arguments"); // Use ESP_LOGE
    return ESP_ERR_INVALID_ARG;
  }

  z_stream stream;
  memset(&stream, 0, sizeof(stream));

  int ret = deflateInit2(&stream, Z_DEFAULT_COMPRESSION, Z_DEFLATED,
                         PSTAR_LOGGING_ZLIB_GZIP_WINDOW_BITS,
                         CONFIG_PSTAR_KCONFIG_LOGGING_ZLIB_MEM_LEVEL, Z_DEFAULT_STRATEGY);
  if (ret != Z_OK) {
    ESP_LOGE(TAG, "Zlib Init Failed: %d", ret); // Use ESP_LOGE
    return ESP_FAIL;
  }

  stream.next_in   = (Bytef*)input;
  stream.avail_in  = input_len;
  stream.next_out  = (Bytef*)output;
  stream.avail_out = *output_len;

  ret = deflate(&stream, Z_FINISH);
  deflateEnd(&stream);

  if (ret != Z_STREAM_END) {
    ESP_LOGE(TAG, "Compression Failed: zlib error %d (%s)", ret, stream.msg ? stream.msg : "No msg"); // Use ESP_LOGE
    return ESP_FAIL;
  }

  *output_len = stream.total_out;
  return ESP_OK;
}
#endif // CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED

static esp_err_t priv_write_log_data(const char* const file_path_relative, const char* const data)
{
  // Assumes mutex is held
  // Assumes SD card is available and storage is initialized

  if (file_path_relative == NULL || data == NULL || s_file_manager == NULL) {
    ESP_LOGE(TAG, "Write Error: Invalid arguments (path=%p, data=%p, fm=%p)", file_path_relative, data, s_file_manager); // Use ESP_LOGE
    return ESP_ERR_INVALID_ARG;
  }

#if CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED
    size_t data_len = strlen(data);
    // Use a dynamically sized buffer for compression output, based on input size
    size_t compress_buf_size_needed = deflateBound(NULL, data_len); // zlib's worst-case estimate
    if (compress_buf_size_needed > PSTAR_LOGGING_MAX_COMPRESSION_INPUT_SIZE) {
         ESP_LOGE(TAG, "Compression Error: Input data size (%zu) exceeds reasonable limit (%d)", data_len, PSTAR_LOGGING_MAX_COMPRESSION_INPUT_SIZE); // Use ESP_LOGE
         return ESP_ERR_NO_MEM; // Indicate size issue
    }
    // Use a slightly larger buffer than strictly needed just in case
     size_t alloc_size = compress_buf_size_needed + 128;
     if (alloc_size > CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_BUFFER_SIZE) {
         // If estimate exceeds configured buffer, use estimate + margin, up to a max
         alloc_size = compress_buf_size_needed + 1024; // Add more margin
         if (alloc_size > PSTAR_LOGGING_MAX_COMPRESSION_INPUT_SIZE) {
             alloc_size = PSTAR_LOGGING_MAX_COMPRESSION_INPUT_SIZE;
         }
         ESP_LOGW(TAG, "Compression buffer estimate %zu exceeds Kconfig %d, using %zu", compress_buf_size_needed, CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_BUFFER_SIZE, alloc_size); // Use ESP_LOGW
     } else {
         alloc_size = CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_BUFFER_SIZE; // Use Kconfig size if sufficient
     }


    char *compress_buffer = malloc(alloc_size);
    if (compress_buffer == NULL) {
      ESP_LOGE(TAG, "Memory Error: Failed to allocate compression buffer (size %zu)", alloc_size); // Use ESP_LOGE
      return ESP_ERR_NO_MEM;
    }

    size_t output_len = alloc_size; // Pass the actual allocated size

    esp_err_t ret = priv_compress_data(data, data_len, compress_buffer, &output_len);
    if (ret != ESP_OK) {
      free(compress_buffer);
      return ret;
    }

    // Write compressed data using the binary enqueue function
    ret = file_write_binary_enqueue(s_file_manager, file_path_relative, compress_buffer, output_len);
    free(compress_buffer);
    return ret;

#else // Compression disabled
    // Write uncompressed data using the standard enqueue function
    return file_write_enqueue(s_file_manager, file_path_relative, data);
#endif
}


static esp_err_t priv_flush_log_buffer(void)
{
  // Assumes mutex is held
  // Assumes SD card logging is enabled

  if (s_log_buffer_index == 0) {
    return ESP_OK; // Nothing to flush
  }

  // Check initialization and availability (redundant check, but safe)
   if (!atomic_load(&s_log_storage_initialized) || !atomic_load(&s_sd_card_available)) {
       ESP_LOGW(TAG, "Flush skipped: Storage not initialized or SD card not available."); // Use ESP_LOGW
       // Don't clear buffer if we can't write
       return ESP_FAIL; // Indicate flush couldn't happen
   }

  // Ensure we have a valid log file path (handles rotation)
  if (priv_rotate_log_file() != ESP_OK) {
    ESP_LOGE(TAG, "Flush Error: Failed to rotate log file before flushing."); // Use ESP_LOGE
    return ESP_FAIL;
  }
  if (strlen(s_current_log_file) == 0) {
    ESP_LOGE(TAG, "Flush Error: Invalid log file path after rotation attempt."); // Use ESP_LOGE
    return ESP_FAIL;
  }


  // Combine all buffered logs into a single string for efficient writing/compression
  // Estimate total size needed
  size_t total_size = 0;
  for (uint32_t i = 0; i < s_log_buffer_index; i++) {
      total_size += strlen(s_log_buffer[i].buffer) + 1; // +1 for newline
  }
   total_size += 1; // +1 for final null terminator

   // Check against a reasonable maximum size before allocating
    if (total_size > PSTAR_LOGGING_MAX_COMPRESSION_INPUT_SIZE * 2) { // Allow larger uncompressed buffer
        ESP_LOGE(TAG, "Total log data size (%zu) exceeds limit (%d)", total_size, PSTAR_LOGGING_MAX_COMPRESSION_INPUT_SIZE * 2); // Use ESP_LOGE
        // Strategy: Flush in chunks or return error? For now, return error.
        return ESP_ERR_NO_MEM;
    }

  char* all_logs = malloc(total_size);
  if (!all_logs) {
    ESP_LOGE(TAG, "Memory Error: Failed to allocate buffer for combining logs (size: %zu)", total_size); // Use ESP_LOGE
    return ESP_ERR_NO_MEM;
  }
  all_logs[0] = '\0'; // Start with an empty string

  // Concatenate all log entries
  size_t current_pos = 0;
  for (uint32_t i = 0; i < s_log_buffer_index; i++) {
      // The buffer already contains the fully formatted log line from log_write_va
      int written = snprintf(all_logs + current_pos, total_size - current_pos, "%s\n", s_log_buffer[i].buffer);
      if (written < 0 || (size_t)written >= total_size - current_pos) {
          ESP_LOGE(TAG, "Buffer Error: Failed to concatenate log entry %u into buffer.", i); // Use ESP_LOGE
          // Continue trying to write what we have? Or stop? Stop for now.
          free(all_logs);
          return ESP_FAIL;
      }
      current_pos += written;
  }

  // Write the combined logs (will handle compression internally if enabled)
  esp_err_t write_ret = priv_write_log_data(s_current_log_file, all_logs);
  free(all_logs); // Free the temporary buffer

  if (write_ret != ESP_OK) {
    ESP_LOGE(TAG, "Flush Write Failed: Failed to write/enqueue logs: %s", esp_err_to_name(write_ret)); // Use ESP_LOGE
    // Do NOT reset buffer index if write failed
    return write_ret;
  }

  // Reset buffer index ONLY if write was successful
  ESP_LOGI(TAG, "Successfully flushed %lu log entries.", s_log_buffer_index); // Use ESP_LOGI
  s_log_buffer_index = 0;
  memset(s_log_buffer, 0, sizeof(s_log_buffer)); // Clear buffer contents


  return ESP_OK;
}

#endif // CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED