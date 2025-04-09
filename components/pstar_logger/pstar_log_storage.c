/* ./components/pstar_logger/pstar_log_storage.c */
#include "pstar_log_storage.h"

#include "pstar_log_macros.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "esp_system.h"
/* Conditionally include zlib.h only when compression is enabled */
#ifdef CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
#ifdef CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED
#include <zlib.h>
#endif
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdatomic.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#ifdef CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
/* Include these only when SD card logging is actually enabled */
#include "pstar_file_write_manager.h" /* Include manager header */
#include "pstar_storage_hal.h"        /* Include HAL header */
#include "pstar_storage_hal.h"
#endif

static const char* TAG = "Log Storage";

/* Constants ******************************************************************/

#define LOG_STORAGE_MUTEX_TIMEOUT_MS (1000)
#define LOG_FLUSH_BATCH_SIZE (10)

/* Globals (Static) ***********************************************************/

/* Buffer for logs when SD card is unavailable or storage not fully initialized */
static log_entry_t s_log_buffer[CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE] = {0};
static uint32_t    s_log_buffer_index                                     = 0;
/* Mutex protects s_log_buffer, s_log_buffer_index, s_current_log_file, and s_file_manager */
static SemaphoreHandle_t s_log_mutex = NULL;
/* Atomic flags for checking state without needing mutex immediately */
static _Atomic bool s_log_storage_initialized = false;
static _Atomic bool s_sd_card_available       = false;

#ifdef CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
static file_write_manager_t* s_file_manager = NULL; /* File write manager instance */
static char s_current_log_file[CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH + 64] = {
  0}; /* Current log file path (relative) + margin */
static sd_card_hal_t* s_sd_card_hal_instance =
  NULL; /* Store HAL instance for availability checks */
#endif

/* Private Helper Functions (Declarations) ************************************/

static inline time_t   priv_timestamp_us_to_seconds(uint64_t timestamp_us);
static inline uint64_t priv_timestamp_us_to_milliseconds(uint64_t timestamp_us);
static inline int
priv_format_date_string(char* buffer, size_t buffer_size, const struct tm* const timeinfo);

#ifdef CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
static inline int priv_format_log_filepath(char*                  buffer,
                                           size_t                 buffer_size,
                                           const struct tm* const timeinfo,
                                           const char* const      extension);
static void       priv_generate_log_file_path(char* file_path, size_t file_path_len);
static bool       priv_check_log_rotation(void);
static esp_err_t  priv_rotate_log_file(void);
#ifdef CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED
static esp_err_t
priv_compress_data(const char* const input, size_t input_len, char* output, size_t* output_len);
#endif
static esp_err_t priv_write_log_data(const char* const file_path, const char* const data);
static esp_err_t priv_flush_log_buffer(void);
#endif /* CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED */

/* Public Functions ***********************************************************/

/* Conditional function signature definition */
#ifdef CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
esp_err_t log_storage_init(file_write_manager_t* manager, sd_card_hal_t* sd_card)
#else
esp_err_t log_storage_init(void* manager_arg, void* sd_card_arg)
#endif
{
  /* Use ESP_LOGI for initial messages */
  ESP_LOGI(TAG, "log_storage_init called...");

  /* Check if already initialized (atomically) */
  if (atomic_load(&s_log_storage_initialized)) {
    ESP_LOGW(TAG, "Log storage already initialized.");
    return ESP_OK;
  }

  /* Create mutex regardless of SD card status */
  if (s_log_mutex ==
      NULL) { /* Check if mutex already exists (shouldn't happen if init flag is correct) */
    s_log_mutex = xSemaphoreCreateMutex();
    if (s_log_mutex == NULL) {
      ESP_LOGE(TAG, "Failed to create log storage mutex");
      return ESP_FAIL;
    }
  }

#ifndef CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  /* Mark arguments as unused when SD is disabled */
  (void)manager_arg;
  (void)sd_card_arg;
  ESP_LOGI(TAG, "Initializing log storage (SD card support DISABLED). Buffering only.");
  /* Minimal initialization: set flag, ensure SD available is false */
  atomic_store(&s_sd_card_available, false);
  atomic_store(&s_log_storage_initialized, true);
  return ESP_OK;

#else
  /* SD Card logging is enabled in Kconfig */
  ESP_LOGI(TAG, "Initializing log storage (SD card support ENABLED).");

  /* Use the correct types for the enabled case */
  file_write_manager_t* actual_manager = (file_write_manager_t*)manager;
  sd_card_hal_t*        actual_sd_card = (sd_card_hal_t*)sd_card;

  if (actual_manager == NULL || actual_sd_card == NULL) {
    ESP_LOGE(TAG,
             "Init Error: manager and sd_card pointers are required when SD logging is enabled.");
    /* Don't delete mutex here, might be needed for buffer access */
    return ESP_ERR_INVALID_ARG;
  }

  /* --- Full Initialization --- */
  /* Store the file manager and SD HAL instance */
  s_file_manager         = actual_manager;
  s_sd_card_hal_instance = actual_sd_card;

  /* Get initial SD card availability status */
  atomic_store(&s_sd_card_available, sd_card_is_available(actual_sd_card));

  /* Register for SD card availability notifications */
  esp_err_t reg_err =
    sd_card_register_availability_callback(actual_sd_card, log_storage_set_sd_available);
  if (reg_err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register SD availability callback: %s", esp_err_to_name(reg_err));
    /* Don't delete mutex, cleanup will handle it */
    return reg_err;
  }

  /* Mark storage as fully initialized */
  atomic_store(&s_log_storage_initialized, true);

  ESP_LOGI(TAG,
           "Log storage initialized successfully (SD Available: %s)",
           atomic_load(&s_sd_card_available) ? "Yes" : "No");
  return ESP_OK;
#endif
}

bool log_storage_is_initialized(void)
{
  return atomic_load(&s_log_storage_initialized);
}

void log_storage_set_sd_available(bool available)
{
#ifndef CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  ESP_LOGW(TAG, "SD card availability change ignored: SD card support is disabled");
  (void)available; /* Mark as unused */
  return;
#else
  /* Check init status first (atomically) */
  if (!atomic_load(&s_log_storage_initialized)) {
    ESP_LOGW(TAG, "Ignoring SD availability change: Log storage not initialized.");
    return;
  }

  /* Atomically update the flag and check if the state actually changed */
  bool old_value = atomic_exchange(&s_sd_card_available, available);

  if (old_value == available) {
    return; /* No change */
  }

  if (available) {
    ESP_LOGI(TAG, "SD card became available, attempting to flush buffered logs.");
    /* Attempt flush - it will handle mutex and check init status again internally */
    esp_err_t flush_err = log_storage_flush();
    if (flush_err != ESP_OK && flush_err != ESP_ERR_TIMEOUT && flush_err != ESP_ERR_INVALID_STATE) {
      ESP_LOGE(TAG,
               "Error flushing buffer after SD card became available: %s",
               esp_err_to_name(flush_err));
    }
  } else {
    ESP_LOGW(TAG, "SD card became unavailable, logs will be buffered.");
  }
#endif
}

esp_err_t log_storage_write(esp_log_level_t level, const char* const message)
{
  /* Ensure message is valid */
  if (message == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Ensure mutex is created - create it on demand if not initialized yet */
  if (s_log_mutex == NULL) {
    /* Create mutex on first use if it doesn't exist yet */
    s_log_mutex = xSemaphoreCreateMutex();
    if (s_log_mutex == NULL) {
      ESP_LOGE(TAG, "Log storage mutex not initialized and creation failed!");
      return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGW(TAG, "Created log mutex on first write call - prefer calling log_init() first");
  }

  /* Acquire mutex to access buffer */
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(LOG_STORAGE_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to acquire mutex for log storage write");
    return ESP_ERR_TIMEOUT;
  }

  esp_err_t result = ESP_OK;

  /* Store log in buffer */
  if (s_log_buffer_index < CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE) {
    s_log_buffer[s_log_buffer_index].level     = level;
    s_log_buffer[s_log_buffer_index].timestamp = esp_timer_get_time();
    strlcpy(s_log_buffer[s_log_buffer_index].buffer,
            message,
            CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH);
    s_log_buffer[s_log_buffer_index].buffer[CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH - 1] =
      '\0'; /* Ensure null termination */
    s_log_buffer_index++;
  } else {
    /* Buffer is full */

#ifdef CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
    /* If SD is enabled, try flushing ONLY if storage is fully initialized and SD is available */
    if (atomic_load(&s_log_storage_initialized) && atomic_load(&s_sd_card_available)) {
      ESP_LOGW(TAG, "Log buffer full, attempting to flush...");
      /* We hold the mutex, call private flush directly */
      result = priv_flush_log_buffer(); /* This function handles rotation/write */

      if (result != ESP_OK || s_log_buffer_index >= CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE) {
        /* Flush failed or didn't clear space, drop oldest log */
        ESP_LOGE(TAG, "Buffer Overflow: SD flush failed/insufficient, dropping oldest log.");
        /* Shift buffer content */
        memmove(&s_log_buffer[0],
                &s_log_buffer[1],
                sizeof(log_entry_t) * (CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1));
        s_log_buffer_index =
          CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1; /* Make space for new log */
      }
    } else {
      /* SD not ready or storage not init, drop oldest log */
      ESP_LOGE(TAG, "Buffer Overflow: SD not available/initialized, dropping oldest log.");
      memmove(&s_log_buffer[0],
              &s_log_buffer[1],
              sizeof(log_entry_t) * (CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1));
      s_log_buffer_index = CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1;
    }
#else
    /* SD Disabled: Buffer is full, drop oldest log */
    ESP_LOGE(TAG, "Buffer Overflow: Log buffer full (SD disabled), dropping oldest log.");
    memmove(&s_log_buffer[0],
            &s_log_buffer[1],
            sizeof(log_entry_t) * (CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1));
    s_log_buffer_index = CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1;
#endif

    /* Store the current log in the now available last slot (or newly freed slot) */
    s_log_buffer[s_log_buffer_index].level     = level;
    s_log_buffer[s_log_buffer_index].timestamp = esp_timer_get_time();
    strlcpy(s_log_buffer[s_log_buffer_index].buffer,
            message,
            CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH);
    s_log_buffer[s_log_buffer_index].buffer[CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH - 1] =
      '\0';
    s_log_buffer_index++; /* Increment index for the newly added log */
  }

  /* If buffer is now full AND SD storage is fully ready, try to flush again (relevant if initial write filled the buffer) */
#ifdef CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  if (s_log_buffer_index >= CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE &&
      atomic_load(&s_log_storage_initialized) && atomic_load(&s_sd_card_available)) {
    /* Call private flush directly as we hold the mutex */
    esp_err_t flush_ret = priv_flush_log_buffer();
    if (flush_ret != ESP_OK) {
      result = flush_ret; /* Report flush error if it occurred */
    }
  }
#endif

  xSemaphoreGive(s_log_mutex);
  return result; /* Return ESP_OK or error from flush attempt */
}

esp_err_t log_storage_flush(void)
{
#ifndef CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  ESP_LOGI(TAG, "Flush called but SD logging is disabled.");
  return ESP_OK; /* Nothing to flush */
#else
  /* Check if storage component is fully initialized */
  if (!atomic_load(&s_log_storage_initialized)) {
    ESP_LOGW(TAG, "Flush called but log storage is not fully initialized.");
    return ESP_ERR_INVALID_STATE;
  }

  /* Check if SD card is available (atomically) */
  if (!atomic_load(&s_sd_card_available)) {
    ESP_LOGW(TAG, "Flush skipped: SD card not available.");
    return ESP_OK; /* Not an error, just can't flush now */
  }

  /* Ensure mutex is created */
  if (s_log_mutex == NULL) {
    /* Create mutex on demand if needed */
    s_log_mutex = xSemaphoreCreateMutex();
    if (s_log_mutex == NULL) {
      ESP_LOGE(TAG, "Flush Error: Log storage mutex not initialized and creation failed!");
      return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGW(TAG, "Created log mutex during flush - prefer calling log_init() first");
  }

  /* Acquire mutex */
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(LOG_STORAGE_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to acquire mutex for log flush");
    return ESP_ERR_TIMEOUT;
  }

  /* Call the internal flush function */
  esp_err_t ret = priv_flush_log_buffer();

  xSemaphoreGive(s_log_mutex);
  return ret;
#endif
}

esp_err_t log_storage_cleanup(void)
{
  ESP_LOGI(TAG, "Beginning log storage cleanup...");
  esp_err_t final_ret = ESP_OK;

  /* Atomically check and set initialized flag */
  bool was_initialized __attribute__((unused)) =
    atomic_exchange(&s_log_storage_initialized, false); /* Mark unused if SD disabled */

  /* If mutex wasn't created, there's nothing else to clean */
  if (s_log_mutex == NULL) {
    ESP_LOGW(TAG, "Cleanup: Mutex was not initialized.");
    atomic_store(&s_sd_card_available, false); /* Ensure flag is false */
    return ESP_OK;
  }

  /* Attempt to acquire the mutex with a generous timeout for cleanup */
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(LOG_STORAGE_MUTEX_TIMEOUT_MS * 2)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to acquire mutex during cleanup. Resources may leak!");
    /* Cannot safely proceed further without mutex, but try deleting it anyway */
    final_ret = ESP_ERR_TIMEOUT;
  } else {
    /* Mutex acquired */

#ifdef CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
    /* Attempt final flush only if it was fully initialized and SD might be available */
    if (was_initialized && atomic_load(&s_sd_card_available)) {
      ESP_LOGI(TAG, "Cleanup: Attempting final buffer flush.");
      esp_err_t flush_ret = priv_flush_log_buffer();
      if (flush_ret != ESP_OK) {
        ESP_LOGW(TAG, "Final flush failed during cleanup: %s", esp_err_to_name(flush_ret));
        if (final_ret == ESP_OK) {
          final_ret = flush_ret;
        }
      }
    }

    /* Reset file manager pointer and current log file path */
    s_file_manager = NULL;
    memset(s_current_log_file, 0, sizeof(s_current_log_file));
    s_sd_card_hal_instance = NULL; /* Clear HAL instance pointer */
#endif

    /* Clear log buffer state */
    s_log_buffer_index = 0;
    memset(s_log_buffer, 0, sizeof(s_log_buffer));

    /* Give the mutex back before deleting it */
    xSemaphoreGive(s_log_mutex);
  } /* End mutex handling block */

  /* Delete the mutex */
  if (s_log_mutex != NULL) {
    SemaphoreHandle_t tmp_mutex = s_log_mutex;
    s_log_mutex                 = NULL; /* Set to NULL before deleting to prevent double-free */
    vSemaphoreDelete(tmp_mutex);
  }

  /* Reset atomic SD availability flag */
  atomic_store(&s_sd_card_available, false);

  ESP_LOGI(TAG,
           "Log storage cleanup %s.",
           (final_ret == ESP_OK) ? "successful" : "completed with warnings/errors");
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

static inline int
priv_format_date_string(char* buffer, size_t buffer_size, const struct tm* const timeinfo)
{
  if (!buffer || buffer_size == 0 || !timeinfo) {
    return -1;
  }
  return snprintf(buffer,
                  buffer_size,
                  "%04d-%02d-%02d",
                  timeinfo->tm_year + 1900,
                  timeinfo->tm_mon + 1,
                  timeinfo->tm_mday);
}

#ifdef CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
/* Only define these if SD card is enabled */
static inline int priv_format_log_filepath(char*                  buffer,
                                           size_t                 buffer_size,
                                           const struct tm* const timeinfo,
                                           const char* const      extension)
{
  if (!buffer || buffer_size == 0 || !timeinfo || !extension) {
    return -1;
  }

  char date_str[CONFIG_PSTAR_KCONFIG_LOGGING_DATE_STRING_BUFFER_SIZE];
  int  date_written = priv_format_date_string(date_str, sizeof(date_str), timeinfo);
  if (date_written < 0 || (size_t)date_written >= sizeof(date_str)) {
    ESP_LOGE(TAG, "Date Format Error: Failed to format date string for log file path");
    return -1;
  }

  /* Path is relative to the mount point */
  return snprintf(buffer,
                  buffer_size,
                  "%s/%s/%04d-%02d-%02d_%02d-%02d-%02d%s",
                  CONFIG_PSTAR_KCONFIG_LOGGING_BASE_DIR,
                  date_str,
                  timeinfo->tm_year + 1900,
                  timeinfo->tm_mon + 1,
                  timeinfo->tm_mday,
                  timeinfo->tm_hour,
                  timeinfo->tm_min,
                  timeinfo->tm_sec,
                  extension);
}

static void priv_generate_log_file_path(char* file_path, size_t file_path_len)
{
  if (file_path == NULL || file_path_len == 0) {
    ESP_LOGE(TAG, "File Path Error: Invalid buffer provided for log file path generation");
    return;
  }

  struct tm timeinfo;
  time_t    now = time(NULL);
  localtime_r(&now, &timeinfo);

#ifdef CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED
  const char* const extension = CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSED_EXTENSION;
#else
  const char* const extension = ".log"; /* Use .log for uncompressed */
#endif
  int written = priv_format_log_filepath(file_path, file_path_len, &timeinfo, extension);

  if (written < 0 || (size_t)written >= file_path_len) {
    ESP_LOGE(TAG, "File Path Error: Failed to format log file path or buffer too small");
    file_path[0] = '\0'; /* Ensure null termination on error */
  }
}

static bool priv_check_log_rotation(void)
{
  /* Assumes SD card is available and storage is initialized */
  /* Assumes mutex is held */

  /* Check if a log file path exists */
  if (strlen(s_current_log_file) == 0) {
    ESP_LOGI(TAG, "Rotation needed: No current log file path.");
    return true; /* Need to generate the first file path */
  }

  /* Construct the full path for stat */
  char
    full_stat_path[CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH * 2]; /* Use Kconfig value */
  if (!s_sd_card_hal_instance || !s_sd_card_hal_instance->mount_path) {
    ESP_LOGE(TAG, "Rotation Check Error: SD HAL instance or mount path is NULL.");
    return false; /* Cannot check without mount path */
  }
  snprintf(full_stat_path,
           sizeof(full_stat_path),
           "%s/%s",
           s_sd_card_hal_instance->mount_path,
           s_current_log_file);
  full_stat_path[sizeof(full_stat_path) - 1] = '\0';

  /* Check file size */
  struct stat st;
  if (stat(full_stat_path, &st) != 0) {
    if (errno == ENOENT) {
      ESP_LOGW(TAG, "Rotation needed: Current log file '%s' not found.", full_stat_path);
      return true; /* File doesn't exist, need a new one */
    } else {
      ESP_LOGE(TAG,
               "Stat Error: Failed to stat current log file '%s': %s",
               full_stat_path,
               strerror(errno));
      return false; /* Avoid rotation if stat fails for other reasons */
    }
  }

  if (st.st_size >= PSTAR_LOGGING_MAX_FILE_SIZE) {
    ESP_LOGI(TAG,
             "Rotation needed: File size %ld >= max %d for %s",
             (long)st.st_size,
             PSTAR_LOGGING_MAX_FILE_SIZE,
             full_stat_path);
    return true;
  }

  /* Check if date has changed */
  struct tm timeinfo;
  time_t    now = time(NULL);
  localtime_r(&now, &timeinfo);

  char date_str[CONFIG_PSTAR_KCONFIG_LOGGING_DATE_STRING_BUFFER_SIZE];
  int  date_written = priv_format_date_string(date_str, sizeof(date_str), &timeinfo);
  if (date_written < 0 || (size_t)date_written >= sizeof(date_str)) {
    ESP_LOGE(TAG, "Date Format Error: Failed to format date string for rotation check");
    return false;
  }

  /* Extract date part from the current RELATIVE log file path's directory component */
  /* Expected relative format: logs/YYYY-MM-DD/YYYY-MM-DD_HH-MM-SS.ext */
  char        current_date_part[CONFIG_PSTAR_KCONFIG_LOGGING_DATE_STRING_BUFFER_SIZE] = {0};
  const char* dir_part_start =
    strchr(s_current_log_file, '/'); /* Find first slash after base dir ("logs/") */
  if (dir_part_start) {
    dir_part_start++; /* Move past the slash */
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
    ESP_LOGI(TAG,
             "Rotation needed: Date changed. Current: '%s', New: '%s'",
             current_date_part,
             date_str);
    return true;
  }

  return false; /* No rotation needed */
}

static esp_err_t priv_rotate_log_file(void)
{
  /* Assumes mutex is held */
  if (!priv_check_log_rotation()) {
    return ESP_OK; /* No rotation needed */
  }

  /* Generate new relative log file path */
  priv_generate_log_file_path(s_current_log_file, sizeof(s_current_log_file));

  if (strlen(s_current_log_file) == 0) {
    ESP_LOGE(TAG, "Log Rotation Error: Failed to generate new log file path during rotation");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Rotating to new log file: %s (relative)", s_current_log_file);
  return ESP_OK;
}

#ifdef CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED
static esp_err_t
priv_compress_data(const char* const input, size_t input_len, char* output, size_t* output_len)
{
  if (input == NULL || input_len == 0 || output == NULL || output_len == NULL || *output_len == 0) {
    ESP_LOGE(TAG, "Compression Error: Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  z_stream stream;
  memset(&stream, 0, sizeof(stream));

  int ret = deflateInit2(&stream,
                         Z_DEFAULT_COMPRESSION,
                         Z_DEFLATED,
                         PSTAR_LOGGING_ZLIB_GZIP_WINDOW_BITS,
                         CONFIG_PSTAR_KCONFIG_LOGGING_ZLIB_MEM_LEVEL,
                         Z_DEFAULT_STRATEGY);
  if (ret != Z_OK) {
    ESP_LOGE(TAG, "Zlib Init Failed: %d", ret);
    return ESP_FAIL;
  }

  stream.next_in   = (Bytef*)input;
  stream.avail_in  = input_len;
  stream.next_out  = (Bytef*)output;
  stream.avail_out = *output_len;

  ret = deflate(&stream, Z_FINISH);
  deflateEnd(&stream);

  if (ret != Z_STREAM_END) {
    ESP_LOGE(TAG,
             "Compression Failed: zlib error %d (%s)",
             ret,
             stream.msg ? stream.msg : "No msg");
    return ESP_FAIL;
  }

  *output_len = stream.total_out;
  return ESP_OK;
}
#endif /* CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED */

static esp_err_t priv_write_log_data(const char* const file_path_relative, const char* const data)
{
  /* Assumes mutex is held */
  /* Assumes SD card is available and storage is initialized */

  if (file_path_relative == NULL || data == NULL || s_file_manager == NULL) {
    ESP_LOGE(TAG,
             "Write Error: Invalid arguments (path=%p, data=%p, fm=%p)",
             file_path_relative,
             data,
             s_file_manager);
    return ESP_ERR_INVALID_ARG;
  }

#ifdef CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED
  size_t data_len = strlen(data);
  // Use a dynamically sized buffer for compression output, based on input size
  size_t compress_buf_size_needed = deflateBound(NULL, data_len); // zlib's worst-case estimate

  // --- FIX: More robust buffer size calculation ---
  size_t alloc_size = compress_buf_size_needed + 128; // Start with estimate + margin
  // Clamp allocation size between a minimum (e.g., Kconfig buffer) and a maximum reasonable limit
  const size_t min_alloc = CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_BUFFER_SIZE;
  const size_t max_alloc = PSTAR_LOGGING_MAX_COMPRESSION_INPUT_SIZE; // Use defined max

  if (alloc_size < min_alloc) {
    alloc_size = min_alloc;
  }
  if (alloc_size > max_alloc) {
    ESP_LOGW(
      TAG,
      "Compression Size Warning - Estimated compression buffer size %zu exceeds max %zu, clamping.",
      alloc_size,
      max_alloc);
    alloc_size = max_alloc;
  }
  // --- End FIX ---

  char* compress_buffer = NULL;
  compress_buffer       = malloc(alloc_size);
  if (compress_buffer == NULL) {
    ESP_LOGE(TAG, "Memory Error: Failed to allocate compression buffer (size %zu)", alloc_size);
    return ESP_ERR_NO_MEM;
  }

  size_t output_len = alloc_size; // Pass the actual allocated size

  esp_err_t ret = priv_compress_data(data, data_len, compress_buffer, &output_len);
  if (ret != ESP_OK) {
    // --- FIX: Ensure buffer is freed on error ---
    if (compress_buffer != NULL) {
      free(compress_buffer);
      compress_buffer = NULL; // Set to NULL after freeing
    }
    // --- End FIX ---
    return ret;
  }

  /* Write compressed data using the binary enqueue function */
  ret = file_write_binary_enqueue(s_file_manager, file_path_relative, compress_buffer, output_len);

  /* Safely free the buffer after use */
  if (compress_buffer != NULL) {
    free(compress_buffer);
    compress_buffer = NULL; // Set to NULL after freeing
  }

  return ret;

#else /* Compression disabled */
  /* Write uncompressed data using the standard enqueue function */
  return file_write_enqueue(s_file_manager, file_path_relative, data);
#endif
}

static esp_err_t priv_flush_log_buffer(void)
{
  /* Assumes mutex is held */
  /* Assumes SD card logging is enabled */

  if (s_log_buffer_index == 0) {
    return ESP_OK; /* Nothing to flush */
  }

  /* Check initialization and availability (redundant check, but safe) */
  if (!atomic_load(&s_log_storage_initialized) || !atomic_load(&s_sd_card_available)) {
    ESP_LOGW(TAG, "Flush skipped: Storage not initialized or SD card not available.");
    /* Don't clear buffer if we can't write */
    return ESP_FAIL;
  }

  /* Ensure we have a valid log file path (handles rotation) */
  if (priv_rotate_log_file() != ESP_OK) {
    ESP_LOGE(TAG, "Flush Error: Failed to rotate log file before flushing.");
    return ESP_FAIL;
  }
  if (strlen(s_current_log_file) == 0) {
    ESP_LOGE(TAG, "Flush Error: Invalid log file path after rotation attempt.");
    return ESP_FAIL;
  }

  esp_err_t write_ret     = ESP_OK;
  uint32_t  flushed_count = 0;

  /* Process log entries one by one */
  for (uint32_t i = 0; i < s_log_buffer_index; i++) {
    /* Prepare the log entry data (already formatted in buffer) */
    /* Add newline for text logs */
    char entry_with_newline[CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH + 2];
    snprintf(entry_with_newline, sizeof(entry_with_newline), "%s\n", s_log_buffer[i].buffer);
    entry_with_newline[sizeof(entry_with_newline) - 1] = '\0'; /* Ensure null termination */

    /* Write the single log entry (will handle compression internally if enabled) */
    write_ret = priv_write_log_data(s_current_log_file, entry_with_newline);

    if (write_ret != ESP_OK) {
      ESP_LOGE(TAG,
               "Flush Write Failed: Failed to write/enqueue log entry %lu: %s",
               (unsigned long)i, // Use %lu for uint32_t
               esp_err_to_name(write_ret));
      /* Do NOT reset buffer index if write failed, stop flushing */
      /* Shift remaining logs? Or just leave them for next flush? Leave for now. */
      if (i > 0) {
        /* Shift the remaining entries down */
        memmove(&s_log_buffer[0], &s_log_buffer[i], sizeof(log_entry_t) * (s_log_buffer_index - i));
        s_log_buffer_index -= i; /* Update index */
        ESP_LOGW(TAG, "Partial Flush: Flushed %lu entries before error.", (unsigned long)i);
      } else {
        /* Error on the very first entry */
        ESP_LOGE(TAG, "Flush Error: Failed on first log entry, no entries flushed.");
        // Keep buffer full? Or clear? Let's keep it for next attempt.
        // s_log_buffer_index = CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE;
      }
      return write_ret; /* Return the error */
    }
    flushed_count++;
  }

  /* Reset buffer index ONLY if all writes were successful */
  ESP_LOGI(TAG, "Successfully flushed %lu log entries.", (unsigned long)flushed_count);
  s_log_buffer_index = 0;
  memset(s_log_buffer, 0, sizeof(s_log_buffer)); /* Clear buffer contents */

  return ESP_OK;
}

#endif /* CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED */