/* components/pstar_logging/log_storage.c */

#include "pstar_log_storage.h"
#include "pstar_log_handler.h"
#include "esp_system.h"
#include "pstar_log_macros.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <unistd.h>
#if CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED
#include <zlib.h>
#endif
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdatomic.h>

static const char* TAG = "Log Storage";

/* Constants ******************************************************************/

#define LOG_STORAGE_MUTEX_TIMEOUT_MS (1000)

/* Globals (Static) ***********************************************************/

static log_entry_t           s_log_buffer[CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE] = {0};   /* Buffer to store logs when SD card is not available */
static uint32_t              s_log_buffer_index                                     = 0;     /* Current index in the buffer */
static bool                  s_log_storage_initialized                              = false; /* Flag to track initialization status */
static char                  s_current_log_file[PSTAR_LOGGING_MAX_FILE_PATH_LENGTH + CONFIG_PSTAR_KCONFIG_LOGGING_DATE_STRING_BUFFER_SIZE + 64]  = {0};   /* Current log file path */
static SemaphoreHandle_t     s_log_mutex                                            = NULL;  /* Mutex for thread-safe access */
static _Atomic bool          s_sd_card_available                                    = false; /* Flag indicating if SD card is available (Made Atomic)*/
static file_write_manager_t* s_file_manager                                         = NULL;  /* File write manager instance */

/* Private Helper Functions ***************************************************/

/**
 * @brief Converts microseconds timestamp to seconds
 *
 * @param[in] timestamp_us Timestamp in microseconds
 * @return Timestamp in seconds
 */
static inline time_t priv_timestamp_us_to_seconds(uint64_t timestamp_us)
{
  return timestamp_us / 1000000;
}

/**
 * @brief Extracts milliseconds from microseconds timestamp
 *
 * @param[in] timestamp_us Timestamp in microseconds
 * @return Milliseconds component (0-999)
 */
static inline uint64_t priv_timestamp_us_to_milliseconds(uint64_t timestamp_us)
{
  return (timestamp_us % 1000000) / 1000;
}

/**
 * @brief Formats a date string in YYYY-MM-DD format
 *
 * @param[out] buffer      Buffer to store formatted date
 * @param[in]  buffer_size Size of the buffer
 * @param[in]  timeinfo    Time information structure
 * @return Number of characters written (excluding null terminator) or -1 on error
 */
static inline int priv_format_date_string(char*                  buffer,
                                          size_t                 buffer_size,
                                          const struct tm *const timeinfo)
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

/**
 * @brief Formats a log entry with timestamp, level, and message
 *
 * @param[out] buffer       Output buffer to store formatted string
 * @param[in]  buffer_size  Size of output buffer
 * @param[in]  timeinfo     Time information structure
 * @param[in]  milliseconds Millisecond component of timestamp
 * @param[in]  level_str    String representation of log level
 * @param[in]  message      The log message
 * @return Number of characters written (excluding null terminator) or -1 on error
 */
static inline int priv_format_log_entry(char*                  buffer,
                                        size_t                 buffer_size,
                                        const struct tm* const timeinfo,
                                        uint64_t               milliseconds,
                                        const char* const      level_str,
                                        const char* const      message)
{
  if (!buffer || buffer_size == 0 || !timeinfo || !level_str || !message) {
    return -1;
  }

  return snprintf(buffer,
                  buffer_size,
                  "%04d-%02d-%02d %02d:%02d:%02d.%03llu [%s] %s",
                  timeinfo->tm_year + 1900,
                  timeinfo->tm_mon + 1,
                  timeinfo->tm_mday,
                  timeinfo->tm_hour,
                  timeinfo->tm_min,
                  timeinfo->tm_sec,
                  milliseconds,
                  level_str,
                  message);
}

/**
 * @brief Formats a log filepath based on date and time
 *
 * @param[out] buffer      Output buffer to store formatted path
 * @param[in]  buffer_size Size of output buffer
 * @param[in]  timeinfo    Time information structure
 * @param[in]  extension   File extension to use
 * @return Number of characters written (excluding null terminator) or -1 on error
 */
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
    log_error(TAG, "Date Format Error", "Failed to format date string for log file path");
    return -1;
  }

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

/* Private Functions **********************************************************/

/**
 * @brief Converts a log level to its string representation
 *
 * @param[in] level The log level
 * @return Const string representing the log level
 */
static const char *priv_log_level_to_string(esp_log_level_t level)
{
  switch (level) {
    case ESP_LOG_ERROR:   return "ERROR";
    case ESP_LOG_WARN:    return "WARN";
    case ESP_LOG_INFO:    return "INFO";
    case ESP_LOG_DEBUG:   return "DEBUG";
    case ESP_LOG_VERBOSE: return "VERBOSE";
    default:              return "UNKNOWN";
  }
}

/**
 * @brief Generates a log file path based on the current date and time
 *
 * @param[out] file_path     Buffer to store the file path
 * @param[in]  file_path_len Length of the buffer
 */
static void priv_generate_log_file_path(char*  file_path,
                                        size_t file_path_len)
{
#if !CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  log_warn(TAG,
           "SD Card Disabled",
           "Cannot generate log file path: SD card support is disabled");
  if (file_path != NULL && file_path_len > 0) {
    file_path[0] = '\0';
  }
  return;
#else
  if (file_path == NULL || file_path_len == 0) {
    log_error(TAG, 
              "File Path Error", 
              "Invalid buffer provided for log file path generation");
    return;
  }

  struct tm timeinfo;
  time_t    now = time(NULL);
  localtime_r(&now, &timeinfo);

  const char* const extension = CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED ? 
                                CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSED_EXTENSION : 
                                ".txt";
  int               written   = priv_format_log_filepath(file_path, 
                                                         file_path_len, 
                                                         &timeinfo, 
                                                         extension);

  if (written < 0 || (size_t)written >= file_path_len) {
    log_error(TAG, 
              "File Path Error", 
              "Failed to format log file path or buffer too small");
    file_path[0] = '\0'; /* Ensure null termination on error */
  }
#endif
}

/**
 * @brief Checks if log rotation is needed
 *
 * @return true if rotation is needed, false otherwise
 */
static bool priv_check_log_rotation(void)
{
#if !CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  return false;
#else
  /* If no current log file, no need to rotate */
  if (strlen(s_current_log_file) == 0) {
    return true; /* No current file, need to generate one */
  }

  /* Check file size */
  struct stat st;
  if (stat(s_current_log_file, &st) != 0) {
    /* File doesn't exist (maybe deleted?), treat as needing rotation to create a new one */
    if (errno == ENOENT) {
      log_warn(TAG, 
               "Log Rotation Check", 
               "Current log file '%s' not found, rotating.", 
               s_current_log_file);
      return true;
    } else {
      log_error(TAG, 
                "Stat Error", 
                "Failed to stat current log file '%s': %s", 
                s_current_log_file, 
                strerror(errno));
      return false; /* Avoid rotation if stat fails for other reasons */
    }
  }

  /* Check if file size exceeds the maximum */
  if (st.st_size >= PSTAR_LOGGING_MAX_FILE_SIZE) {
    return true;
  }

  /* Check if date has changed */
  struct tm timeinfo;
  time_t    now = time(NULL);
  localtime_r(&now, &timeinfo);

  char date_str[CONFIG_PSTAR_KCONFIG_LOGGING_DATE_STRING_BUFFER_SIZE];
  int  date_written = priv_format_date_string(date_str, sizeof(date_str), &timeinfo);
  if (date_written < 0 || (size_t)date_written >= sizeof(date_str)) {
    log_error(TAG, "Date Format Error", "Failed to format date string for rotation check");
    return false; /* Avoid rotation if formatting fails */
  }

  /* Extract date from current log file path's directory component */
  char current_date_part[CONFIG_PSTAR_KCONFIG_LOGGING_DATE_STRING_BUFFER_SIZE] = {0};
  /* Expected format: logs/YYYY-MM-DD/YYYY-MM-DD_HH-MM-SS.ext */
  /* Find the first slash after the base directory */
  const char* dir_part_start = strstr(s_current_log_file, 
                                      CONFIG_PSTAR_KCONFIG_LOGGING_BASE_DIR);
  if (dir_part_start) {
    dir_part_start += strlen(CONFIG_PSTAR_KCONFIG_LOGGING_BASE_DIR);
    if (*dir_part_start == '/') dir_part_start++; /* Skip the first slash */
    const char* dir_part_end = strchr(dir_part_start, '/');
    if (dir_part_end) {
      size_t len = dir_part_end - dir_part_start;
      if (len < sizeof(current_date_part)) {
        strncpy(current_date_part, dir_part_start, len);
        current_date_part[len] = '\0';
      }
    }
  }

  /* If we couldn't extract the date part or it doesn't match, rotate */
  if (strlen(current_date_part) == 0 || strcmp(current_date_part, date_str) != 0) {
    log_info(TAG, 
             "Log Rotation", 
             "Date changed. Current: '%s', New: '%s'", 
             current_date_part, 
             date_str);
    return true;
  }

  return false;
#endif
}

/**
 * @brief Rotates the log file if needed
 *
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_rotate_log_file(void)
{
#if !CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  log_warn(TAG,
           "SD Card Disabled",
           "Cannot rotate log file: SD card support is disabled");
  return ESP_ERR_NOT_SUPPORTED;
#else
  if (!priv_check_log_rotation()) {
    return ESP_OK; /* No rotation needed */
  }

  /* Generate new log file path */
  priv_generate_log_file_path(s_current_log_file, sizeof(s_current_log_file));
  /* Check if path generation failed */
  if (strlen(s_current_log_file) == 0) {
      log_error(TAG, "Log Rotation Error", "Failed to generate new log file path during rotation");
      return ESP_FAIL;
  }

  log_info(TAG,
           "Log Rotation",
           "Rotating to new log file: %s",
           s_current_log_file);

  return ESP_OK;
#endif
}

/**
 * @brief Compresses data using zlib
 *
 * @param[in]     input      Input data to compress
 * @param[in]     input_len  Length of input data
 * @param[out]    output     Buffer to store compressed data
 * @param[in,out] output_len Size of output buffer on input, size of compressed data on output
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_compress_data(const char* const input,
                                    size_t            input_len,
                                    char*             output,
                                    size_t*           output_len)
{
#if !CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED || !CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  log_warn(TAG,
           "Compression Disabled",
           "Cannot compress data: compression or SD card support is disabled");
  return ESP_ERR_NOT_SUPPORTED;
#else
  if (input == NULL || input_len == 0 || output == NULL || output_len == NULL || 
      *output_len == 0) {
    log_error(TAG, "Compression Error", "Invalid arguments for compression");
    return ESP_ERR_INVALID_ARG;
  }

  z_stream stream;
  memset(&stream, 0, sizeof(stream));

  /* Initialize zlib for gzip compression */
  int ret = deflateInit2(&stream,
                         Z_DEFAULT_COMPRESSION,
                         Z_DEFLATED,
                         PSTAR_LOGGING_ZLIB_GZIP_WINDOW_BITS,
                         CONFIG_PSTAR_KCONFIG_LOGGING_ZLIB_MEM_LEVEL,
                         Z_DEFAULT_STRATEGY);
  if (ret != Z_OK) {
    log_error(TAG,
              "Zlib Init Failed",
              "Failed to initialize zlib: %d",
              ret);
    return ESP_FAIL;
  }

  /* Set up input and output buffers */
  stream.next_in   = (Bytef*)input;
  stream.avail_in  = input_len;
  stream.next_out  = (Bytef*)output;
  stream.avail_out = *output_len;

  /* Compress data */
  ret = deflate(&stream, Z_FINISH);

  /* Clean up */
  deflateEnd(&stream);

  if (ret != Z_STREAM_END) {
    /* Log specific zlib error messages if possible */
    log_error(TAG,
              "Compression Failed",
              "Failed to compress data: zlib error %d (%s)",
              ret, 
              stream.msg ? stream.msg : "No message");
    return ESP_FAIL;
  }

  /* Update output length */
  *output_len = stream.total_out;

  return ESP_OK;
#endif
}

/**
 * @brief Writes log data to a file
 *
 * @param[in] file_path Path to the log file
 * @param[in] data      Data to write
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_write_log_data(const char* const file_path,
                                     const char* const data)
{
#if !CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  log_warn(TAG,
           "SD Card Disabled",
           "Cannot write log data: SD card support is disabled");
  return ESP_ERR_NOT_SUPPORTED;
#else
  if (file_path == NULL || data == NULL || s_file_manager == NULL) {
    log_error(TAG, 
              "Write Error", 
              "Invalid arguments for writing log data (path=%p, data=%p, fm=%p)", 
              file_path, 
              data, 
              s_file_manager);
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if compression is enabled */
  if (CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED) {
    /* Compress data before writing */
    /* Check potential overflow for compression buffer size */
    /* XXX: This is using estimations */
    size_t data_len          = strlen(data);
    size_t compress_buf_size = CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_BUFFER_SIZE;
    /* Estimate required size, zlib might slightly increase size for small inputs */
    size_t required_size = data_len + (data_len / 1000) + 18; /* Rough estimate */
    if (required_size > compress_buf_size) {
      compress_buf_size = required_size; // Increase buffer if estimate is larger
      /* Add a cap to prevent excessive allocation */
      if (compress_buf_size > PSTAR_LOGGING_MAX_COMPRESSION_INPUT_SIZE) {
        log_error(TAG, 
                  "Memory Error", 
                  "Calculated compression buffer size (%zu) exceeds limit (%d)", 
                  compress_buf_size, 
                  PSTAR_LOGGING_MAX_COMPRESSION_INPUT_SIZE);
        return ESP_ERR_NO_MEM;
      }
    }

    char *compress_buffer = malloc(compress_buf_size);
    if (compress_buffer == NULL) {
      log_error(TAG,
                "Memory Error",
                "Failed to allocate compression buffer (size %zu)", 
                compress_buf_size);
      return ESP_ERR_NO_MEM;
    }

    size_t output_len = compress_buf_size; /* Pass the actual allocated size */

    esp_err_t ret = priv_compress_data(data,
                                       data_len,
                                       compress_buffer,
                                       &output_len);
    if (ret != ESP_OK) {
      free(compress_buffer);
      return ret;
    }

    /* Write compressed data to file */
    ret = file_write_binary_enqueue(s_file_manager, 
                                    file_path, 
                                    compress_buffer, 
                                    output_len);
    free(compress_buffer); /* Free buffer regardless of enqueue result */
    return ret;
  } else {
    /* Write uncompressed data to file */
    return file_write_enqueue(s_file_manager, file_path, data);
  }
#endif
}

/**
 * @brief Flushes the log buffer to disk
 *
 * @note Refactoring Opportunity: This function is quite long and complex.
 *       Consider breaking down the compression logic and file writing
 *       logic into smaller helper functions.
 *
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_flush_log_buffer(void)
{
#if !CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  log_warn(TAG,
           "SD Card Disabled",
           "Cannot flush log buffer: SD card support is disabled");
  /* Clear the buffer to avoid memory issues */
  s_log_buffer_index = 0;
  return ESP_ERR_NOT_SUPPORTED;
#else
  if (s_log_buffer_index == 0) {
    return ESP_OK; /* Nothing to flush */
  }

  /* Use atomic read for SD card availability */
  if (!atomic_load(&s_sd_card_available)) {
    log_warn(TAG,
             "Flush Skip",
             "SD card not available, keeping %lu logs in buffer",
             s_log_buffer_index);
    return ESP_FAIL; /* Return failure if SD card is not available */
  }

  /* Ensure we have a valid log file */
  if (priv_rotate_log_file() != ESP_OK) {
    log_error(TAG, "Flush Error", "Failed to rotate log file before flushing");
    return ESP_FAIL;
  }
  /* Check if log file path is valid after rotation attempt */
  if (strlen(s_current_log_file) == 0) {
    log_error(TAG, "Flush Error", "Invalid log file path after rotation attempt");
    return ESP_FAIL;
  }

  /* If compression is enabled, we'll collect all logs into a single buffer first */
  char*     all_logs   = NULL;
  size_t    total_size = 0;
  esp_err_t final_ret  = ESP_OK;

  if (CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED) {
    /* Calculate total size needed */
    for (uint32_t i = 0; i < s_log_buffer_index; i++) {
      /* Ensure level_str is valid */
      const char* level_str = priv_log_level_to_string(s_log_buffer[i].level);
      if (!level_str) {
        level_str = "UNKNOWN";
      }

      /* Convert timestamp to time components */
      time_t    log_time = priv_timestamp_us_to_seconds(s_log_buffer[i].timestamp);
      struct tm timeinfo;
      localtime_r(&log_time, &timeinfo);

      /* Get milliseconds for formatting */
      uint64_t milliseconds = priv_timestamp_us_to_milliseconds(s_log_buffer[i].timestamp);

      /* Estimate size needed for this entry */
      size_t entry_len = snprintf(NULL, 
                                  0, 
                                  "%04d-%02d-%02d %02d:%02d:%02d.%03llu [%s] %s\n",
                                  timeinfo.tm_year + 1900, 
                                  timeinfo.tm_mon + 1, 
                                  timeinfo.tm_mday,
                                  timeinfo.tm_hour, 
                                  timeinfo.tm_min, 
                                  timeinfo.tm_sec,
                                  milliseconds, 
                                  level_str, 
                                  s_log_buffer[i].buffer);

      /* Check for snprintf error or unrealistic size */
      if (entry_len <= 0 || entry_len > (CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH * 2)) {
        log_error(TAG, "Buffer Size Error", "Invalid calculated entry length: %zd", entry_len);
        final_ret = ESP_FAIL; /* Mark failure but continue to potentially flush remaining buffer */
        continue; /* Skip this entry */
      }

      /* Check for potential overflow */
      if (SIZE_MAX - entry_len < total_size) {
        log_error(TAG,
                  "Buffer Error",
                  "Log buffer size would overflow size_t");
        /* FIXME: Decide how to handle - maybe flush what we have so far? */
        /* For now, return error. A better solution might flush in chunks. */
        return ESP_ERR_INVALID_SIZE;
      }

      total_size += entry_len;
    }

    /* Add 1 for null terminator */
    if (SIZE_MAX - 1 < total_size) {
      log_error(TAG,
                "Buffer Error",
                "Log buffer size would overflow with null terminator");
      return ESP_ERR_INVALID_SIZE;
    }
    total_size += 1;

    /* Check against a reasonable maximum size before allocating */
    if (total_size > PSTAR_LOGGING_MAX_COMPRESSION_INPUT_SIZE) {
      log_error(TAG, 
                "Buffer Error", 
                "Total log data size (%zu) exceeds max limit (%d) for compression buffer",
                total_size, 
                PSTAR_LOGGING_MAX_COMPRESSION_INPUT_SIZE);
      /* FIXME: Strategy - Flush uncompressed in chunks instead? */
      /* For now, return error. A better solution is needed. */
      return ESP_ERR_NO_MEM;
    }

    /* Allocate buffer for all logs */
    all_logs = malloc(total_size);
    if (!all_logs) {
      log_error(TAG,
                "Memory Error",
                "Failed to allocate buffer for log compression (size: %zu)",
                total_size);
      return ESP_ERR_NO_MEM;
    }
    all_logs[0] = '\0'; /* Ensure buffer starts empty */

    /* Reset buffer position */
    size_t pos = 0;

    /* Collect all logs into the buffer */
    for (uint32_t i = 0; i < s_log_buffer_index; i++) {
      const char* level_str = priv_log_level_to_string(s_log_buffer[i].level);
      if (!level_str) {
        level_str = "UNKNOWN";
      }

      /* Convert timestamp to time components */
      time_t    log_time = priv_timestamp_us_to_seconds(s_log_buffer[i].timestamp);
      struct tm timeinfo;
      localtime_r(&log_time, &timeinfo);

      /* Get milliseconds for formatting */
      uint64_t milliseconds = priv_timestamp_us_to_milliseconds(s_log_buffer[i].timestamp);

      /* Format the timestamp and log entry - ensure there's enough space */
      size_t remaining_space = total_size - pos;
      if (remaining_space <= 1) { /* Need space for at least one char + null */
        log_error(TAG,
                  "Buffer Error",
                  "Insufficient space remaining in buffer during formatting");
        final_ret = ESP_FAIL; /* Mark as failed, but continue to write what we have */
        break;                /* Stop adding entries */
      }

      int written = priv_format_log_entry(all_logs + pos,
                                          remaining_space,
                                          &timeinfo,
                                          milliseconds,
                                          level_str,
                                          s_log_buffer[i].buffer);

      if (written < 0 || (size_t)written >= remaining_space - 1) { /* Leave space for newline */
        log_error(TAG,
                  "Format Error",
                  "Failed to format log entry or insufficient buffer space (written: %d, remaining: %zu)",
                  written, remaining_space);
        final_ret = ESP_FAIL; /* Mark as failed, continue */
        break;                /* Stop adding entries */
      }

      pos += written;

      /* Add newline if there's space */
      if (pos < total_size - 1) {
        all_logs[pos++] = '\n';
        all_logs[pos]   = '\0'; /* Keep it null-terminated during build */
      } else {
        log_warn(TAG,
                 "Format Warning",
                 "No space for newline in buffer");
        final_ret = ESP_FAIL; /* Mark as failed */
        break;                /* Stop adding entries */
      }
    }

    /* Ensure null termination */
    if (pos < total_size) {
      all_logs[pos] = '\0';
    } else if (total_size > 0) {
      all_logs[total_size - 1] = '\0'; /* Force termination */
      log_warn(TAG,
               "Format Warning",
               "Buffer filled completely, possible truncation");
      final_ret = ESP_FAIL;
    }


    /* Write all logs at once (only if buffer is not empty) */
    if (pos > 0) {
      esp_err_t ret = priv_write_log_data(s_current_log_file, all_logs);
      if (ret != ESP_OK) {
        log_error(TAG,
                  "Write Failed",
                  "Failed to write compressed logs: %s",
                  esp_err_to_name(ret));
        final_ret = ret; /* Use the specific write error */
      }
    } else if (final_ret == ESP_OK) {
      /* If pos is 0 but no error was previously recorded, maybe all entries failed formatting? */
      log_warn(TAG, "Flush Warning", "No log data formatted for compressed write.");
    }

    /* free buffer */
    if (all_logs != NULL) {
      free(all_logs);
      all_logs = NULL;
    }
  } else {
    /* No compression, write each log entry individually */
    for (uint32_t i = 0; i < s_log_buffer_index; i++) {
      const char* level_str = priv_log_level_to_string(s_log_buffer[i].level);
      if (!level_str) {
        level_str = "UNKNOWN";
      }

      /* Convert timestamp to time components */
      time_t    log_time = priv_timestamp_us_to_seconds(s_log_buffer[i].timestamp);
      struct tm timeinfo;
      localtime_r(&log_time, &timeinfo);

      uint64_t milliseconds = priv_timestamp_us_to_milliseconds(s_log_buffer[i].timestamp);

      /* Format the timestamp and log entry */
      char formatted_log[PSTAR_LOGGING_MAX_FORMATTED_ENTRY_LENGTH];
      int  written = priv_format_log_entry(formatted_log,
                                           sizeof(formatted_log),
                                           &timeinfo,
                                           milliseconds,
                                           level_str,
                                           s_log_buffer[i].buffer);

      if (written < 0 || (size_t)written >= sizeof(formatted_log)) {
        log_error(TAG,
                  "Format Error",
                  "Failed to format log entry or buffer too small (written: %d, size: %zu)",
                  written, 
                  sizeof(formatted_log));
        final_ret = ESP_FAIL; /* Mark failure but continue */
        continue;             /* Skip this entry */
      }

      /* Enqueue the log for writing */
      esp_err_t ret = file_write_enqueue(s_file_manager, s_current_log_file, formatted_log);
      if (ret != ESP_OK) {
        log_error(TAG,
                  "Write Failed",
                  "Failed to enqueue log for writing: %s",
                  esp_err_to_name(ret));
        final_ret = ret; /* Use specific write error, continue */
      }
    }
  }

  /* Reset buffer index only if flush was generally successful
   * (or partially successful - avoid losing buffer if write failed completely) */
  if (final_ret == ESP_OK) {
    s_log_buffer_index = 0;
    log_info(TAG,
             "Buffer Flushed",
             "Successfully flushed log buffer to SD card");
  } else {
    log_error(TAG,
              "Flush Error",
              "Log buffer flush completed with errors, buffer NOT cleared.");
  }

  return final_ret;
#endif
}

/* Public Functions ***********************************************************/

esp_err_t log_storage_init(file_write_manager_t* manager, sd_card_hal_t* sd_card)
{
#if !CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  log_warn(TAG,
           "SD Card Disabled",
           "Initializing log storage with limited functionality (SD card support is disabled)");

  /* Create mutex for thread-safe access */
  s_log_mutex = xSemaphoreCreateMutex();
  if (s_log_mutex == NULL) {
    log_error(TAG, "Mutex Failed", "Failed to create mutex");
    return ESP_FAIL;
  }

  s_log_storage_initialized = true;
  atomic_store(&s_sd_card_available, false); /* SD not available */
  log_info(TAG,
           "Init Success",
           "Log storage initialized with limited functionality (no SD card)");

  return ESP_OK;
#else
  if (manager == NULL || sd_card == NULL) {
    log_error(TAG,
              "Init Error",
              "Invalid arguments: manager or sd_card is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(TAG, "Init", "Initializing log storage");

  if (s_log_storage_initialized) {
    log_warn(TAG, "Already Init", "Log storage already initialized");
    return ESP_OK;
  }

  /* Save the file manager instance */
  s_file_manager = manager;

  /* Create mutex for thread-safe access */
  s_log_mutex = xSemaphoreCreateMutex();
  if (s_log_mutex == NULL) {
    log_error(TAG, "Mutex Failed", "Failed to create mutex");
    return ESP_FAIL;
  }

  /* Initialize SD card availability status based on current state from HAL */
  atomic_store(&s_sd_card_available, sd_card_is_available(sd_card));

  /* Register for SD card availability notifications */
  esp_err_t reg_err = sd_card_register_availability_callback(sd_card, 
                                                             log_storage_set_sd_available);
  if (reg_err != ESP_OK) {
    log_error(TAG, 
              "Callback Error", 
              "Failed to register SD availability callback: %s", 
              esp_err_to_name(reg_err));
    vSemaphoreDelete(s_log_mutex);
    s_log_mutex = NULL;
    return reg_err;
  }


  s_log_storage_initialized = true;
  log_info(TAG,
           "Init Success",
           "Log storage initialized successfully (SD Available: %s)",
           atomic_load(&s_sd_card_available) ? "Yes" : "No");

  return ESP_OK;
#endif
}

/**
 * @brief Callback function for SD card availability changes
 *
 * This function is called by the SD card HAL when the SD card availability changes.
 * It updates the internal state and flushes the log buffer if the SD card becomes available.
 *
 * @param available Whether the SD card is available
 */
void log_storage_set_sd_available(bool available)
{
#if !CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  log_warn(TAG,
           "SD Card Disabled",
           "SD card availability change ignored: SD card support is disabled");
  return;
#else
  if (!s_log_storage_initialized) {
    return;
  }

  /* Use atomic store for thread safety without needing mutex here */
  bool old_value = atomic_exchange(&s_sd_card_available, available);

  /* Only proceed if the state actually changed */
  if (old_value == available) {
    return;
  }

  if (available) {
    /* SD card became available, try to flush buffer (take mutex inside flush) */
    log_info(TAG,
             "SD Available",
             "SD card became available, attempting to flush buffered logs");
    /* Attempt flush - it will handle mutex internally */
    log_storage_flush();
  } else {
    log_warn(TAG,
             "SD Unavailable",
             "SD card became unavailable, logs will be buffered");
  }

#endif
}

esp_err_t log_storage_write(esp_log_level_t  level,
                           const char* const message)
{
  if (!s_log_storage_initialized) {
    ESP_LOGW(TAG, "log_storage_write: Logger not initialized");
    return ESP_FAIL;
  }

  if (message == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Increased mutex timeout */
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(LOG_STORAGE_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    log_error(TAG,
              "Mutex Error",
              "Failed to acquire mutex for log write");
    return ESP_ERR_TIMEOUT;
  }

  /* Store log in buffer */
  if (s_log_buffer_index < CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE) {
    s_log_buffer[s_log_buffer_index].level     = level;
    s_log_buffer[s_log_buffer_index].timestamp = esp_timer_get_time();

    strlcpy(s_log_buffer[s_log_buffer_index].buffer,
            message,
            CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH);
    /* Ensure null termination even if strlcpy truncated (shouldn't happen if buffer sizes match) */
    s_log_buffer[s_log_buffer_index].buffer[CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH - 1] = '\0';
    s_log_buffer_index++;
  } else {
    /* Buffer is full */
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
    log_warn(TAG, "Buffer Full", "Log buffer full, forcing flush");
    /* Attempt to flush - function handles SD availability check & mutex internally (but we hold it here) */
    esp_err_t flush_ret = priv_flush_log_buffer();
    if (flush_ret != ESP_OK || s_log_buffer_index >= CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE) {
      /* If flush failed or didn't clear enough space, drop the oldest log */
      log_error(TAG,
                "Buffer Overflow",
                "Log buffer full, SD flush failed/insufficient, dropping oldest log");

      /* Move everything one slot down */
      memmove(&s_log_buffer[0], 
              &s_log_buffer[1], 
              sizeof(log_entry_t) * (CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1));

        /* Keep the buffer index at max-1 to make space for the new log */
        s_log_buffer_index = CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1;
    }
#else
    /* When SD card is disabled, drop oldest log */
    log_error(TAG,
             "Buffer Overflow",
             "Log buffer full and SD card disabled, dropping oldest log");

    /* Move everything one slot down */
    memmove(&s_log_buffer[0], 
            &s_log_buffer[1], 
            sizeof(log_entry_t) * (CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1));

    /* Keep the buffer index at max-1 */
    s_log_buffer_index = CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE - 1;
#endif

    /* Store the current log in the now available last slot */
    s_log_buffer[s_log_buffer_index].level     = level;
    s_log_buffer[s_log_buffer_index].timestamp = esp_timer_get_time();

    strlcpy(s_log_buffer[s_log_buffer_index].buffer,
            message,
            CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH);
    s_log_buffer[s_log_buffer_index].buffer[CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH - 1] = '\0';
    s_log_buffer_index++;
  }

  /* If buffer is now full AND SD card is available, try to flush */
  /* Use atomic read for SD card availability */
  if (s_log_buffer_index >= CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE && 
      atomic_load(&s_sd_card_available)) {
    priv_flush_log_buffer();
  }

  xSemaphoreGive(s_log_mutex);
  return ESP_OK;
}

esp_err_t log_storage_flush(void)
{
  if (!s_log_storage_initialized) {
    log_error(TAG, "Flush Error", "Log storage not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  // Fix: Increased mutex timeout
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(LOG_STORAGE_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    log_error(TAG,
              "Mutex Error",
              "Failed to acquire mutex for log flush");
    return ESP_ERR_TIMEOUT;
  }

#if !CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  log_warn(TAG,
           "SD Card Disabled",
           "Cannot flush log buffer: SD card support is disabled");
  xSemaphoreGive(s_log_mutex);
  return ESP_ERR_NOT_SUPPORTED;
#else
  /* Flush function now checks atomic flag internally */
  /* Avoid logging success/failure here as priv_flush_log_buffer logs details */
  esp_err_t ret = priv_flush_log_buffer();
  xSemaphoreGive(s_log_mutex);
  return ret;
#endif
}

esp_err_t log_storage_set_compression(bool enabled)
{
  if (!s_log_storage_initialized) {
    log_error(TAG, "Config Error", "Log storage not initialized");
    return ESP_FAIL;
  }

  /* Cannot modify compression setting at runtime, controlled by Kconfig */
  log_warn(TAG,
           "Compression Config",
           "Compression settings controlled by Kconfig, cannot be changed at runtime");
  return ESP_OK;
}

bool log_storage_is_compression_enabled(void)
{
#if !CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED || !CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED
  return false;
#else
  return true;
#endif
}

esp_err_t log_storage_cleanup(void)
{
  log_info(TAG, "Cleanup Start", "Beginning log storage cleanup");
  esp_err_t ret = ESP_OK;

  /* Ensure mutex exists before trying to take it */
  if (s_log_mutex == NULL) {
    log_warn(TAG, "Cleanup Warning", "Mutex not initialized during cleanup");
    /* Try to continue cleanup anyway, but state is uncertain */
  } else if (xSemaphoreTake(s_log_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(TAG,
              "Mutex Error",
              "Failed to take mutex during cleanup");
    /* Cannot safely proceed with buffer flush without mutex */
    ret = ESP_ERR_TIMEOUT;
  } else {
    /* Mutex acquired successfully */

#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
    /* Flush any remaining logs */
    esp_err_t temp_ret = priv_flush_log_buffer(); /* Call private flush directly */
    if (temp_ret != ESP_OK) {
      log_warn(TAG,
               "Flush Warning",
               "Failed to flush logs during cleanup: %s",
               esp_err_to_name(temp_ret));
      ret = temp_ret; /* Record the error */
    }
#endif

    /* Reset current log file path */
    memset(s_current_log_file, 0, sizeof(s_current_log_file));

    /* Reset buffer state */
    s_log_buffer_index = 0;
    memset(s_log_buffer, 0, sizeof(s_log_buffer));

    /* Reset file manager pointer */
    s_file_manager = NULL;

    /* Give mutex back before deleting it */
    xSemaphoreGive(s_log_mutex);
  } /* End mutex handling block */

  /* Delete mutex */
  if (s_log_mutex != NULL) {
    vSemaphoreDelete(s_log_mutex);
    s_log_mutex = NULL;
  }

  /* Reset initialization flag */
  s_log_storage_initialized = false;
  /* Reset atomic SD availability flag */
  atomic_store(&s_sd_card_available, false);

  log_info(TAG,
           "Cleanup Complete",
           "Log storage cleanup %s",
           (ret == ESP_OK) ? "successful" : "completed with warnings");

  return ret;
}
