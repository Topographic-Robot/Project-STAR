/* components/pstar_logging/log_storage.c */

#include "log_storage.h"
#include "log_handler.h"
#include "esp_system.h"
#include "log_macros.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <unistd.h>
#include <zlib.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Constants ******************************************************************/

#define LOG_STORAGE_TAG ("Log Storage")

/* Using Kconfig macros with the proper prefix */
#define PSTAR_LOGGING_MAX_FILE_SIZE          (CONFIG_PSTAR_KCONFIG_LOGGING_MAX_FILE_SIZE_KB * 1024)
#define PSTAR_LOGGING_COMPRESSION_BUFFER     CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_BUFFER_SIZE
#define PSTAR_LOGGING_ZLIB_GZIP_WINDOW_BITS  (CONFIG_PSTAR_KCONFIG_LOGGING_ZLIB_WINDOW_BITS + 16)
#define PSTAR_LOGGING_ZLIB_MEMORY_LEVEL      CONFIG_PSTAR_KCONFIG_LOGGING_ZLIB_MEM_LEVEL
#define PSTAR_LOGGING_BASE_DIRECTORY         CONFIG_PSTAR_KCONFIG_LOGGING_BASE_DIR
#define PSTAR_LOGGING_COMPRESSED_FILE_EXT    CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSED_EXTENSION

/* Globals (Static) ***********************************************************/

static log_entry_t       s_log_buffer[CONFIG_PSTAR_KCONFIG_LOGGING_BUFFER_SIZE]                = {0};   /* Buffer to store logs when SD card is not available */
static uint32_t          s_log_buffer_index                                                    = 0;     /* Current index in the buffer */
static bool              s_log_storage_initialized                                             = false; /* Flag to track initialization status */
static char              s_current_log_file[CONFIG_PSTAR_KCONFIG_LOGGING_MAX_FILE_PATH_LENGTH] = {0};   /* Current log file path */
static SemaphoreHandle_t s_log_mutex                                                           = NULL;  /* Mutex for thread-safe access */
static bool              s_compression_enabled                                                 = true;  /* Compression state */
static bool              s_sd_card_available                                                   = false; /* Flag indicating if SD card is available */
static file_write_manager_t* s_file_manager                                                    = NULL;  /* File write manager instance */
static sd_card_hal_t*    s_sd_card                                                             = NULL;  /* SD card HAL instance */

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
 * @return Number of characters written (excluding null terminator)
 */
static inline int priv_format_date_string(char*                  buffer, 
                                          size_t                 buffer_size, 
                                          const struct tm *const timeinfo)
{
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
 * @return Number of characters written (excluding null terminator)
 */
static inline int priv_format_log_entry(char*                  buffer,
                                        size_t                 buffer_size,
                                        const struct tm* const timeinfo,
                                        uint64_t               milliseconds,
                                        const char* const      level_str,
                                        const char* const      message)
{
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
 * @return Number of characters written (excluding null terminator)
 */
static inline int priv_format_log_filepath(char*                  buffer,
                                           size_t                 buffer_size,
                                           const struct tm* const timeinfo,
                                           const char* const      extension)
{
  char date_str[PSTAR_LOGGING_DATE_STRING_BUFFER_SIZE];
  priv_format_date_string(date_str, sizeof(date_str), timeinfo);
  
  return snprintf(buffer, 
                  buffer_size,
                  "%s/%s/%04d-%02d-%02d_%02d-%02d-%02d%s",
                  PSTAR_LOGGING_BASE_DIRECTORY,
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
  struct tm timeinfo;
  time_t    now = time(NULL);
  localtime_r(&now, &timeinfo);
  
  const char* const extension = s_compression_enabled ? PSTAR_LOGGING_COMPRESSED_FILE_EXT : ".txt";
  priv_format_log_filepath(file_path, file_path_len, &timeinfo, extension);
}

/**
 * @brief Checks if log rotation is needed
 * 
 * @return true if rotation is needed, false otherwise
 */
static bool priv_check_log_rotation(void)
{
  /* If no current log file, no need to rotate */
  if (strlen(s_current_log_file) == 0) {
    return false;
  }
  
  /* Check file size */
  struct stat st;
  if (stat(s_current_log_file, &st) != 0) {
    /* File doesn't exist, no need to rotate */
    return false;
  }
  
  /* Check if file size exceeds the maximum */
  if (st.st_size >= PSTAR_LOGGING_MAX_FILE_SIZE) {
    return true;
  }
  
  /* Check if date has changed */
  struct tm timeinfo;
  time_t    now = time(NULL);
  localtime_r(&now, &timeinfo);
  
  char date_str[PSTAR_LOGGING_DATE_STRING_BUFFER_SIZE];
  priv_format_date_string(date_str, sizeof(date_str), &timeinfo);
  
  /* Extract date from current log file path */
  char *date_start = strstr(s_current_log_file, PSTAR_LOGGING_BASE_DIRECTORY);
  if (date_start == NULL) {
    return false;
  }
  
  /* If date has changed, rotate */
  if (strstr(s_current_log_file, date_str) == NULL) {
    return true;
  }
  
  return false;
}

/**
 * @brief Rotates the log file if needed
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_rotate_log_file(void)
{
  if (!priv_check_log_rotation()) {
    return ESP_OK; /* No rotation needed */
  }
  
  /* Generate new log file path */
  priv_generate_log_file_path(s_current_log_file, sizeof(s_current_log_file));
  log_info(LOG_STORAGE_TAG, 
           "Log Rotation", 
           "Rotating to new log file: %s", 
           s_current_log_file);
  
  return ESP_OK;
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
  if (input == NULL || output == NULL || output_len == NULL) {
    return ESP_FAIL;
  }
  
  z_stream stream;
  memset(&stream, 0, sizeof(stream));
  
  /* Initialize zlib for gzip compression */
  int ret = deflateInit2(&stream, 
                         Z_DEFAULT_COMPRESSION, 
                         Z_DEFLATED,
                         PSTAR_LOGGING_ZLIB_GZIP_WINDOW_BITS,
                         PSTAR_LOGGING_ZLIB_MEMORY_LEVEL,
                         Z_DEFAULT_STRATEGY);
  if (ret != Z_OK) {
    log_error(LOG_STORAGE_TAG, 
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
    log_error(LOG_STORAGE_TAG, 
              "Compression Failed", 
              "Failed to compress data: %d", 
              ret);
    return ESP_FAIL;
  }
  
  /* Update output length */
  *output_len = stream.total_out;
  
  return ESP_OK;
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
  if (file_path == NULL || data == NULL || s_file_manager == NULL) {
    return ESP_FAIL;
  }
  
  /* Check if compression is enabled */
  if (s_compression_enabled) {
    /* Compress data before writing */
    char *compress_buffer = malloc(PSTAR_LOGGING_COMPRESSION_BUFFER);
    if (compress_buffer == NULL) {
      log_error(LOG_STORAGE_TAG, 
                "Memory Error", 
                "Failed to allocate compression buffer");
      return ESP_FAIL;
    }
    
    size_t data_len   = strlen(data);
    size_t output_len = PSTAR_LOGGING_COMPRESSION_BUFFER;
    
    esp_err_t ret = priv_compress_data(data, 
                                       data_len, 
                                       compress_buffer, 
                                       &output_len);
    if (ret != ESP_OK) {
      free(compress_buffer);
      return ret;
    }
    
    /* Write compressed data to file */
    ret = file_write_binary_enqueue(s_file_manager, file_path, compress_buffer, output_len);
    free(compress_buffer);
    return ret;
  } else {
    /* Write uncompressed data to file */
    return file_write_enqueue(s_file_manager, file_path, data);
  }
}

/**
 * @brief Flushes the log buffer to disk
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_flush_log_buffer(void)
{
  if (s_log_buffer_index == 0) {
    return ESP_OK; /* Nothing to flush */
  }
  
  if (!s_sd_card_available) {
    log_warn(LOG_STORAGE_TAG, 
             "Flush Skip", 
             "SD card not available, keeping %lu logs in buffer", 
             s_log_buffer_index);
    return ESP_FAIL;
  }
  
  /* Ensure we have a valid log file */
  if (priv_rotate_log_file() != ESP_OK) {
    return ESP_FAIL;
  }
  
  /* If compression is enabled, we'll collect all logs into a single buffer first */
  char*  all_logs   = NULL;
  size_t total_size = 0;
  
  if (s_compression_enabled) {
    /* Calculate total size needed */
    for (uint32_t i = 0; i < s_log_buffer_index; i++) {
      const char* level_str = priv_log_level_to_string(s_log_buffer[i].level);
      
      /* Convert timestamp to time components */
      time_t    log_time = priv_timestamp_us_to_seconds(s_log_buffer[i].timestamp);
      struct tm timeinfo;
      localtime_r(&log_time, &timeinfo);
      
      /* Calculate the size of this log entry */
      /* Format: "%04d-%02d-%02d %02d:%02d:%02d.%03llu [%s] %s\n" */
      /* 4+1+2+1+2 + 1 + 2+1+2+1+2+1+3 + 3 + strlen(level_str) + 2 + strlen(buffer) + 1(newline) + 1(null) */
      size_t timestamp_len = 26; /* Fixed length for timestamp format */
      size_t level_len     = strlen(level_str);
      size_t message_len   = strlen(s_log_buffer[i].buffer);
      size_t entry_len     = timestamp_len + level_len + message_len + 2; /* +1 for newline, +1 for safety */
      
      /* Check for potential overflow */
      if (total_size > SIZE_MAX - entry_len) {
        log_error(LOG_STORAGE_TAG, 
                  "Buffer Error", 
                  "Log buffer size would overflow size_t");
        return ESP_ERR_INVALID_SIZE;
      }
      
      total_size += entry_len;
    }
    
    /* Add 1 for null terminator */
    if (total_size > SIZE_MAX - 1) {
      log_error(LOG_STORAGE_TAG, 
                "Buffer Error", 
                "Log buffer size would overflow with null terminator");
      return ESP_ERR_INVALID_SIZE;
    }
    total_size += 1;
    
    /* Allocate buffer for all logs */
    all_logs = malloc(total_size);
    if (!all_logs) {
      log_error(LOG_STORAGE_TAG, 
                "Memory Error", 
                "Failed to allocate buffer for log compression (size: %zu)", 
                total_size);
      return ESP_ERR_NO_MEM;
    }
    
    /* Reset buffer position */
    size_t pos = 0;
    
    /* Collect all logs into the buffer */
    for (uint32_t i = 0; i < s_log_buffer_index; i++) {
      const char* level_str = priv_log_level_to_string(s_log_buffer[i].level);
      
      /* Convert timestamp to time components */
      time_t    log_time = priv_timestamp_us_to_seconds(s_log_buffer[i].timestamp);
      struct tm timeinfo;
      localtime_r(&log_time, &timeinfo);
      
      /* Get milliseconds for formatting */
      uint64_t milliseconds = priv_timestamp_us_to_milliseconds(s_log_buffer[i].timestamp);
      
      /* Format the timestamp and log entry - ensure there's enough space */
      size_t remaining_space = total_size - pos;
      if (remaining_space <= 1) {
        log_error(LOG_STORAGE_TAG,
                  "Buffer Error",
                  "Insufficient space remaining in buffer");
        free(all_logs);
        return ESP_FAIL;
      }
      
      int written = priv_format_log_entry(all_logs + pos, 
                                          remaining_space, 
                                          &timeinfo,
                                          milliseconds,
                                          level_str,
                                          s_log_buffer[i].buffer);
      
      if (written < 0 || (size_t)written >= remaining_space) {
        log_error(LOG_STORAGE_TAG,
                  "Format Error",
                  "Failed to format log entry or insufficient buffer space");
        free(all_logs);
        return ESP_FAIL;
      }
      
      pos += written;
      
      /* Add newline if there's space */
      if (pos < total_size - 1) {
        all_logs[pos++] = '\n';
      } else {
        log_warn(LOG_STORAGE_TAG,
                 "Format Warning",
                 "No space for newline in buffer");
        break;
      }
    }
    
    /* Ensure null termination */
    if (pos < total_size) {
      all_logs[pos] = '\0';
    } else {
      all_logs[total_size - 1] = '\0';
      log_warn(LOG_STORAGE_TAG,
               "Format Warning",
               "Buffer filled completely, possible truncation");
    }
    
    /* Write all logs at once */
    esp_err_t ret = priv_write_log_data(s_current_log_file, all_logs);
    free(all_logs);
    
    if (ret != ESP_OK) {
      log_error(LOG_STORAGE_TAG, 
                "Write Failed", 
                "Failed to write compressed logs: %s", 
                esp_err_to_name(ret));
      return ESP_FAIL;
    }
  } else {
    /* No compression, write each log entry individually */
    for (uint32_t i = 0; i < s_log_buffer_index; i++) {
      const char* level_str = priv_log_level_to_string(s_log_buffer[i].level);
      
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
        log_error(LOG_STORAGE_TAG,
                  "Format Error",
                  "Failed to format log entry or buffer too small");
        return ESP_FAIL;
      }
      
      /* Enqueue the log for writing */
      esp_err_t ret = file_write_enqueue(s_file_manager, s_current_log_file, formatted_log);
      if (ret != ESP_OK) {
        log_error(LOG_STORAGE_TAG, 
                  "Write Failed", 
                  "Failed to enqueue log for writing: %s", 
                  esp_err_to_name(ret));
        return ESP_FAIL;
      }
    }
  }
  
  /* Reset buffer index */
  s_log_buffer_index = 0;
  
  log_info(LOG_STORAGE_TAG, 
           "Buffer Flushed", 
           "Successfully flushed log buffer to SD card");
  return ESP_OK;
}

/* Public Functions ***********************************************************/

esp_err_t log_storage_init(file_write_manager_t* manager, sd_card_hal_t* sd_card)
{
  if (manager == NULL || sd_card == NULL) {
    log_error(LOG_STORAGE_TAG, 
              "Init Error", 
              "Invalid arguments: manager or sd_card is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(LOG_STORAGE_TAG, "Init", "Initializing log storage");
  
  if (s_log_storage_initialized) {
    log_warn(LOG_STORAGE_TAG, "Already Init", "Log storage already initialized");
    return ESP_OK;
  }
  
  /* Save the file manager and SD card instances */
  s_file_manager = manager;
  s_sd_card = sd_card;
  
  /* Create mutex for thread-safe access */
  s_log_mutex = xSemaphoreCreateMutex();
  if (s_log_mutex == NULL) {
    log_error(LOG_STORAGE_TAG, "Mutex Failed", "Failed to create mutex");
    return ESP_FAIL;
  }
  
  /* Initialize SD card availability status based on current state */
  s_sd_card_available = sd_card_is_available(sd_card);
  
  /* Register for SD card availability notifications */
  /* Note: The actual function call was causing an error. This needs to be
   * implemented in the SD card HAL. For now, we'll update the SD card state
   * manually when we need to write logs. */
   
  /* commented out for now as it's not defined in the SD card HAL
  sd_card_register_availability_callback(log_storage_set_sd_available);
  */
  
  s_log_storage_initialized = true;
  log_info(LOG_STORAGE_TAG, 
           "Init Success", 
           "Log storage initialized successfully");
  
  return ESP_OK;
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
  if (!s_log_storage_initialized) {
    return;
  }
  
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(LOG_STORAGE_TAG, 
              "Mutex Error", 
              "Failed to acquire mutex for SD card status update");
    return;
  }
  
  s_sd_card_available = available;
  
  if (available) {
    /* SD card became available, try to flush buffer */
    log_info(LOG_STORAGE_TAG, 
             "SD Available", 
             "SD card became available, flushing buffered logs");
    priv_flush_log_buffer();
  } else {
    log_warn(LOG_STORAGE_TAG, 
             "SD Unavailable", 
             "SD card became unavailable, logs will be buffered");
  }
  
  xSemaphoreGive(s_log_mutex);
}

esp_err_t log_storage_write(esp_log_level_t  level, 
                           const char* const message)
{
  if (!s_log_storage_initialized) {
    return ESP_FAIL;
  }
  
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(LOG_STORAGE_TAG, 
              "Mutex Error", 
              "Failed to acquire mutex for log write");
    return ESP_FAIL;
  }
  
  /* Update SD card availability (in place of the callback that's not defined) */
  if (s_sd_card != NULL) {
    s_sd_card_available = sd_card_is_available(s_sd_card);
  }
  
  /* Store log in buffer */
  if (s_log_buffer_index < PSTAR_LOGGING_BUFFER_SIZE) {
    s_log_buffer[s_log_buffer_index].level     = level;
    s_log_buffer[s_log_buffer_index].timestamp = esp_timer_get_time();

    strncpy(s_log_buffer[s_log_buffer_index].buffer, 
            message, 
            PSTAR_LOGGING_MAX_MESSAGE_LENGTH - 1);
    s_log_buffer[s_log_buffer_index].buffer[PSTAR_LOGGING_MAX_MESSAGE_LENGTH - 1] = '\0';
    s_log_buffer_index++;
  } else {
    /* Buffer is full, need to flush */
    log_warn(LOG_STORAGE_TAG, "Buffer Full", "Log buffer full, forcing flush");
    priv_flush_log_buffer();
    
    /* Store the current log */
    s_log_buffer[0].level     = level;
    s_log_buffer[0].timestamp = esp_timer_get_time();

    strncpy(s_log_buffer[0].buffer, 
            message, 
            PSTAR_LOGGING_MAX_MESSAGE_LENGTH - 1);
    s_log_buffer[0].buffer[PSTAR_LOGGING_MAX_MESSAGE_LENGTH - 1] = '\0';
    s_log_buffer_index = 1;
  }
  
  /* If buffer is full or SD card is available, try to flush */
  if (s_log_buffer_index >= PSTAR_LOGGING_BUFFER_SIZE && s_sd_card_available) {
    priv_flush_log_buffer();
  }
  
  xSemaphoreGive(s_log_mutex);
  return ESP_OK;
}

esp_err_t log_storage_flush(void)
{
  if (!s_log_storage_initialized) {
    log_error(LOG_STORAGE_TAG, "Flush Error", "Log storage not initialized");
    return ESP_FAIL;
  }
  
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(LOG_STORAGE_TAG, 
              "Mutex Error", 
              "Failed to acquire mutex for log flush");
    return ESP_FAIL;
  }
  
  /* Update SD card availability (in place of the callback that's not defined) */
  if (s_sd_card != NULL) {
    s_sd_card_available = sd_card_is_available(s_sd_card);
  }
  
  esp_err_t ret = priv_flush_log_buffer();
  
  xSemaphoreGive(s_log_mutex);
  log_info(LOG_STORAGE_TAG, 
           "Flush Complete", 
           "Log flush completed successfully");
  return ret;
}

esp_err_t log_storage_set_compression(bool enabled)
{
  if (!s_log_storage_initialized) {
    log_error(LOG_STORAGE_TAG, "Config Error", "Log storage not initialized");
    return ESP_FAIL;
  }
  
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(LOG_STORAGE_TAG, 
              "Mutex Error", 
              "Failed to acquire mutex for compression config");
    return ESP_FAIL;
  }
  
  /* Only update if the setting has changed */
  if (s_compression_enabled != enabled) {
    s_compression_enabled = enabled;
    log_info(LOG_STORAGE_TAG, 
             "Compression Config", 
             "Log compression %s", 
             enabled ? "enabled" : "disabled");
    
    /* Flush current logs before changing compression setting */
    priv_flush_log_buffer();
    
    /* Force log rotation on next write to use new extension */
    s_current_log_file[0] = '\0';
  }
  
  xSemaphoreGive(s_log_mutex);
  return ESP_OK;
}

bool log_storage_is_compression_enabled(void)
{
  return s_compression_enabled;
}

esp_err_t log_storage_cleanup(void)
{
  log_info(LOG_STORAGE_TAG, "Cleanup Start", "Beginning log storage cleanup");
  esp_err_t ret = ESP_OK;

  /* Take mutex for thread safety */
  if (xSemaphoreTake(s_log_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(LOG_STORAGE_TAG, 
              "Mutex Error", 
              "Failed to take mutex during cleanup");
    return ESP_ERR_TIMEOUT;
  }

  /* Flush any remaining logs */
  esp_err_t temp_ret = log_storage_flush();
  if (temp_ret != ESP_OK) {
    log_warn(LOG_STORAGE_TAG, 
             "Flush Warning", 
             "Failed to flush logs during cleanup: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }

  /* Reset current log file path */
  memset(s_current_log_file, 0, sizeof(s_current_log_file));

  /* Reset buffer state */
  s_log_buffer_index = 0;
  memset(s_log_buffer, 0, sizeof(s_log_buffer));

  /* Reset file manager and SD card pointers */
  s_file_manager = NULL;
  s_sd_card = NULL;

  /* Give mutex back before deleting it */
  xSemaphoreGive(s_log_mutex);

  /* Delete mutex */
  if (s_log_mutex != NULL) {
    vSemaphoreDelete(s_log_mutex);
    s_log_mutex = NULL;
  }

  /* Reset initialization flag */
  s_log_storage_initialized = false;

  log_info(LOG_STORAGE_TAG, 
           "Cleanup Complete", 
           "Log storage cleanup %s", 
           (ret == ESP_OK) ? "successful" : "completed with warnings");

  return ret;
}

