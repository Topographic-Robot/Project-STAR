/* components/common/include/log_storage.h */

#ifndef TOPOROBO_LOG_STORAGE_H
#define TOPOROBO_LOG_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sd_card_hal.h"

/* Constants ******************************************************************/

extern const char    *log_storage_tag;          /* Tag for logging */
extern const char    *log_base_dir;             /* Base directory for logs */
extern const int      log_max_file_size;        /* Maximum log file size in bytes */
extern const int      log_max_files;            /* Maximum number of log files to keep */
extern const int      date_string_buffer_size;  /* Size of buffer for date strings */
extern const int      log_compression_enabled;  /* Enable/disable compression (1=enabled, 0=disabled) */
extern const int      log_compression_level;    /* Compression level (0-9, or Z_DEFAULT_COMPRESSION) */
extern const int      log_compression_buffer;   /* Size of compression buffer */
extern const char    *log_compressed_extension; /* Extension for compressed log files */

/* Macros *********************************************************************/

#define LOG_BUFFER_SIZE                 10   /* Size of the log buffer for temporary storage */
#define LOG_STORAGE_MAX_MESSAGE_LENGTH  256  /* Maximum length of log messages */
#define TIMESTAMP_FORMAT                "%04d-%02d-%02d %02d:%02d:%02d.%03llu"

/* Format helpers for timestamp */
#define FORMAT_DATE_ARGS(tm_ptr)        ((tm_ptr)->tm_year + 1900), ((tm_ptr)->tm_mon + 1), ((tm_ptr)->tm_mday)
#define FORMAT_TIME_ARGS(tm_ptr)        ((tm_ptr)->tm_hour), ((tm_ptr)->tm_min), ((tm_ptr)->tm_sec)

/* Structs ********************************************************************/

typedef struct {
  char            buffer[LOG_STORAGE_MAX_MESSAGE_LENGTH];
  esp_log_level_t level;
  uint64_t        timestamp;
} log_entry_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the log storage system
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_storage_init(void);

/**
 * @brief Callback function for SD card availability changes
 * 
 * This function is called by the SD card HAL when the SD card availability changes.
 * It updates the internal state and flushes the log buffer if the SD card becomes available.
 * 
 * @param available Whether the SD card is available
 */
void log_storage_set_sd_available(bool available);

/**
 * @brief Writes a log message to storage
 * 
 * @param[in] level Log level
 * @param[in] message Log message
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_storage_write(esp_log_level_t level, const char *message);

/**
 * @brief Flushes the log buffer to disk
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_storage_flush(void);

/**
 * @brief Sets whether log compression is enabled
 * 
 * @param[in] enabled Whether compression is enabled
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_storage_set_compression(bool enabled);

/**
 * @brief Checks if log compression is enabled
 * 
 * @return true if compression is enabled, false otherwise
 */
bool log_storage_is_compression_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_LOG_STORAGE_H */ 