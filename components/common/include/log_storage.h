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

extern const char *log_storage_tag;          /* Tag for logging */
extern const char *log_base_dir;             /* Base directory for logs */
extern const int   log_max_file_size;        /* Maximum log file size in bytes */
extern const int   log_max_files;            /* Maximum number of log files to keep */
extern const int   log_compression_enabled;  /* Enable/disable compression (1=enabled, 0=disabled) */
extern const int   log_compression_level;    /* Compression level (0-9, or Z_DEFAULT_COMPRESSION) */
extern const int   log_compression_buffer;   /* Size of compression buffer */
extern const char *log_compressed_extension; /* Extension for compressed log files */
extern const int   zlib_window_bits;         /* Window size with gzip header */
extern const int   zlib_mem_level;           /* Memory level for zlib compression */

/* Macros *********************************************************************/

#define LOG_BUFFER_SIZE                        (10)                                     /* Size of the log buffer for temporary storage */
#define LOG_STORAGE_MAX_MESSAGE_LENGTH         (256)                                    /* Maximum length of log messages */
#define TIMESTAMP_BUFFER_SIZE                  (64)                                     /* Buffer size for formatted timestamp strings */
#define DATE_STRING_BUFFER_SIZE                (32)                                     /* Buffer size for date strings */
#define LOG_STORAGE_MAX_FORMATTED_ENTRY_LENGTH (LOG_STORAGE_MAX_MESSAGE_LENGTH * 2)     /* Formatted log entry buffer size */

/* Structs ********************************************************************/

/**
 * @brief Structure for storing a log entry
 *      
 * This structure contains all the information needed for a single log entry,
 * including the message text, log level, and timestamp when the log was created.
 */
typedef struct {
  char            buffer[LOG_STORAGE_MAX_MESSAGE_LENGTH]; /**< Buffer for the log message text */
  esp_log_level_t level;                                  /**< Log level (error, warning, info, etc.) */
  uint64_t        timestamp;                              /**< Timestamp when the log was created */
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
 * @param[in] level   Log level
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
