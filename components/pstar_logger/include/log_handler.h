/* components/pstar_logger/include/log_handler.h */

#ifndef PSTAR_LOG_HANDLER_H
#define PSTAR_LOG_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdatomic.h>

/* Forward declarations */
typedef struct sd_card_hal        sd_card_hal_t;
typedef struct file_write_manager file_write_manager_t;

/* Global Variables ***********************************************************/

extern _Atomic uint64_t g_log_sequence_number; /* Atomic counter for log sequence numbers */

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the log handler
 * 
 * Sets up the log handler and optionally initializes SD card logging if 
 * CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED is set.
 * 
 * @param[in] file_manager Pointer to the file write manager instance (required if SD logging is enabled)
 * @param[in] sd_card      Pointer to the SD card HAL instance (required if SD logging is enabled)
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_init(file_write_manager_t* file_manager, 
                   sd_card_hal_t*        sd_card);

/**
 * @brief Cleans up the log handler resources
 * 
 * Performs cleanup of resources allocated during log system initialization.
 * This includes:
 * 1. Flushing any buffered logs
 * 2. Closing open log files
 * 3. Freeing allocated memory
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 * 
 * @note This function should be called during system shutdown after all
 *       other components have finished logging.
 */
esp_err_t log_cleanup(void);

/**
 * @brief Enables or disables logging to SD card
 * 
 * This is only operational if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED is set.
 * It provides runtime control over the SD card logging feature.
 * 
 * @param[in] enabled Whether to enable logging to SD card
 */
void log_set_sd_logging(bool enabled);

/**
 * @brief Flushes any buffered logs to the SD card
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_flush(void);

/**
 * @brief Log a message with the specified level and tag
 * 
 * This is the core logging function. The inline wrappers below provide
 * a more convenient interface with compile-time format checking.
 * 
 * @param[in] level        Log level (ESP_LOG_xxx)
 * @param[in] tag          Component or module identifier
 * @param[in] short_msg    Short description of the log
 * @param[in] detailed_msg Detailed message with optional format specifiers
 * @param[in] args         va_list of arguments for format string
 */
void log_write_va(esp_log_level_t   level, 
                  const char* const tag,
                  const char* const short_msg, 
                  const char* const detailed_msg,
                  va_list           args);

/**
 * @brief Variadic version of log_write
 */
void log_write(esp_log_level_t   level, 
               const char* const tag,
               const char* const short_msg, 
               const char* const detailed_msg, 
               ...)
               __attribute__((format(printf, 4, 5)));

/* Inline Function Wrappers ***************************************************/

/**
 * @brief Log an error message
 * 
 * @param[in] tag          Component or module identifier
 * @param[in] short_msg    Short description of the log
 * @param[in] detailed_msg Detailed message with optional format specifiers
 * @param[in] args         va_list of arguments for format string
 * 
 * The inline keyword suggests to the compiler to insert the function code
 * directly at the call site instead of generating a function call.
 * 
 * The format attribute tells the compiler:
 * - This function uses printf-style formatting
 * - The format string is the 3rd parameter
 * - The variable arguments start at the 4th parameter
 * This enables compile-time format string checking.
 */
static inline __attribute__((format(printf, 3, 4))) 
void log_error(const char* const tag, 
               const char* const short_msg, 
               const char* const detailed_msg, 
               ...) 
{
  va_list args;
  va_start(args, detailed_msg);
  log_write_va(ESP_LOG_ERROR, tag, short_msg, detailed_msg, args);
  va_end(args);
}

/**
 * @brief Log a warning message
 * 
 * @param[in] tag          Component or module identifier
 * @param[in] short_msg    Short description of the log
 * @param[in] detailed_msg Detailed message with optional format specifiers
 * @param[in] args         va_list of arguments for format string
 */
static inline __attribute__((format(printf, 3, 4))) 
void log_warn(const char* const tag, 
              const char* const short_msg, 
              const char* const detailed_msg, 
              ...) 
{
  va_list args;
  va_start(args, detailed_msg);
  log_write_va(ESP_LOG_WARN, tag, short_msg, detailed_msg, args);
  va_end(args);
}

/**
 * @brief Log an info message
 * 
 * @param[in] tag          Component or module identifier
 * @param[in] short_msg    Short description of the log
 * @param[in] detailed_msg Detailed message with optional format specifiers
 * @param[in] args         va_list of arguments for format string
 */
static inline __attribute__((format(printf, 3, 4))) 
void log_info(const char* const tag, 
              const char* const short_msg,
              const char* const detailed_msg, 
              ...) 
{
  va_list args;
  va_start(args, detailed_msg);
  log_write_va(ESP_LOG_INFO, tag, short_msg, detailed_msg, args);
  va_end(args);
}

/**
 * @brief Log a debug message
 * 
 * @param[in] tag          Component or module identifier
 * @param[in] short_msg    Short description of the log
 * @param[in] detailed_msg Detailed message with optional format specifiers
 * @param[in] args         va_list of arguments for format string
 */
static inline __attribute__((format(printf, 3, 4))) 
void log_debug(const char* const tag, 
               const char* const short_msg,
               const char* const detailed_msg, 
               ...) 
{
  va_list args;
  va_start(args, detailed_msg);
  log_write_va(ESP_LOG_DEBUG, tag, short_msg, detailed_msg, args);
  va_end(args);
}

/**
 * @brief Log a verbose message
 * 
 * @param[in] tag          Component or module identifier
 * @param[in] short_msg    Short description of the log
 * @param[in] detailed_msg Detailed message with optional format specifiers
 * @param[in] args         va_list of arguments for format string
 */
static inline __attribute__((format(printf, 3, 4))) 
void log_verbose(const char* const tag, 
                 const char* const short_msg,
                 const char* const detailed_msg, 
                 ...) 
{
  va_list args;
  va_start(args, detailed_msg);
  log_write_va(ESP_LOG_VERBOSE, tag, short_msg, detailed_msg, args);
  va_end(args);
}

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_LOG_HANDLER_H */