/* components/common/include/log_handler.h */

#ifndef TOPOROBO_LOG_HANDLER_H
#define TOPOROBO_LOG_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_log.h"
#include <stdarg.h>

/* Macros ********************************************************************/

#define LOG_MAX_MESSAGE_LENGTH (256)
#define LOG_MAX_TAG_LENGTH     (32)
#define LOG_SEPARATOR          (" - ")  /* Separator between log components */

/* Public Functions **********************************************************/

/**
 * @brief Log a message with the specified level and tag
 * 
 * This is the core logging function. The inline wrappers below provide
 * a more convenient interface with compile-time format checking.
 * 
 * @param level        Log level (ESP_LOG_xxx)
 * @param tag          Component or module identifier
 * @param short_msg    Short description of the log
 * @param detailed_msg Detailed message with optional format specifiers
 * @param args         va_list of arguments for format string
 * @return esp_err_t   ESP_OK on success, error code otherwise
 */
esp_err_t log_write_va(esp_log_level_t level, const char *tag,
                       const char *short_msg, const char *detailed_msg,
                       va_list args);

/**
 * @brief Variadic version of log_write
 */
esp_err_t log_write(esp_log_level_t level, const char *tag,
                    const char *short_msg, const char *detailed_msg, ...)
                    __attribute__((format(printf, 4, 5)));

/* Inline Function Wrappers *************************************************/

/**
 * @brief Log an error message
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
esp_err_t log_error(const char *tag, const char *short_msg, 
                    const char *detailed_msg, ...) 
{
  va_list args;
  va_start(args, detailed_msg);
  esp_err_t ret = log_write_va(ESP_LOG_ERROR, tag, short_msg, detailed_msg, args);
  va_end(args);
  return ret;
}

/**
 * @brief Log a warning message
 */
static inline __attribute__((format(printf, 3, 4))) 
esp_err_t log_warn(const char *tag, const char *short_msg,
                   const char *detailed_msg, ...) 
{
  va_list args;
  va_start(args, detailed_msg);
  esp_err_t ret = log_write_va(ESP_LOG_WARN, tag, short_msg, detailed_msg, args);
  va_end(args);
  return ret;
}

/**
 * @brief Log an info message
 */
static inline __attribute__((format(printf, 3, 4))) 
esp_err_t log_info(const char *tag, const char *short_msg,
                   const char *detailed_msg, ...) 
{
  va_list args;
  va_start(args, detailed_msg);
  esp_err_t ret = log_write_va(ESP_LOG_INFO, tag, short_msg, detailed_msg, args);
  va_end(args);
  return ret;
}

/**
 * @brief Log a debug message
 */
static inline __attribute__((format(printf, 3, 4))) 
esp_err_t log_debug(const char *tag, const char *short_msg,
                    const char *detailed_msg, ...) 
{
  va_list args;
  va_start(args, detailed_msg);
  esp_err_t ret = log_write_va(ESP_LOG_DEBUG, tag, short_msg, detailed_msg, args);
  va_end(args);
  return ret;
}

/**
 * @brief Log a verbose message
 */
static inline __attribute__((format(printf, 3, 4))) 
esp_err_t log_verbose(const char *tag, const char *short_msg,
                      const char *detailed_msg, ...) 
{
  va_list args;
  va_start(args, detailed_msg);
  esp_err_t ret = log_write_va(ESP_LOG_VERBOSE, tag, short_msg, detailed_msg, args);
  va_end(args);
  return ret;
}

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_LOG_HANDLER_H */ 