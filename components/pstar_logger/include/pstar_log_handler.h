/* components/pstar_logger/include/pstar_log_handler.h */

#ifndef PSTAR_LOG_HANDLER_H
#define PSTAR_LOG_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stddef.h>

/* Forward declarations */
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
typedef struct sd_card_hal        sd_card_hal_t;
typedef struct file_write_manager file_write_manager_t;
#endif

/* Global Variables ***********************************************************/

extern _Atomic uint64_t g_log_sequence_number; /* Atomic counter for log sequence numbers */

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the log handler for full functionality.
 *
 * Sets up the log handler, including initializing SD card logging if
 * CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED is set and valid pointers
 * are provided. If SD logging is disabled, or if NULL pointers are provided
 * when SD logging is enabled, this function configures the system for
 * console-only logging initially. A subsequent call with valid pointers
 * (if SD is enabled) can complete the full initialization.
 *
 * @param[in] file_manager Pointer to the file write manager instance. Required for full
 *                         initialization if SD logging is enabled. Can be NULL for
 *                         minimal (console-only) initialization.
 * @param[in] sd_card      Pointer to the SD card HAL instance. Required for full
 *                         initialization if SD logging is enabled. Can be NULL for
 *                         minimal (console-only) initialization.
 * @return
 *  - ESP_OK on success (minimal or full initialization).
 *  - ESP_ERR_INVALID_ARG if SD logging is enabled but only one of file_manager/sd_card is NULL.
 *  - ESP_FAIL or other error codes if underlying storage initialization fails during full init.
 *
 * @note Basic console logging works even before this function is called.
 *       Calling this function enables SD card logging (if configured and pointers are valid).
 */
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
esp_err_t log_init(file_write_manager_t* file_manager,
                   sd_card_hal_t*        sd_card);
#else
esp_err_t log_init(void* file_manager, /* Parameters ignored if SD disabled */
                   void* sd_card);
#endif


/**
 * @brief Checks if the full logger (including potential storage) is initialized.
 *
 * @return true if the logger has been fully initialized via log_init(), false otherwise.
 *         Note: Console logging works even if this returns false.
 */
bool log_is_fully_initialized(void);

/**
 * @brief Cleans up the log handler resources.
 *
 * Performs cleanup of resources allocated during log system initialization.
 * This includes:
 * 1. Flushing any buffered logs (if SD storage was initialized).
 * 2. Cleaning up the storage component (if initialized).
 * 3. Resetting internal state.
 *
 * @return ESP_OK if successful, ESP_ERR_INVALID_STATE if called before initialization,
 *         or other error codes from underlying operations.
 *
 * @note This function should be called during system shutdown after all
 *       other components have finished logging. Console logging via ESP_LOGx
 *       may still work after this, but pstar_logger features will be disabled.
 */
esp_err_t log_cleanup(void);

/**
 * @brief Flushes any buffered logs to the SD card.
 *
 * If SD card logging is enabled and initialized, this function triggers
 * a flush of any logs currently buffered in memory to the storage device.
 *
 * @return
 *  - ESP_OK if flush was successful or if SD logging is disabled/not initialized.
 *  - ESP_ERR_INVALID_STATE if called before full initialization (when SD enabled).
 *  - Or other error codes from the storage flush operation.
 */
esp_err_t log_flush(void);

/**
 * @brief Log a message with the specified level and tag.
 *
 * This is the core logging function. It handles formatting, console output,
 * and (if fully initialized) storage output. The inline wrappers below provide
 * a more convenient interface.
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
 * @brief Variadic version of log_write.
 *
 * @param[in] level        Log level (ESP_LOG_xxx)
 * @param[in] tag          Component or module identifier
 * @param[in] short_msg    Short description of the log
 * @param[in] detailed_msg Detailed message with optional format specifiers
 * @param[in] ...          Variable arguments for format string
 */
void log_write(esp_log_level_t   level,
               const char* const tag,
               const char* const short_msg,
               const char* const detailed_msg,
               ...)
               __attribute__((format(printf, 4, 5)));

/* Inline Function Wrappers ***************************************************/

/**
 * @brief Log an error message.
 *
 * Always outputs to console if enabled. Outputs to storage if logger is fully initialized.
 *
 * @param[in] tag          Component or module identifier
 * @param[in] short_msg    Short description of the log
 * @param[in] detailed_msg Detailed message with optional format specifiers
 * @param[in] ...          Variable arguments for format string
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
 * @brief Log a warning message.
 *
 * Always outputs to console if enabled. Outputs to storage if logger is fully initialized.
 *
 * @param[in] tag          Component or module identifier
 * @param[in] short_msg    Short description of the log
 * @param[in] detailed_msg Detailed message with optional format specifiers
 * @param[in] ...          Variable arguments for format string
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
 * @brief Log an info message.
 *
 * Always outputs to console if enabled. Outputs to storage if logger is fully initialized.
 *
 * @param[in] tag          Component or module identifier
 * @param[in] short_msg    Short description of the log
 * @param[in] detailed_msg Detailed message with optional format specifiers
 * @param[in] ...          Variable arguments for format string
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
 * @brief Log a debug message.
 *
 * Always outputs to console if enabled. Outputs to storage if logger is fully initialized.
 *
 * @param[in] tag          Component or module identifier
 * @param[in] short_msg    Short description of the log
 * @param[in] detailed_msg Detailed message with optional format specifiers
 * @param[in] ...          Variable arguments for format string
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
 * @brief Log a verbose message.
 *
 * Always outputs to console if enabled. Outputs to storage if logger is fully initialized.
 *
 * @param[in] tag          Component or module identifier
 * @param[in] short_msg    Short description of the log
 * @param[in] detailed_msg Detailed message with optional format specifiers
 * @param[in] ...          Variable arguments for format string
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
