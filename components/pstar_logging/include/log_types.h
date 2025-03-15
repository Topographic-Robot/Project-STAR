/* components/pstar_logging/include/log_types.h */

#ifndef PSTAR_LOG_TYPES_H
#define PSTAR_LOG_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "log_macros.h"

/* Structs ********************************************************************/

/**
 * @brief Structure for storing a log entry
 *      
 * This structure contains all the information needed for a single log entry,
 * including the message text, log level, and timestamp when the log was created.
 */
typedef struct log_entry {
  char            buffer[PSTAR_LOGGING_MAX_MESSAGE_LENGTH]; /**< Buffer for the log message text */
  esp_log_level_t level;                                  /**< Log level (error, warning, info, etc.) */
  uint64_t        timestamp;                              /**< Timestamp when the log was created */
} log_entry_t;

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_LOG_TYPES_H */
