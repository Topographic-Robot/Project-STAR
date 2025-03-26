/* components/pstar_logger/include/log_macros.h */

#ifndef PSTAR_LOG_MACROS_H
#define PSTAR_LOG_MACROS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include <stdint.h>

/**
 * @brief Macros for logging constants not directly available from Kconfig
 *
 * These macros define derived constants that aren't directly specified in Kconfig.
 */

#define PSTAR_LOGGING_MAX_FORMATTED_ENTRY_LENGTH (CONFIG_PSTAR_KCONFIG_LOGGING_MAX_MESSAGE_LENGTH * 2)  /**< Maximum length for complete formatted log entry including timestamp, level, task info, etc. */
#define PSTAR_LOGGING_MAX_FILE_PATH_LENGTH       (256)                                                  /**< Maximum path length for log files */
#define PSTAR_LOGGING_MAX_FILE_SIZE              (CONFIG_PSTAR_KCONFIG_LOGGING_MAX_FILE_SIZE_KB * 1024) /**< Maximum file size for log files */
#define PSTAR_LOGGING_ZLIB_GZIP_WINDOW_BITS      (CONFIG_PSTAR_KCONFIG_LOGGING_ZLIB_WINDOW_BITS + 16)   /**< Zlib Gzip window bits */
#define PSTAR_LOGGING_MAX_COMPRESSION_INPUT_SIZE (1024 * 1024)                                          /**< 1MB max buffer before compression */

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_LOG_MACROS_H */