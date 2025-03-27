/* components/pstar_logging/include/log_storage.h */

#ifndef PSTAR_LOG_STORAGE_H
#define PSTAR_LOG_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_log.h"
#include "sdkconfig.h" // Include sdkconfig first
#include "pstar_log_types.h"
#include <stdbool.h> // Include for bool type

/* Forward declarations for required types */
typedef struct sd_card_hal        sd_card_hal_t;
typedef struct file_write_manager file_write_manager_t;


#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
// Include full definitions only if SD is enabled
#include "pstar_sd_card_hal.h"
#include "pstar_file_write_manager.h"
#endif

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the log storage system (if SD card logging is enabled).
 *
 * If CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED is set, this function initializes
 * the necessary components for storing logs on the SD card, including setting up
 * mutexes and registering for SD card availability callbacks.
 * If SD card logging is disabled, this function performs minimal setup (mutex creation).
 *
 * @param[in] manager The file write manager instance to use for writing logs.
 *                    Required if SD card logging is enabled, ignored otherwise.
 * @param[in] sd_card The SD card HAL instance to use.
 *                    Required if SD card logging is enabled, ignored otherwise.
 * @return
 *  - ESP_OK if successful.
 *  - ESP_ERR_INVALID_ARG if SD logging is enabled and manager or sd_card is NULL.
 *  - ESP_FAIL if mutex creation or callback registration fails.
 *  - ESP_ERR_INVALID_STATE if already initialized.
 */
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
esp_err_t log_storage_init(file_write_manager_t* manager, sd_card_hal_t* sd_card);
#else
esp_err_t log_storage_init(void* manager, void* sd_card); /* Parameters ignored */
#endif

/**
 * @brief Checks if the log storage component has been initialized.
 *
 * @return true if log_storage_init has completed successfully, false otherwise.
 */
bool log_storage_is_initialized(void);

/**
 * @brief Callback function for SD card availability changes.
 *
 * This function is called by the SD card HAL when the SD card availability
 * changes. It updates the internal state and triggers a log buffer flush if the SD
 * card becomes available and storage is initialized.
 *
 * @param[in] available Whether the SD card is available.
 */
void log_storage_set_sd_available(bool available);

/**
 * @brief Writes a log message to the storage buffer or directly to storage.
 *
 * If SD card logging is enabled and initialized:
 *  - If the SD card is available, the message is formatted and enqueued for writing.
 *  - If the SD card is not available, the message is stored in an internal buffer.
 *  - If the internal buffer is full, it attempts to flush; if flushing fails or
 *    is not possible, the oldest log entry in the buffer is discarded.
 * If SD card logging is disabled or storage is not initialized, this function
 * stores the message in the buffer (discarding oldest if full).
 *
 * @param[in] level   Log level.
 * @param[in] message Log message (already formatted including timestamp, task info, etc.).
 * @return
 *  - ESP_OK if the message was successfully buffered or enqueued.
 *  - ESP_ERR_INVALID_STATE if storage is not initialized or mutex error.
 *  - ESP_ERR_INVALID_ARG if message is NULL.
 *  - ESP_ERR_TIMEOUT if the mutex cannot be acquired.
 *  - Other error codes from underlying flush operations if buffer was full.
 */
esp_err_t log_storage_write(esp_log_level_t level, const char* message);


/**
 * @brief Flushes the log buffer to disk if SD card is available and initialized.
 *
 * If SD card logging is enabled, initialized, and the SD card is available,
 * this function attempts to write all buffered log messages to the current log file.
 *
 * @return
 *  - ESP_OK if flush was successful, or if SD logging is disabled/not initialized/SD not available.
 *  - ESP_ERR_INVALID_STATE if called before storage initialization (when SD enabled) or mutex error.
 *  - ESP_ERR_TIMEOUT if the mutex cannot be acquired.
 *  - Other error codes from the underlying file writing operations.
 */
esp_err_t log_storage_flush(void);

/**
 * @brief Sets whether log compression is enabled (Not runtime configurable).
 *
 * @param[in] enabled Whether compression is enabled.
 * @return ESP_OK (This is a no-op as compression is Kconfig controlled).
 */
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED && CONFIG_PSTAR_KCONFIG_LOGGING_COMPRESSION_ENABLED
esp_err_t log_storage_set_compression(bool enabled);

/**
 * @brief Checks if log compression is enabled via Kconfig.
 *
 * @return true if compression is enabled via Kconfig, false otherwise.
 */
bool log_storage_is_compression_enabled(void);
#else
/* Define stubs if compression is disabled */
static inline esp_err_t log_storage_set_compression(bool enabled) { (void)enabled; return ESP_OK; }
static inline bool log_storage_is_compression_enabled(void) { return false; }
#endif


/**
 * @brief Cleans up the log storage system.
 *
 * Performs cleanup of resources allocated during log storage initialization.
 * This includes:
 * 1. Attempting to flush any buffered logs (if SD card logging was enabled/init).
 * 2. Closing open files (handled by file manager).
 * 3. Freeing allocated memory (mutex).
 * 4. Resetting internal state.
 *
 * @return
 *  - ESP_OK if cleanup is successful.
 *  - ESP_ERR_TIMEOUT if mutex cannot be acquired during cleanup.
 *  - Other error codes if the final flush operation fails.
 */
esp_err_t log_storage_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_LOG_STORAGE_H */
