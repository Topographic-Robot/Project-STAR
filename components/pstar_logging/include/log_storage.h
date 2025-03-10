/* components/pstar_logging/include/log_storage.h */

#ifndef PSTAR_LOG_STORAGE_H
#define PSTAR_LOG_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_log.h"
#include "sd_card_hal.h"
#include "file_write_manager.h"
#include "log_types.h"

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the log storage system
 * 
 * @param[in] manager The file write manager instance to use for writing logs
 * @param[in] sd_card The SD card HAL instance to use
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_storage_init(file_write_manager_t* manager, sd_card_hal_t* sd_card);

/**
 * @brief Callback function for SD card availability changes
 * 
 * This function is called by the SD card HAL when the SD card availability 
 * changes. It updates the internal state and flushes the log buffer if the SD 
 * card becomes available.
 * 
 * @param[in] available Whether the SD card is available
 */
void log_storage_set_sd_available(bool available);

/**
 * @brief Writes a log message to storage
 * 
 * @param[in] level   Log level
 * @param[in] message Log message
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_storage_write(esp_log_level_t level, const char* message);

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

/**
 * @brief Cleans up the log storage system
 * 
 * Performs cleanup of resources allocated during log storage initialization.
 * This includes:
 * 1. Flushing any buffered logs
 * 2. Closing open files
 * 3. Freeing allocated memory
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t log_storage_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_LOG_STORAGE_H */
