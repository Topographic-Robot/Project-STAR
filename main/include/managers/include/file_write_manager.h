/* main/include/managers/include/file_write_manager.h */

#ifndef TOPOROBO_FILE_WRITE_MANAGER_H
#define TOPOROBO_FILE_WRITE_MANAGER_H

#include "esp_err.h"

/* Constants ******************************************************************/

/**
 * @brief Logging tag for the file write manager.
 *
 * This tag is used in ESP_LOG messages to categorize log output
 * related to the file write manager. It simplifies debugging and
 * log filtering.
 */
extern const char *file_manager_tag;

/**
 * @brief Maximum number of queued file write requests.
 *
 * Defines the maximum number of pending write operations that
 * can be enqueued. Requests beyond this limit will be rejected
 * to prevent queue overflow.
 */
extern const uint32_t max_pending_writes;

/* Macros *********************************************************************/

/**
 * @brief Maximum file path length.
 *
 * Defines the maximum number of characters (including the null terminator)
 * that can be used for a file path in the file write manager.
 */
#define max_file_path_length (64)

/**
 * @brief Maximum data length per write request.
 *
 * Specifies the maximum number of characters (including the null terminator)
 * that can be included in the data for a single write request.
 */
#define max_data_length (256)

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the file write manager.
 *
 * This function creates a FreeRTOS queue for managing asynchronous
 * file write requests and starts a background task to handle the queued
 * requests. Files are always opened in append mode, creating them
 * if they do not exist. Each line of data written will include a
 * timestamp at the start.
 *
 * @return
 * - ESP_OK if the initialization is successful.
 * - ESP_FAIL if the queue creation fails.
 */
esp_err_t file_write_manager_init(void);

/**
 * @brief Enqueues a file write request.
 *
 * This function adds a file write request to the queue. The data will
 * be written to the specified file in the background by the file write task.
 * If the file does not exist, it will be created automatically. All writes
 * append data to the file. Each line written includes a timestamp at the
 * beginning in the format `YYYY-MM-DD HH:MM:SS`.
 *
 * @param[in] file_path Path to the file (e.g., "/sdcard/sensor1.txt").
 * @param[in] data Null-terminated string to write to the file.
 *
 * @return
 * - ESP_OK if the request was successfully enqueued.
 * - ESP_ERR_INVALID_ARG if arguments are invalid.
 * - ESP_FAIL if the queue is full.
 */
esp_err_t file_write_enqueue(const char *file_path, const char *data);

#endif /* TOPOROBO_FILE_WRITE_MANAGER_H */

