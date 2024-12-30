/* main/include/managers/include/file_write_manager.h */

#ifndef TOPOROBO_FILE_WRITE_MANAGER_H
#define TOPOROBO_FILE_WRITE_MANAGER_H

#include "esp_err.h"

/* Constants ******************************************************************/

extern const char    *file_manager_tag;   /**< Logging tag for ESP_LOG messages related to the file write manager. */
extern const uint32_t max_pending_writes; /**< Maximum number of queued file write requests to prevent overflow. */

/* Macros *********************************************************************/

#define max_file_path_length (64)  /**< Maximum file path length, including the null terminator. */
#define max_data_length      (256) /**< Maximum data length per write request, including the null terminator. */

/* Structs ********************************************************************/

/**
 * @struct file_write_request_t
 * @brief Structure representing a file write request.
 *
 * This structure encapsulates the necessary information for writing data to a file,
 * including the file path and the data to be written.
 *
 * **Fields:**
 * - `file_path`: The path to the file where data should be written. The length is defined
 *   by the `max_file_path_length` constant to ensure proper memory allocation and avoid overflow.
 * - `data`: The content to be written to the file. The length is defined by the
 *   `max_data_length` constant to ensure proper memory allocation and avoid overflow.
 *
 * **Usage Notes:**
 * - Ensure that `file_path` is null-terminated and points to a valid path.
 * - `data` must be null-terminated if it contains string data.
 */
typedef struct {
  char file_path[max_file_path_length]; /**< Path to the target file. */
  char data[max_data_length];           /**< Data to be written to the file. */
} file_write_request_t;

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

