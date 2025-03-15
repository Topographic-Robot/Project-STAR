/* main/pstar_managers/include/file_write_manager.h */

#ifndef PSTAR_FILE_WRITE_MANAGER_H
#define PSTAR_FILE_WRITE_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

/* Forward declarations */
typedef struct sd_card_hal sd_card_hal_t;

/* Macros *********************************************************************/

#define MAX_FILE_PATH_LENGTH  (256) /**< Maximum file path length, including the null terminator. */
#define MAX_DATA_LENGTH       (256) /**< Maximum data length per write request, including the null terminator. */
#define TIMESTAMP_BUFFER_SIZE (64)  /**< Size of buffer for timestamp strings. */

/* Structs ********************************************************************/

/**
 * @brief Represents a request to write data to a file.
 *
 * Contains the file path, data, and data length for a file write operation.
 * The is_binary flag indicates whether the data should be treated as binary or text.
 * 
 * @note
 * - The `file_path` must be null-terminated and have a length defined by 
 *   the `max_file_path_length` macro to prevent memory overflow.
 * - For text data (is_binary = false), the data field contains the 
 *   null-terminated string.
 * - For binary data (is_binary = true), the data field contains a pointer to 
 *   the binary data.
 */
typedef struct file_write_request {
  char     file_path[MAX_FILE_PATH_LENGTH]; /**< Path to the target file. Must be null-terminated and within the length limit. */
  void*    data;                            /**< Pointer to the data to write. For text, this is a null-terminated string. */
  uint32_t data_length;                     /**< Length of the data in bytes. For text, this can be 0 (will use strlen). */
  bool     is_binary;                       /**< Flag indicating if this is a binary write request (true) or text (false). */
} file_write_request_t;

/**
 * @brief Configuration structure for the file write manager task.
 *
 * Contains settings for the file write manager task including its name,
 * priority, stack size, and enablement flag.
 */
typedef struct file_writer_config {
  UBaseType_t priority;    /**< Priority of the file writer task for scheduling purposes. */
  uint32_t    stack_depth; /**< Stack depth allocated for the file writer task, in words. */
  bool        enabled;     /**< Flag indicating if the file writer is enabled (true) or disabled (false). */
} file_writer_config_t;

/**
 * @brief File write manager structure that encapsulates state.
 * 
 * Contains all the state needed for the file write manager to operate, including
 * queues, tasks, and the associated SD card instance.
 */
typedef struct file_write_manager {
  QueueHandle_t    file_write_queue; /**< Queue for file write requests. */
  TaskHandle_t     file_write_task;  /**< Task handle for the file writer. */
  sd_card_hal_t*   sd_card;          /**< Pointer to the SD card HAL instance. */
  bool             initialized;      /**< Flag indicating if the manager is initialized. */
  file_writer_config_t config;       /**< Configuration for the file writer. */
} file_write_manager_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the file write manager.
 *
 * Sets up a FreeRTOS queue for managing asynchronous file write requests 
 * and starts a background task to process them. Files are always opened 
 * in append mode, creating them if they do not exist. Each line of data 
 * written to a file will include a timestamp at the start in the format 
 * `YYYY-MM-DD HH:MM:SS`.
 *
 * @param[in,out] manager Pointer to the file write manager structure to initialize.
 * @param[in]     sd_card Pointer to the SD card HAL instance to use.
 * @param[in]     config  Configuration for the file writer task. If NULL, 
 *                        default values will be used.
 *
 * @return
 * - ESP_OK              if the initialization is successful.
 * - ESP_ERR_INVALID_ARG if any argument is invalid (e.g., NULL pointers).
 * - ESP_FAIL            if the queue or task creation fails.
 *
 * @note This function must be called before attempting to enqueue file 
 *       write requests.
 */
esp_err_t file_write_manager_init(file_write_manager_t*       manager, 
                                  sd_card_hal_t*              sd_card,
                                  const file_writer_config_t* config);

/**
 * @brief Enqueues a file write request.
 *
 * Adds a file write request to the queue for asynchronous processing. 
 * The data will be written to the specified file in the background by 
 * the file write task. If the file does not exist, it will be created 
 * automatically. All writes append data to the file. Each line written 
 * includes a timestamp at the beginning in the format `YYYY-MM-DD HH:MM:SS`.
 *
 * @param[in] manager   Pointer to the initialized file write manager.
 * @param[in] file_path Path to the file (e.g., "/logs/sensor1.txt"). 
 *                      This must be a valid file path accessible by 
 *                      the system.
 * @param[in] data      Null-terminated string to write to the file. The 
 *                      string should not exceed the maximum size allowed 
 *                      by the queue.
 *
 * @return
 * - ESP_OK              if the request was successfully enqueued.
 * - ESP_ERR_INVALID_ARG if any argument is invalid (e.g., NULL pointers).
 * - ESP_FAIL            if the queue is full or the manager isn't initialized.
 *
 * @note The function does not block but returns immediately after enqueueing 
 *       the request. The data is copied to an internal buffer, so the caller 
 *       can free the original data after this function returns.
 */
esp_err_t file_write_enqueue(file_write_manager_t* manager,
                             const char* const     file_path, 
                             const char* const     data);

/**
 * @brief Enqueues a binary file write request.
 *
 * Adds a binary file write request to the queue for asynchronous processing.
 * The binary data will be written to the specified file in the background by
 * the file write task. If the file does not exist, it will be created
 * automatically. All writes append data to the file.
 *
 * @param[in] manager     Pointer to the initialized file write manager.
 * @param[in] file_path   Path to the file (e.g., "/logs/data.bin").
 *                        This must be a valid file path accessible by
 *                        the system.
 * @param[in] data        Pointer to the binary data to write.
 * @param[in] data_length Length of the binary data in bytes.
 *
 * @return
 * - ESP_OK              if the request was successfully enqueued.
 * - ESP_ERR_INVALID_ARG if any argument is invalid (e.g., NULL pointers).
 * - ESP_FAIL            if the queue is full or the manager isn't initialized.
 *
 * @note The function does not block but returns immediately after enqueueing
 *       the request. The data is copied to an internal buffer, so the caller
 *       can free the original data after this function returns.
 */
esp_err_t file_write_binary_enqueue(file_write_manager_t* manager,
                                    const char* const     file_path, 
                                    const void* const     data, 
                                    uint32_t              data_length);

/**
 * @brief Cleans up resources used by the file write manager.
 *
 * Stops the file write task, processes any remaining items in the queue,
 * and frees all allocated resources. This function should be called during
 * system shutdown to ensure proper cleanup of the file write subsystem and
 * prevent data loss.
 *
 * @param[in,out] manager Pointer to the initialized file write manager.
 *
 * @return
 * - ESP_OK              if all resources are successfully cleaned up.
 * - ESP_ERR_INVALID_ARG if manager is NULL.
 * - ESP_FAIL            if any cleanup operation fails.
 *
 * @note
 * - Call this function during the system shutdown phase.
 * - This function will attempt to process any pending write requests in the
 *   queue before shutting down to prevent data loss.
 * - Ensure all components that use the file write manager have stopped
 *   enqueueing new requests before calling this function.
 * - This function does NOT free the manager pointer itself, only its resources.
 */
esp_err_t file_write_manager_cleanup(file_write_manager_t* manager);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_FILE_WRITE_MANAGER_H */
