/* components/pstar_managers/include/file_write_manager.h */

#ifndef PSTAR_FILE_WRITE_MANAGER_H
#define PSTAR_FILE_WRITE_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include <stdatomic.h>

/* Forward declarations */
#if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
typedef struct sd_card_hal sd_card_hal_t;
#endif

/* Structs ********************************************************************/

/**
 * @brief Represents a request to write data to a file.
 *
 * Contains the file path, data, and data length for a file write operation.
 * The is_binary flag indicates whether the data should be treated as binary or text.
 *
 * @note
 * - The `file_path` must be null-terminated and have a length defined by
 *   CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH to prevent memory overflow.
 * - For text data (is_binary = false), the `data` field contains a pointer to the
 *   dynamically allocated null-terminated string.
 * - For binary data (is_binary = true), the `data` field contains a pointer to
 *   the dynamically allocated binary data buffer.
 * - The `data` pointer MUST be freed by the consumer (file write task) after processing.
 */
typedef struct file_write_request {
  char     file_path[CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH]; /**< Path to the target file. Must be null-terminated and within the length limit. */
  void*    data;                                                         /**< Pointer to the dynamically allocated data to write. */
  uint32_t data_length;                                                  /**< Length of the data in bytes. */
  bool     is_binary;                                                    /**< Flag indicating if this is a binary write request (true) or text (false). */
  bool     is_terminate_request;                                         /**< Special flag to signal task termination */
} file_write_request_t;

/**
 * @brief Configuration structure for the file write manager task.
 *
 * Contains settings for the file write manager task including its
 * priority and stack size.
 */
typedef struct file_writer_config {
  UBaseType_t priority;    /**< Priority of the file writer task for scheduling purposes. */
  uint32_t    stack_depth; /**< Stack depth allocated for the file writer task, in words. */
} file_writer_config_t;

/**
 * @brief File write manager structure that encapsulates state.
 *
 * Contains all the state needed for the file write manager to operate, including
 * queues, tasks, and the associated SD card instance.
 */
typedef struct file_write_manager {
  QueueHandle_t        file_write_queue; /**< Queue for file write requests. */
  TaskHandle_t         file_write_task;  /**< Task handle for the file writer. */
#if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  sd_card_hal_t*       sd_card;          /**< Pointer to the SD card HAL instance. */
#endif
  _Atomic bool         initialized;      /**< Flag indicating if the manager is initialized. */
  _Atomic bool         cleanup_requested;/**< Flag to signal cleanup sequence is active */
  file_writer_config_t config;           /**< Configuration for the file writer. */
  SemaphoreHandle_t    task_exit_sem;    /**< Semaphore signaled when task exits */
} file_write_manager_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the file write manager.
 *
 * Sets up a FreeRTOS queue for managing asynchronous file write requests
 * and starts a background task to process them. Files are always opened
 * in append mode, creating them if they do not exist. For text writes,
 * each line of data written to a file will include a timestamp at the start
 * in the format `YYYY-MM-DD HH:MM:SS`.
 *
 * @param[in,out] manager Pointer to the file write manager structure to initialize.
 * @param[in]     sd_card Pointer to the SD card HAL instance to use (if SD enabled).
 * @param[in]     config  Configuration for the file writer task. If NULL,
 *                        default values from Kconfig will be used.
 *
 * @return
 * - ESP_OK              if the initialization is successful.
 * - ESP_ERR_INVALID_ARG if any required argument is invalid (e.g., NULL pointers).
 * - ESP_ERR_NO_MEM      if memory allocation fails (queue, task, semaphore).
 * - ESP_FAIL            if task creation fails.
 * - ESP_ERR_NOT_SUPPORTED if SD card is disabled but required.
 *
 * @note This function must be called before attempting to enqueue file
 *       write requests. The HAL pointers (`sd_card`) must remain valid for the
 *       lifetime of the file write manager.
 */
esp_err_t file_write_manager_init(file_write_manager_t*       manager,
#if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
                                  sd_card_hal_t*              sd_card,
#else
                                  void*                       sd_card,
#endif
                                  const file_writer_config_t* config);

/**
 * @brief Enqueues a file write request.
 *
 * Adds a file write request to the queue for asynchronous processing.
 * The data will be written to the specified file in the background by
 * the file write task. If the file does not exist, it will be created
 * automatically (including parent directories). All writes append data to the file.
 * Each line written includes a timestamp at the beginning in the format `YYYY-MM-DD HH:MM:SS`.
 *
 * @param[in] manager   Pointer to the initialized file write manager.
 * @param[in] file_path Path to the file relative to the SD card mount point
 *                      (e.g., "logs/sensor1.txt"). Must be null-terminated and
 *                      adhere to path safety rules.
 * @param[in] data      Null-terminated string to write to the file.
 *
 * @return
 * - ESP_OK              if the request was successfully enqueued.
 * - ESP_ERR_INVALID_ARG if any argument is invalid (e.g., NULL pointers, unsafe path).
 * - ESP_ERR_INVALID_STATE if the manager isn't initialized or is cleaning up.
 * - ESP_ERR_NO_MEM      if memory allocation for the request data fails.
 * - ESP_FAIL            if the queue is full.
 * - ESP_ERR_NOT_SUPPORTED if SD card is disabled.
 *
 * @note The function does not block but returns immediately after enqueueing
 *       the request. The `data` string is copied internally, so the caller
 *       does not need to keep it valid after the call.
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
 * automatically (including parent directories). All writes append data to the file.
 * Binary writes do NOT include a timestamp.
 *
 * @param[in] manager     Pointer to the initialized file write manager.
 * @param[in] file_path   Path to the file relative to the SD card mount point
 *                        (e.g., "data/image.bin"). Must be null-terminated and
 *                        adhere to path safety rules.
 * @param[in] data        Pointer to the binary data to write.
 * @param[in] data_length Length of the binary data in bytes.
 *
 * @return
 * - ESP_OK              if the request was successfully enqueued.
 * - ESP_ERR_INVALID_ARG if any argument is invalid (e.g., NULL pointers, zero length, unsafe path).
 * - ESP_ERR_INVALID_STATE if the manager isn't initialized or is cleaning up.
 * - ESP_ERR_NO_MEM      if memory allocation for the request data fails.
 * - ESP_FAIL            if the queue is full.
 * - ESP_ERR_NOT_SUPPORTED if SD card is disabled.
 *
 * @note The function does not block but returns immediately after enqueueing
 *       the request. The `data` buffer is copied internally, so the caller
 *       does not need to keep it valid after the call.
 */
esp_err_t file_write_binary_enqueue(file_write_manager_t* manager,
                                    const char* const     file_path,
                                    const void* const     data,
                                    uint32_t              data_length);

/**
 * @brief Cleans up resources used by the file write manager.
 *
 * Signals the file write task to stop, processes any remaining items in the queue,
 * waits for the task to exit, and frees all allocated resources (queue, semaphore).
 * This function should be called during system shutdown to ensure proper cleanup
 * of the file write subsystem and prevent data loss or resource leaks.
 *
 * @param[in,out] manager Pointer to the initialized file write manager.
 *
 * @return
 * - ESP_OK              if all resources are successfully cleaned up.
 * - ESP_ERR_INVALID_ARG if manager is NULL.
 * - ESP_ERR_INVALID_STATE if called before init or already cleaned up.
 * - ESP_ERR_TIMEOUT     if waiting for the task to exit times out.
 * - ESP_FAIL            if any internal cleanup operation fails.
 *
 * @note
 * - Call this function during the system shutdown phase.
 * - This function will attempt to process any pending write requests in the
 *   queue before shutting down to minimize data loss.
 * - Ensure all components that use the file write manager have stopped
 *   enqueueing new requests before calling this function.
 * - This function does NOT free the manager pointer itself, only its internal resources.
 */
esp_err_t file_write_manager_cleanup(file_write_manager_t* manager);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_FILE_WRITE_MANAGER_H */