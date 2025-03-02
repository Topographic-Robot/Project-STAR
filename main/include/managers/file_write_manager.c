/* main/include/managers/file_write_manager.c */

/* TODO: Test this */

#include "file_write_manager.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "sd_card_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "log_handler.h"
#include "time_manager.h"

/* TODO: Implement directory creation support
 *
 * Currently, the file writer assumes all directories in the file path
 * already exist. If they don't, the file open operation will fail.
 *
 * Add a helper function to:
 * - Parse the file path to extract directory components
 * - Create each directory level if it doesn't exist
 * - Log success/failure of directory creation
 *
 * This will make the file writer more robust when dealing with
 * complex file paths.
 */

/* TODO: Implement a graceful shutdown mechanism for the file writer
 *
 * When power management is implemented, add a function to:
 * - Process any remaining items in the write queue
 * - Close any open files properly
 * - Free allocated resources
 * - Signal completion to allow safe power down
 *
 * This is important because sudden power loss (like unplugging the USB)
 * could lead to data corruption or loss, even with fsync() in place.
 */

/* TODO: Enhance error handling mechanisms
 *
 * Current error handling primarily logs errors but doesn't provide
 * recovery mechanisms. Consider implementing:
 *
 * 1. Retry mechanism: Attempt to retry failed operations with
 *    exponential backoff to handle transient failures
 *
 * 2. Fallback storage: If primary storage fails, attempt to write
 *    to a secondary location (e.g., internal flash if SD card fails)
 *
 * 3. Error statistics: Track error rates and types to help diagnose
 *    recurring issues
 *
 * 4. Watchdog integration: Reset the file writer task if it becomes
 *    unresponsive
 *
 * 5. Critical data prioritization: Ensure critical data gets written
 *    even when resources are constrained
 *
 * These improvements will make the system more resilient to failures
 * and provide better diagnostics for troubleshooting.
 */

/* Constants ******************************************************************/

const char    *file_manager_tag   = "FILE_MANAGER";
const uint32_t max_pending_writes = 10; /**< Maximum queued write operations */

/* Globals (Static) ***********************************************************/

static QueueHandle_t        s_file_write_queue       = NULL; /**< Queue for handling file write requests */
static TaskHandle_t         s_file_write_task_handle = NULL;
static file_writer_config_t s_file_writer_config     = { 3, 4096, false };

/* Private Functions **********************************************************/

/**
 * @brief Task to handle queued file write requests.
 *
 * Processes file write requests from a queue and writes data to the specified 
 * files. The task runs continuously, handling requests asynchronously.
 *
 * @param[in] param Pointer to task-specific parameters (unused)
 *
 * @note 
 * - This function is intended to run as a FreeRTOS task.
 * - Each write request includes a timestamp and data to be written.
 */
static void priv_file_write_task(void *param)
{
  log_info(file_manager_tag, "Task Start", "File writer task started, processing queue");
  file_write_request_t request;

  while (1) {
    if (xQueueReceive(s_file_write_queue, &request, portMAX_DELAY) == pdTRUE) {
      log_info(file_manager_tag, "Write Start", "Processing write request for file: %s", request.file_path);
      
      FILE *file = fopen(request.file_path, "a");
      if (file == NULL) {
        log_error(file_manager_tag, "File Error", "Failed to open file: %s", request.file_path);
        continue;
      }

      size_t bytes_written = fwrite(request.data, 1, strlen(request.data), file);
      fflush(file);
      fsync(fileno(file));
      fclose(file);

      if (bytes_written != strlen(request.data)) {
        log_error(file_manager_tag, "Write Error", "Incomplete write to %s: %zu/%zu bytes written", 
                 request.file_path, bytes_written, strlen(request.data));
      } else {
        log_info(file_manager_tag, "Write Success", "Successfully wrote %zu bytes to %s", 
                 bytes_written, request.file_path);
      }
    }
  }
}

/* Public Functions ***********************************************************/

esp_err_t file_write_manager_init(void)
{
  if (!s_file_writer_config.enabled) {
    log_info(file_manager_tag, "Init Skip", "File writer initialization skipped (disabled in configuration)");
    return ESP_OK;
  }

  log_info(file_manager_tag, "Init Start", "Beginning file writer initialization");

  s_file_write_queue = xQueueCreate(max_pending_writes, sizeof(file_write_request_t));
  if (s_file_write_queue == NULL) {
    log_error(file_manager_tag, "Queue Error", "Failed to create write queue: insufficient memory");
    return ESP_FAIL;
  }
  log_info(file_manager_tag, "Queue Create", "Write queue created with capacity: %lu requests", max_pending_writes);

  BaseType_t task_created = xTaskCreate(priv_file_write_task,
                                        file_manager_tag,
                                        s_file_writer_config.stack_depth,
                                        NULL,
                                        s_file_writer_config.priority,
                                        &s_file_write_task_handle);
  if (task_created != pdPASS) {
    log_error(file_manager_tag, "Task Error", "Failed to create write task: insufficient resources");
    return ESP_FAIL;
  }
  log_info(file_manager_tag, "Task Create", "Write task created with priority %u", s_file_writer_config.priority);

  if (sd_card_init() != ESP_OK) {
    log_error(file_manager_tag, "SD Error", "Failed to initialize SD card storage");
    return ESP_FAIL;
  }
  log_info(file_manager_tag, "SD Ready", "SD card storage system initialized");

  log_info(file_manager_tag, "Init Complete", "File writer system initialization successful");
  return ESP_OK;
}

esp_err_t file_write_enqueue(const char *file_path, const char *data)
{
  if (!s_file_writer_config.enabled) {
    log_warn(file_manager_tag, "Write Skip", "Write request rejected: file writer is disabled");
    return ESP_FAIL;
  }

  if (file_path == NULL || data == NULL) {
    log_error(file_manager_tag, "Param Error", "Invalid parameters: file_path or data is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  if (s_file_write_queue == NULL) {
    log_error(file_manager_tag, "Queue Error", "Write request rejected: queue not initialized");
    return ESP_FAIL;
  }

  file_write_request_t request;
  char *timestamp = time_manager_get_timestamp();
  if (timestamp == NULL) {
    log_error(file_manager_tag, "Memory Error", "Failed to get timestamp");
    return ESP_FAIL;
  }
  
  /* Handle file path with or without leading slash */
  const char *format = (file_path[0] == '/') ? "%s%s" : "%s/%s";
  int result = snprintf(request.file_path, MAX_FILE_PATH_LENGTH, format, sd_card_mount_path, file_path);
  if (result >= MAX_FILE_PATH_LENGTH) {
    log_error(file_manager_tag, "Path Error", "File path too long");
    free(timestamp);
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the combined timestamp and data will fit in the buffer */
  size_t timestamp_len = strlen(timestamp);
  size_t data_len = strlen(data);
  if (timestamp_len + data_len + 2 > MAX_DATA_LENGTH) { /* +2 for space and newline */
    log_error(file_manager_tag, "Data Error", "Combined data too large for buffer");
    free(timestamp);
    return ESP_ERR_INVALID_ARG;
  }
  
  snprintf(request.data, MAX_DATA_LENGTH, "%s %s\n", timestamp, data);

  /* Free the dynamically allocated timestamp */
  free(timestamp);

  log_info(file_manager_tag, "Queue Write", "Enqueueing write request for file: %s", file_path);
  if (xQueueSend(s_file_write_queue, &request, 0) != pdTRUE) {
    log_error(file_manager_tag, "Queue Error", "Write request rejected: queue is full");
    return ESP_FAIL;
  }

  log_info(file_manager_tag, "Queue Success", "Write request successfully queued for processing");
  return ESP_OK;
}

