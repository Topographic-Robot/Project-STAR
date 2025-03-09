/* main/include/managers/file_write_manager.c */

/* TODO: Test this */
/* TODO: Don't use a buffer, but like dynamically allocate memory for the file path and data */

#include "file_write_manager.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "sd_card_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "log_handler.h"
#include "time_manager.h"

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
 * 2. Fallback storage options: If SD card fails, try to use flash
 *    memory or other storage options
 * 3. Error statistics: Track error rates and types for diagnostics
 * 4. Watchdog integration: Reset the system if file operations
 *    consistently fail
 * 5. Critical data prioritization: Ensure critical data is written
 *    first and with higher reliability guarantees
 */

/* Constants ******************************************************************/

const char* const file_manager_tag = "File Manager";
const uint32_t    max_pending_writes = 20;

/* Globals (Static) ***********************************************************/

static QueueHandle_t s_file_write_queue = NULL;
static TaskHandle_t  s_file_write_task  = NULL;
static bool          s_initialized      = false;

/* Private Functions **********************************************************/

/**
 * @brief Creates all directories in a file path if they don't exist
 * 
 * @param[in] file_path Full path to the file (including SD card mount path)
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_create_directories(const char* const file_path)
{
  if (file_path == NULL) {
    log_error(file_manager_tag, "Dir Create Error", "Invalid file path (NULL)");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Make a copy of the path that we can modify */
  char path_copy[MAX_FILE_PATH_LENGTH * 2];
  strncpy(path_copy, file_path, sizeof(path_copy) - 1);
  path_copy[sizeof(path_copy) - 1] = '\0';
  
  /* Find the last slash in the path to get the directory part */
  char* last_slash = strrchr(path_copy, '/');
  if (last_slash == NULL) {
    /* No directory component, nothing to create */
    return ESP_OK;
  }
  
  /* Terminate the string at the last slash to get just the directory path */
  *last_slash = '\0';
  
  /* If the directory path is empty, nothing to create */
  if (strlen(path_copy) == 0) {
    return ESP_OK;
  }
  
  /* Create the directory path recursively */
  char* p = path_copy;
  
  /* Skip leading slashes */
  while (*p == '/') {
    p++;
  }
  
  while (p && *p) { /* while there is a path and the path is not empty */
    /* Find the next slash */
    char* next_slash = strchr(p, '/');
    if (next_slash) {
      *next_slash = '\0'; /* Temporarily terminate the string at this slash */
    }
    
    /* Create the directory up to this point */
    struct stat st;
    if (stat(path_copy, &st) != 0) {
      /* Directory doesn't exist, create it */
      if (mkdir(path_copy, 0755) != 0) {
        log_error(file_manager_tag, 
                  "Dir Create Failed", 
                  "Failed to create directory: %s (errno: %d)", 
                  path_copy, 
                  errno);
        return ESP_FAIL;
      }
      log_debug(file_manager_tag, "Dir Created", "Created directory: %s", path_copy);
    } else if (!S_ISDIR(st.st_mode)) {
      /* Path exists but is not a directory */
      log_error(file_manager_tag, 
                "Dir Create Failed", 
                "Path exists but is not a directory: %s", 
                path_copy);
      return ESP_FAIL;
    }
    
    /* Restore the slash and move to the next component */
    if (next_slash) {
      *next_slash = '/';
      p           = next_slash + 1;
    } else {
      break;
    }
  }
  
  return ESP_OK;
}

/**
 * @brief Writes a string to a file with timestamp
 * 
 * @param[in] file_path Path to the file
 * @param[in] data      String to write
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_write_to_file(const char* const file_path, 
                                    const char* const data)
{
  if (file_path == NULL || data == NULL) {
    log_error(file_manager_tag, 
              "Write Error", 
              "Invalid arguments: file_path or data is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if SD card is available */
  if (!sd_card_is_available()) {
    log_error(file_manager_tag, 
              "SD Card Error", 
              "SD card not available, cannot write to file: %s", 
              file_path);
    return ESP_FAIL;
  }
  
  /* Prepend the SD card mount path to the file path */
  char full_path[MAX_FILE_PATH_LENGTH * 2];
  snprintf(full_path, sizeof(full_path), "%s/%s", sd_card_mount_path, file_path);
  
  /* Create directories if they don't exist */
  esp_err_t ret = priv_create_directories(full_path);
  if (ret != ESP_OK) {
    log_error(file_manager_tag, 
              "Dir Create Error", 
              "Failed to create directories for file: %s", 
              full_path);
    return ret;
  }
  
  /* Open the file for writing (append mode) */
  FILE* file = fopen(full_path, "a");
  if (file == NULL) {
    log_error(file_manager_tag, 
              "File Open Error", 
              "Failed to open file: %s (errno: %d)",
              full_path, 
              errno);
    return ESP_FAIL;
  }
  
  /* Get current timestamp */
  char      timestamp[TIMESTAMP_BUFFER_SIZE];
  time_t    now      = time(NULL);
  struct tm timeinfo = { 0 };
  localtime_r(&now, &timeinfo);
  
  /* Format timestamp */
  snprintf(timestamp, 
           sizeof(timestamp), 
           "%04d-%02d-%02d %02d:%02d:%02d",
           timeinfo.tm_year + 1900, 
           timeinfo.tm_mon + 1, 
           timeinfo.tm_mday,
           timeinfo.tm_hour, 
           timeinfo.tm_min, 
           timeinfo.tm_sec);
  
  /* Write timestamp and data to file */
  fprintf(file, "%s: %s\n", timestamp, data);
  
  /* Ensure data is written to disk */
  fflush(file);
  fsync(fileno(file));
  
  /* Close file */
  fclose(file);
  
  log_debug(file_manager_tag, 
            "Write Success", 
            "Data written to file: %s", 
            file_path);
  return ESP_OK;
}

/**
 * @brief Writes binary data to a file
 * 
 * @param[in] file_path   Path to the file
 * @param[in] data        Binary data to write
 * @param[in] data_length Length of the binary data
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_write_binary_to_file(const char* const file_path, 
                                           const void* const data, 
                                           uint32_t          data_length)
{
  if (file_path == NULL || data == NULL || data_length == 0) {
    log_error(file_manager_tag, 
              "Binary Write Error", 
              "Invalid arguments: file_path or data is NULL, or data_length is 0");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if SD card is available */
  if (!sd_card_is_available()) {
    log_error(file_manager_tag, 
              "SD Card Error", 
              "SD card not available, cannot write binary data to file: %s", 
              file_path);
    return ESP_FAIL;
  }
  
  /* Prepend the SD card mount path to the file path */
  char full_path[MAX_FILE_PATH_LENGTH * 2];
  snprintf(full_path, sizeof(full_path), "%s/%s", sd_card_mount_path, file_path);
  
  /* Create directories if they don't exist */
  esp_err_t ret = priv_create_directories(full_path);
  if (ret != ESP_OK) {
    log_error(file_manager_tag, 
              "Dir Create Error", 
              "Failed to create directories for binary file: %s", 
              full_path);
    return ret;
  }
  
  /* Open the file for writing (binary mode) */
  FILE* file = fopen(full_path, "wb");
  if (file == NULL) {
    log_error(file_manager_tag, 
              "Binary File Open Error", 
              "Failed to open file: %s (errno: %d)",
              full_path, 
              errno);
    return ESP_FAIL;
  }
  
  /* Write the binary data to the file */
  size_t bytes_written = fwrite(data, 1, data_length, file);
  if (bytes_written != data_length) {
    log_error(file_manager_tag, 
              "Binary Write Error", 
              "Failed to write all data: %zu of %lu bytes written",
              bytes_written, 
              data_length);
  }
  
  /* Ensure data is written to disk */
  fflush(file);
  fsync(fileno(file));
  
  /* Close the file */
  fclose(file);
  
  log_debug(file_manager_tag, 
            "Binary Write Success", 
            "Binary data written to file: %s (%lu bytes)",
            file_path, 
            data_length);
  return ESP_OK;
}

/**
 * @brief Task that processes file write requests from the queue
 * 
 * @param[in] param Task parameters (unused)
 */
static void priv_file_write_task(void* param)
{
  log_info(file_manager_tag, "Task Start", "File write task started");
  
  while (1) {
    /* Wait for a request from the queue */
    file_write_request_t request;
    
    if (xQueueReceive(s_file_write_queue, &request, portMAX_DELAY) == pdTRUE) {
      /* Check if this is a binary request */
      if (request.is_binary) {
        /* Process binary write request */
        priv_write_binary_to_file(request.file_path,
                                  request.data,
                                  request.data_length);
        
        /* Free the allocated data buffer */
        if (request.data) {
          free(request.data);
          request.data = NULL;
        }
      } else {
        /* Process text write request */
        priv_write_to_file(request.file_path, (const char* const)request.data);
        
        /* Free the allocated data buffer */
        if (request.data) {
          free(request.data);
          request.data = NULL;
        }
      }
    }
  }
}

/* Public Functions ***********************************************************/

esp_err_t file_write_manager_init(void)
{
  if (s_initialized) {
    log_warn(file_manager_tag, 
             "Init Skip", 
             "File write manager already initialized");
    return ESP_OK;
  }
  
  log_info(file_manager_tag, "Init Start", "Initializing file write manager");
  
  /* Create queue for file write requests */
  s_file_write_queue = xQueueCreate(max_pending_writes, sizeof(file_write_request_t));
  if (s_file_write_queue == NULL) {
    log_error(file_manager_tag, "Queue Error", "Failed to create file write queue");
    return ESP_FAIL;
  }
  
  /* Initialize SD card detection system */
  esp_err_t ret = sd_card_detection_init();
  if (ret != ESP_OK) {
    log_warn(file_manager_tag, 
             "SD Card Warning", 
             "SD card detection system initialization failed, file writes may fail");
  }
  
  /* Create task to process file write requests */
  BaseType_t task_created = xTaskCreate(priv_file_write_task,
                                        "file_write_task",
                                        4096,
                                        NULL,
                                        5,
                                        &s_file_write_task);
  
  if (task_created != pdPASS) {
    log_error(file_manager_tag, "Task Error", "Failed to create file write task");
    vQueueDelete(s_file_write_queue);
    s_file_write_queue = NULL;
    return ESP_FAIL;
  }
  
  s_initialized = true;
  log_info(file_manager_tag, 
           "Init Complete", 
           "File write manager initialized successfully");
  return ESP_OK;
}

esp_err_t file_write_enqueue(const char* const file_path, 
                             const char* const data)
{
  if (!s_initialized) {
    log_error(file_manager_tag, 
              "Enqueue Error", 
              "File write manager not initialized");
    return ESP_FAIL;
  }
  
  if (!file_path || !data) {
    log_error(file_manager_tag, 
              "Enqueue Error", 
              "Invalid arguments: file_path or data is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Create a request */
  file_write_request_t request;
  memset(&request, 0, sizeof(request));
  
  /* Copy file path to the request */
  strncpy(request.file_path, file_path, MAX_FILE_PATH_LENGTH - 1);
  request.file_path[MAX_FILE_PATH_LENGTH - 1] = '\0';
  
  /* Allocate memory for the text data and copy it */
  size_t data_length = strlen(data) + 1; /* Include null terminator */
  request.data       = malloc(data_length);
  if (!request.data) {
    log_error(file_manager_tag, 
              "Memory Error", 
              "Failed to allocate memory for text data");
    return ESP_FAIL;
  }
  
  memcpy(request.data, data, data_length);
  request.data_length = data_length;
  request.is_binary   = false; /* This is a text request */
  
  /* Send the request to the queue */
  if (xQueueSend(s_file_write_queue, &request, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(file_manager_tag, 
              "Queue Error", 
              "Failed to enqueue file write request: queue full");
    free(request.data);
    return ESP_FAIL;
  }
  
  log_debug(file_manager_tag, 
            "Enqueue Success", 
            "Enqueued write request for file: %s", 
            file_path);
  return ESP_OK;
}

esp_err_t file_write_binary_enqueue(const char* const file_path, 
                                    const void* const data, 
                                    uint32_t          data_length)
{
  if (!s_initialized) {
    log_error(file_manager_tag, 
              "Binary Enqueue Error", 
              "File write manager not initialized");
    return ESP_FAIL;
  }
  
  if (!file_path || !data || data_length == 0) {
    log_error(file_manager_tag, 
              "Binary Enqueue Error", 
              "Invalid arguments: file_path or data is NULL, or data_length is 0");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Create a binary request */
  file_write_request_t request;
  memset(&request, 0, sizeof(request));
  
  /* Copy file path to the request */
  strncpy(request.file_path, file_path, MAX_FILE_PATH_LENGTH - 1);
  request.file_path[MAX_FILE_PATH_LENGTH - 1] = '\0';
  
  /* Allocate memory for the binary data and copy it */
  request.data = malloc(data_length);
  if (!request.data) {
    log_error(file_manager_tag, 
              "Memory Error", 
              "Failed to allocate memory for binary data");
    return ESP_FAIL;
  }
  
  memcpy(request.data, data, data_length);
  request.data_length = data_length;
  request.is_binary   = true; /* This is a binary request */
  
  /* Send the request to the queue */
  if (xQueueSend(s_file_write_queue, &request, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(file_manager_tag, 
              "Queue Error", 
              "Failed to enqueue binary write request: queue full");
    free(request.data);
    return ESP_FAIL;
  }
  
  log_debug(file_manager_tag, 
            "Binary Enqueue Success", 
            "Enqueued binary write request for file: %s (%lu bytes)", 
            file_path, (unsigned long)data_length);
  return ESP_OK;
}

