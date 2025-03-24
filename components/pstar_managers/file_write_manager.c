/* components/pstar_managers/file_write_manager.c */

#include "include/file_write_manager.h"
#include "log_handler.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sd_card_hal.h"

/* Constants ******************************************************************/

#define FILE_MANAGER_TAG ("File Manager")

/* Default configuration values */
static const file_writer_config_t s_default_config = {
  .priority    = 5,
  .stack_depth = 4096,
  .enabled     = true
};

/* Private Function Prototypes ************************************************/

static esp_err_t priv_create_directories(const char* const file_path);
static esp_err_t priv_write_to_file(file_write_manager_t* manager,
                                    const char* const     file_path, 
                                    const char* const     data);
static esp_err_t priv_write_binary_to_file(file_write_manager_t* manager,
                                           const char* const     file_path, 
                                           const void* const     data, 
                                           uint32_t              data_length);
static void priv_file_write_task(void* param);

/* Public Functions ***********************************************************/

esp_err_t file_write_manager_init(file_write_manager_t*       manager,
                                  sd_card_hal_t*              sd_card,
                                  const file_writer_config_t* config)
{
  /* Validate arguments */
  if (manager == NULL || sd_card == NULL) {
    log_error(FILE_MANAGER_TAG, 
              "Init Error", 
              "Invalid arguments: manager or sd_card is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  if (manager->initialized) {
    log_warn(FILE_MANAGER_TAG, 
             "Init Skip", 
             "File write manager already initialized");
    return ESP_OK;
  }
  
  log_info(FILE_MANAGER_TAG, "Init Start", "Initializing file write manager");
  
  /* Initialize the manager structure */
  memset(manager, 0, sizeof(file_write_manager_t));
  manager->sd_card = sd_card;
  
  /* Apply configuration (use defaults if config is NULL) */
  if (config != NULL) {
    memcpy(&manager->config, config, sizeof(file_writer_config_t));
  } else {
    memcpy(&manager->config, &s_default_config, sizeof(file_writer_config_t));
  }
  
  /* Skip initialization if not enabled */
  if (!manager->config.enabled) {
    log_info(FILE_MANAGER_TAG, 
             "Init Skip", 
             "File write manager disabled in configuration");
    return ESP_OK;
  }
  
  /* Create queue for file write requests */
  manager->file_write_queue = xQueueCreate(CONFIG_PSTAR_FILE_MANAGER_MAX_PENDING_WRITES, 
                                           sizeof(file_write_request_t));
  if (manager->file_write_queue == NULL) {
    log_error(FILE_MANAGER_TAG, 
              "Queue Error", 
              "Failed to create file write queue");
    return ESP_FAIL;
  }
  
  /* Create task to process file write requests */
  BaseType_t task_created = xTaskCreate(priv_file_write_task,
                                        "file_write_task",
                                        manager->config.stack_depth,
                                        manager,
                                        manager->config.priority,
                                        &manager->file_write_task);
  
  if (task_created != pdPASS) {
    log_error(FILE_MANAGER_TAG, "Task Error", "Failed to create file write task");
    vQueueDelete(manager->file_write_queue);
    manager->file_write_queue = NULL;
    return ESP_FAIL;
  }
  
  manager->initialized = true;
  log_info(FILE_MANAGER_TAG, 
           "Init Complete", 
           "File write manager initialized successfully");
  return ESP_OK;
}

esp_err_t file_write_enqueue(file_write_manager_t* manager,
                             const char* const file_path, 
                             const char* const data)
{
  if (manager == NULL || !manager->initialized) {
    log_error(FILE_MANAGER_TAG, 
              "Enqueue Error", 
              "File write manager not initialized or NULL");
    return ESP_FAIL;
  }
  
  if (!file_path || !data) {
    log_error(FILE_MANAGER_TAG, 
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
    log_error(FILE_MANAGER_TAG, 
              "Memory Error", 
              "Failed to allocate memory for text data");
    return ESP_FAIL;
  }
  
  memcpy(request.data, data, data_length);
  request.data_length = data_length;
  request.is_binary   = false; /* This is a text request */
  
  /* Send the request to the queue */
  if (xQueueSend(manager->file_write_queue, &request, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(FILE_MANAGER_TAG, 
              "Queue Error", 
              "Failed to enqueue file write request: queue full");
    free(request.data);
    return ESP_FAIL;
  }
  
  log_debug(FILE_MANAGER_TAG, 
            "Enqueue Success", 
            "Enqueued write request for file: %s", 
            file_path);
  return ESP_OK;
}

esp_err_t file_write_binary_enqueue(file_write_manager_t* manager,
                                    const char* const     file_path, 
                                    const void* const     data, 
                                    uint32_t              data_length)
{
  if (manager == NULL || !manager->initialized) {
    log_error(FILE_MANAGER_TAG, 
              "Binary Enqueue Error", 
              "File write manager not initialized or NULL");
    return ESP_FAIL;
  }
  
  if (!file_path || !data || data_length == 0) {
    log_error(FILE_MANAGER_TAG, 
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
    log_error(FILE_MANAGER_TAG, 
              "Memory Error", 
              "Failed to allocate memory for binary data");
    return ESP_FAIL;
  }
  
  memcpy(request.data, data, data_length);
  request.data_length = data_length;
  request.is_binary   = true; /* This is a binary request */
  
  /* Send the request to the queue */
  if (xQueueSend(manager->file_write_queue, &request, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(FILE_MANAGER_TAG, 
              "Queue Error", 
              "Failed to enqueue binary write request: queue full");
    free(request.data);
    return ESP_FAIL;
  }
  
  log_debug(FILE_MANAGER_TAG, 
            "Binary Enqueue Success", 
            "Enqueued binary write request for file: %s (%lu bytes)", 
            file_path, (unsigned long)data_length);
  return ESP_OK;
}

esp_err_t file_write_manager_cleanup(file_write_manager_t* manager)
{
  if (manager == NULL) {
    log_error(FILE_MANAGER_TAG, 
              "Cleanup Error", 
              "Manager is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  if (!manager->initialized) {
    log_warn(FILE_MANAGER_TAG, 
             "Cleanup Skip", 
             "File write manager not initialized, nothing to clean up");
    return ESP_OK;
  }

  log_info(FILE_MANAGER_TAG, 
           "Cleanup Start", 
           "Beginning file write manager cleanup");

  /* Process any remaining items in the queue */
  log_info(FILE_MANAGER_TAG, 
           "Queue Drain", 
           "Processing remaining items in the write queue");

  /* Create a special termination request to signal the task to exit */
  file_write_request_t termination_request;
  memset(&termination_request, 0, sizeof(file_write_request_t));
  strncpy(termination_request.file_path, "TERMINATE", MAX_FILE_PATH_LENGTH - 1);
  termination_request.data        = NULL;
  termination_request.data_length = 0;
  termination_request.is_binary   = false;

  /* Send the termination request to the queue */
  if (xQueueSend(manager->file_write_queue, &termination_request, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(FILE_MANAGER_TAG, 
              "Termination Error", 
              "Failed to enqueue termination request: queue full");
    /* Continue with cleanup anyway */
  }

  /* Wait for the task to process the termination request and exit */
  vTaskDelay(pdMS_TO_TICKS(500)); /* Give the task some time to process the request */

  /* Delete the task */
  if (manager->file_write_task != NULL) {
    log_info(FILE_MANAGER_TAG, 
             "Task Delete", 
             "Deleting file write task");
    vTaskDelete(manager->file_write_task);
    manager->file_write_task = NULL;
  }

  /* Delete the queue */
  if (manager->file_write_queue != NULL) {
    log_info(FILE_MANAGER_TAG, 
             "Queue Delete", 
             "Deleting file write queue");
    vQueueDelete(manager->file_write_queue);
    manager->file_write_queue = NULL;
  }

  manager->initialized = false;
  log_info(FILE_MANAGER_TAG, 
           "Cleanup Complete", 
           "File write manager cleanup completed successfully");
  
  return ESP_OK;
}

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
    log_error(FILE_MANAGER_TAG, 
              "Dir Create Error", 
              "Invalid file path (NULL)");
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
        log_error(FILE_MANAGER_TAG, 
                  "Dir Create Failed", 
                  "Failed to create directory: %s (errno: %d)", 
                  path_copy, 
                  errno);
        return ESP_FAIL;
      }
      log_debug(FILE_MANAGER_TAG, "Dir Created", "Created directory: %s", path_copy);
    } else if (!S_ISDIR(st.st_mode)) {
      /* Path exists but is not a directory */
      log_error(FILE_MANAGER_TAG, 
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
 * @param[in] manager   Pointer to the file write manager
 * @param[in] file_path Path to the file
 * @param[in] data      String to write
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_write_to_file(file_write_manager_t* manager,
                                    const char* const     file_path, 
                                    const char* const     data)
{
  if (manager == NULL || file_path == NULL || data == NULL) {
    log_error(FILE_MANAGER_TAG, 
              "Write Error", 
              "Invalid arguments: manager, file_path or data is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if SD card is available */
  if (!sd_card_is_available(manager->sd_card)) {
    log_error(FILE_MANAGER_TAG, 
              "SD Card Error", 
              "SD card not available, cannot write to file: %s", 
              file_path);
    return ESP_FAIL;
  }
  
  /* Prepend the SD card mount path to the file path */
  char full_path[MAX_FILE_PATH_LENGTH * 2];
  snprintf(full_path, 
           sizeof(full_path), 
           "%s/%s", 
           manager->sd_card->mount_path, 
           file_path);
  
  /* Create directories if they don't exist */
  esp_err_t ret = priv_create_directories(full_path);
  if (ret != ESP_OK) {
    log_error(FILE_MANAGER_TAG, 
              "Dir Create Error", 
              "Failed to create directories for file: %s", 
              full_path);
    return ret;
  }
  
  /* Open the file for writing (append mode) */
  FILE* file = fopen(full_path, "a");
  if (file == NULL) {
    log_error(FILE_MANAGER_TAG, 
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
  
  log_debug(FILE_MANAGER_TAG, 
            "Write Success", 
            "Data written to file: %s", 
            file_path);
  return ESP_OK;
}

/**
 * @brief Writes binary data to a file
 * 
 * @param[in] manager     Pointer to the file write manager
 * @param[in] file_path   Path to the file
 * @param[in] data        Binary data to write
 * @param[in] data_length Length of the binary data
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_write_binary_to_file(file_write_manager_t* manager,
                                           const char* const     file_path, 
                                           const void* const     data, 
                                           uint32_t              data_length)
{
  if (manager == NULL || file_path == NULL || data == NULL || data_length == 0) {
    log_error(FILE_MANAGER_TAG, 
              "Binary Write Error", 
              "Invalid arguments: manager, file_path or data is NULL, or data_length is 0");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if SD card is available */
  if (!sd_card_is_available(manager->sd_card)) {
    log_error(FILE_MANAGER_TAG, 
              "SD Card Error", 
              "SD card not available, cannot write binary data to file: %s", 
              file_path);
    return ESP_FAIL;
  }
  
  /* Prepend the SD card mount path to the file path */
  char full_path[MAX_FILE_PATH_LENGTH * 2];
  snprintf(full_path, 
           sizeof(full_path), 
           "%s/%s", 
           manager->sd_card->mount_path, 
           file_path);
  
  /* Create directories if they don't exist */
  esp_err_t ret = priv_create_directories(full_path);
  if (ret != ESP_OK) {
    log_error(FILE_MANAGER_TAG, 
              "Dir Create Error", 
              "Failed to create directories for binary file: %s", 
              full_path);
    return ret;
  }
  
  /* Open the file for writing (binary mode) */
  FILE* file = fopen(full_path, "wb");
  if (file == NULL) {
    log_error(FILE_MANAGER_TAG, 
              "Binary File Open Error", 
              "Failed to open file: %s (errno: %d)",
              full_path, 
              errno);
    return ESP_FAIL;
  }
  
  /* Write the binary data to the file */
  size_t bytes_written = fwrite(data, 1, data_length, file);
  if (bytes_written != data_length) {
    log_error(FILE_MANAGER_TAG, 
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
  
  log_debug(FILE_MANAGER_TAG, 
            "Binary Write Success", 
            "Binary data written to file: %s (%lu bytes)",
            file_path, 
            data_length);
  return ESP_OK;
}

/**
 * @brief Task that processes file write requests from the queue
 * 
 * @param[in] param Pointer to the file write manager
 */
static void priv_file_write_task(void* param)
{
  file_write_manager_t* manager = (file_write_manager_t*)param;
  
  if (manager == NULL) {
    log_error(FILE_MANAGER_TAG, "Task Error", "Null manager pointer, exiting task");
    vTaskDelete(NULL);
    return;
  }
  
  log_info(FILE_MANAGER_TAG, "Task Start", "File write task started");
  
  while (1) {
    /* Wait for a request from the queue */
    file_write_request_t request;
    
    if (xQueueReceive(manager->file_write_queue, &request, portMAX_DELAY) == pdTRUE) {
      /* Check if this is a termination request */
      if (strcmp(request.file_path, "TERMINATE") == 0) {
        log_info(FILE_MANAGER_TAG, 
                 "Task Terminate", 
                 "Received termination request, exiting task");
        break; /* Exit the task loop */
      }
      
      /* Check if this is a binary request */
      if (request.is_binary) {
        /* Process binary write request */
        priv_write_binary_to_file(manager,
                                  request.file_path,
                                  request.data,
                                  request.data_length);
        
        /* Free the allocated data buffer */
        if (request.data) {
          free(request.data);
          request.data = NULL;
        }
      } else {
        /* Process text write request */
        priv_write_to_file(manager, 
                           request.file_path, 
                           (const char* const)request.data);
        
        /* Free the allocated data buffer */
        if (request.data) {
          free(request.data);
          request.data = NULL;
        }
      }
    }
  }
  
  /* Task cleanup before exit */
  log_info(FILE_MANAGER_TAG, 
           "Task Exit", 
           "File write task exiting");
  
  /* The task will be deleted by the cleanup function */
  vTaskDelete(NULL);
}