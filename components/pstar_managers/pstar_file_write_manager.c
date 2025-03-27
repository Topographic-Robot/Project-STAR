/* components/pstar_managers/file_write_manager.c */

#include "pstar_file_write_manager.h"
#include "pstar_log_handler.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h> /* For mkdir, stat */
#include <sys/types.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h" /* For semaphore */
#include <stdatomic.h> /* For atomic bool */
#if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
#include "pstar_sd_card_hal.h"
#include "pstar_time_manager.h" /* Include for timestamping */
/* Fix: Include path validation function from SD HAL if needed (better: use HAL function) */
/* Assuming pstar_sd_card_hal.h provides necessary path validation helpers if used directly */
/* Need a way to validate paths within the manager itself if SD HAL isn't directly used */
#endif
#include <ctype.h> /* For isalnum */

/* Constants ******************************************************************/

/* Fix: Use TAG consistently */
static const char* TAG = "File Manager"; /* Renamed FILE_MANAGER_TAG */

/* Default configuration values (using Kconfig) */
static const file_writer_config_t s_default_config = {
  .priority    = CONFIG_PSTAR_KCONFIG_FILE_MANAGER_TASK_PRIORITY,
  .stack_depth = CONFIG_PSTAR_KCONFIG_FILE_MANAGER_TASK_STACK_SIZE,
  /* .enabled field removed from config struct */
};

/* Private Function Prototypes ************************************************/

/* Removed priv_create_directories, assuming SD HAL handles it or we use its helpers */
static bool priv_validate_relative_path(const char* path); /* Simple relative path check */
static esp_err_t priv_create_directories_recursive(const char* path); /* Recursive directory creation */
static esp_err_t priv_write_to_file(file_write_manager_t* manager,
                                    const char* const     file_path_relative, /* Path relative to mount point */
                                    const char* const     data);
static esp_err_t priv_write_binary_to_file(file_write_manager_t* manager,
                                           const char* const     file_path_relative, /* Path relative to mount point */
                                           const void* const     data,
                                           uint32_t              data_length);
static void priv_file_write_task(void* param);
static void priv_free_request_data(file_write_request_t* request); /* Helper to free data */

/* Public Functions ***********************************************************/

esp_err_t file_write_manager_init(file_write_manager_t*       manager,
#if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
                                  sd_card_hal_t*              sd_card,
#else
                                  void*                       sd_card_placeholder,
#endif
                                  const file_writer_config_t* config)
{
  /* Validate arguments */
  if (manager == NULL) {
    /* Use log_error now - assumes minimal logger is initialized before this. */
    log_error(TAG, "Init Error", "Invalid arguments: manager is NULL");
    return ESP_ERR_INVALID_ARG;
  }
#if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  if (sd_card == NULL) {
      /* Use log_error now */
      log_error(TAG, "Init Error", "Invalid arguments: sd_card is NULL");
      return ESP_ERR_INVALID_ARG;
  }
#else
  /* If SD is disabled, this manager doesn't do anything useful */
  log_warn(TAG, "Init Info", "SD Card support is disabled, file write manager has no storage backend.");
  /* Proceed with minimal init, enqueue will fail later */
#endif

  /* Fix: Use atomic load */
  if (atomic_load(&manager->initialized)) {
    log_warn(TAG, "Init Skip", "File write manager already initialized"); /* Use log_warn now logger should be min init */
    return ESP_OK;
  }

  log_info(TAG, "Init Start", "Initializing file write manager");

  /* Initialize the manager structure */
  memset(manager, 0, sizeof(file_write_manager_t));
#if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  manager->sd_card = sd_card;
#endif
  atomic_init(&manager->initialized, false); /* Initialize atomic flag */
  atomic_init(&manager->cleanup_requested, false); /* Initialize atomic flag */

  /* Apply configuration (use defaults if config is NULL) */
  if (config != NULL) {
    /* Validate config values? */
    if (config->stack_depth < 2048) { /* Use log_warn */
        log_warn(TAG, "Config Warning", "Stack depth %lu too small, using default %u",
                 config->stack_depth, s_default_config.stack_depth);
        manager->config.stack_depth = s_default_config.stack_depth;
    } else {
        manager->config.stack_depth = config->stack_depth;
    }
     manager->config.priority = config->priority;
  } else {
    memcpy(&manager->config, &s_default_config, sizeof(file_writer_config_t));
  }

  /* Create queue for file write requests */
  manager->file_write_queue = xQueueCreate(CONFIG_PSTAR_KCONFIG_FILE_MANAGER_QUEUE_SIZE,
                                           sizeof(file_write_request_t));
  if (manager->file_write_queue == NULL) {
    log_error(TAG, "Queue Error", "Failed to create file write queue"); /* Use log_error */
    return ESP_ERR_NO_MEM; /* Fix: Correct error code */
  }

  /* Fix: Create semaphore for task exit synchronization */
  manager->task_exit_sem = xSemaphoreCreateBinary();
  if (manager->task_exit_sem == NULL) { /* Use log_error */
      log_error(TAG, "Semaphore Error", "Failed to create task exit semaphore");
      vQueueDelete(manager->file_write_queue);
      manager->file_write_queue = NULL;
      return ESP_ERR_NO_MEM;
  }


  /* Create task to process file write requests */
  BaseType_t task_created = xTaskCreate(priv_file_write_task,
                                        "file_write_task",
                                        manager->config.stack_depth,
                                        manager,
                                        manager->config.priority,
                                        &manager->file_write_task);

  if (task_created != pdPASS) {
    log_error(TAG, "Task Error", "Failed to create file write task"); /* Use log_error */
    vQueueDelete(manager->file_write_queue);
    manager->file_write_queue = NULL;
    vSemaphoreDelete(manager->task_exit_sem); /* Fix: Delete semaphore on failure */
    manager->task_exit_sem = NULL;
    return ESP_FAIL;
  }

  atomic_store(&manager->initialized, true); /* Fix: Use atomic store */
  log_info(TAG, "Init Complete", "File write manager initialized successfully");
  return ESP_OK;
}

esp_err_t file_write_enqueue(file_write_manager_t* manager,
                             const char* const     file_path_relative, /* Changed name */
                             const char* const     data)
{
  if (manager == NULL) { /* Fix: Check manager first */
      log_error(TAG, "Enqueue Error", "File write manager pointer is NULL");
      return ESP_ERR_INVALID_ARG;
  }
  /* Fix: Use atomic load and check cleanup flag */
  if (!atomic_load(&manager->initialized) || atomic_load(&manager->cleanup_requested)) {
    log_error(TAG, "Enqueue Error", "File write manager not initialized or cleaning up");
    return ESP_ERR_INVALID_STATE;
  }

  if (!file_path_relative || !data || strlen(file_path_relative) == 0 || strlen(data) == 0) { /* Fix: Check empty strings */
    log_error(TAG, "Enqueue Error", "Invalid arguments: file_path or data is NULL/empty");
    return ESP_ERR_INVALID_ARG;
  }

#if !CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  log_error(TAG, "SD Card Disabled", "Cannot enqueue file write request: SD card support is disabled");
  return ESP_ERR_NOT_SUPPORTED;
#else
  /* Fix: Validate relative path */
  if (!priv_validate_relative_path(file_path_relative)) {
       log_error(TAG, "Enqueue Error", "Invalid relative file path: %s", file_path_relative);
       return ESP_ERR_INVALID_ARG;
   }
   if (strlen(file_path_relative) >= CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH) {
       log_error(TAG, "Enqueue Error", "File path too long: %s", file_path_relative);
       return ESP_ERR_INVALID_ARG;
   }


  /* Create a request */
  file_write_request_t request;
  memset(&request, 0, sizeof(request));
  request.is_terminate_request = false; /* Not a terminate request */

  /* Copy file path to the request */
  strncpy(request.file_path, file_path_relative, CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH - 1);
  request.file_path[CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH - 1] = '\0';

  /* Allocate memory for the text data and copy it */
  size_t data_len_with_null = strlen(data) + 1; /* Include null terminator */
  /* Fix: Check allocation size against max data length? No, CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_DATA_LENGTH is for the content. */
  request.data = malloc(data_len_with_null);
  if (!request.data) { /* Fix: Check malloc result */
    log_error(TAG, "Memory Error", "Failed to allocate memory (%zu bytes) for text data", data_len_with_null);
    return ESP_ERR_NO_MEM; /* Fix: Return correct error */
  }

  memcpy(request.data, data, data_len_with_null);
  request.data_length = data_len_with_null; /* Store length including null */
  request.is_binary   = false; /* This is a text request */

  /* Send the request to the queue */
  if (xQueueSend(manager->file_write_queue, &request, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(TAG, "Queue Error", "Failed to enqueue file write request: queue full");
    priv_free_request_data(&request); /* Fix: Use helper to free data */
    return ESP_FAIL; /* Should be ESP_FAIL or a specific queue full error */
  }

  log_debug(TAG, "Enqueue Success", "Enqueued write request for file: %s", file_path_relative);
  return ESP_OK;
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED */
}

esp_err_t file_write_binary_enqueue(file_write_manager_t* manager,
                                    const char* const     file_path_relative, /* Changed name */
                                    const void* const     data,
                                    uint32_t              data_length)
{
   if (manager == NULL) { /* Fix: Check manager first */
      log_error(TAG, "Binary Enqueue Error", "File write manager pointer is NULL");
      return ESP_ERR_INVALID_ARG;
  }
  /* Fix: Use atomic load and check cleanup flag */
  if (!atomic_load(&manager->initialized) || atomic_load(&manager->cleanup_requested)) {
    log_error(TAG, "Binary Enqueue Error", "File write manager not initialized or cleaning up");
    return ESP_ERR_INVALID_STATE;
  }

  if (!file_path_relative || !data || data_length == 0 || strlen(file_path_relative) == 0) { /* Fix: Check empty path */
    log_error(TAG, "Binary Enqueue Error", "Invalid arguments: file_path/data is NULL, or data_length/path_len is 0");
    return ESP_ERR_INVALID_ARG;
  }

#if !CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  log_error(TAG, "SD Card Disabled", "Cannot enqueue binary write request: SD card support is disabled");
  return ESP_ERR_NOT_SUPPORTED;
#else
  /* Fix: Validate relative path */
  if (!priv_validate_relative_path(file_path_relative)) {
       log_error(TAG, "Binary Enqueue Error", "Invalid relative file path: %s", file_path_relative);
       return ESP_ERR_INVALID_ARG;
  }
   if (strlen(file_path_relative) >= CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH) {
       log_error(TAG, "Binary Enqueue Error", "File path too long: %s", file_path_relative);
       return ESP_ERR_INVALID_ARG;
   }
   /* Fix: Check data_length against a reasonable limit if necessary? */
   /* if (data_length > SOME_MAX_LIMIT) return ESP_ERR_INVALID_SIZE; */


  /* Create a binary request */
  file_write_request_t request;
  memset(&request, 0, sizeof(request));
  request.is_terminate_request = false; /* Not a terminate request */

  /* Copy file path to the request */
  strncpy(request.file_path, file_path_relative, CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH - 1);
  request.file_path[CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH - 1] = '\0';

  /* Allocate memory for the binary data and copy it */
  request.data = malloc(data_length);
  if (!request.data) { /* Fix: Check malloc result */
    log_error(TAG, "Memory Error", "Failed to allocate memory (%lu bytes) for binary data", (unsigned long)data_length);
    return ESP_ERR_NO_MEM; /* Fix: Return correct error */
  }

  memcpy(request.data, data, data_length);
  request.data_length = data_length;
  request.is_binary   = true; /* This is a binary request */

  /* Send the request to the queue */
  if (xQueueSend(manager->file_write_queue, &request, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(TAG, "Queue Error", "Failed to enqueue binary write request: queue full");
    priv_free_request_data(&request); /* Fix: Use helper to free data */
    return ESP_FAIL; /* Should be ESP_FAIL or a specific queue full error */
  }

  log_debug(TAG, "Binary Enqueue Success", "Enqueued binary write request for file: %s (%lu bytes)",
            file_path_relative, (unsigned long)data_length);
  return ESP_OK;
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED */
}

/* Fix: Improved cleanup logic */
esp_err_t file_write_manager_cleanup(file_write_manager_t* manager)
{
  if (manager == NULL) {
    log_error(TAG, "Cleanup Error", "Manager is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  /* Fix: Check initialization state atomically */
  if (!atomic_load(&manager->initialized)) {
    log_warn(TAG, "Cleanup Skip", "File write manager not initialized, nothing to clean up");
    return ESP_OK; /* Not an error if already cleaned up or never init */
  }

  log_info(TAG, "Cleanup Start", "Beginning file write manager cleanup"); /* Use log_info */

  /* Signal that cleanup is requested to prevent new enqueues */
  atomic_store(&manager->cleanup_requested, true);

  /* Create a special termination request to signal the task to exit */
  file_write_request_t termination_request = {0}; /* Initialize all fields to 0/false/NULL */
  termination_request.is_terminate_request = true; /* Set the termination flag */
  /* file_path can remain empty or be set to "TERMINATE" */

  /* Send the termination request to the queue */
  if (manager->file_write_queue != NULL) {
      if (xQueueSend(manager->file_write_queue, &termination_request, pdMS_TO_TICKS(1000)) != pdTRUE) { /* Increased timeout */
          log_error(TAG, "Termination Error", "Failed to enqueue termination request: queue might be full or task unresponsive");
          /* Attempt to continue cleanup, but task might not exit cleanly */
      } else {
          log_info(TAG, "Cleanup", "Termination request sent to file write task."); /* Use log_info */
      }
  }


  /* Wait for the task to signal its exit */
  esp_err_t cleanup_result = ESP_OK;
  if (manager->file_write_task != NULL && manager->task_exit_sem != NULL) {
      log_info(TAG, "Cleanup", "Waiting for file write task to exit...");
      /* Wait for a reasonable time for the task to process remaining queue items and exit */
      if (xSemaphoreTake(manager->task_exit_sem, pdMS_TO_TICKS(5000)) != pdTRUE) { /* Wait up to 5 seconds */
          log_error(TAG, "Cleanup Error", "File write task did not signal exit within timeout. Forcing deletion."); /* Use log_error */
          vTaskDelete(manager->file_write_task); /* Force delete if timeout */
          cleanup_result = ESP_ERR_TIMEOUT; /* Indicate timeout */
      } else {
           log_info(TAG, "Cleanup", "File write task exited cleanly."); /* Use log_info */
       }
      manager->file_write_task = NULL; /* Mark task as gone */
  } else if (manager->file_write_task != NULL) {
       /* If semaphore wasn't created or is NULL, force delete task (less clean) */
       log_warn(TAG, "Cleanup Warning", "Task exit semaphore invalid, forcing task deletion.");
       vTaskDelete(manager->file_write_task);
       manager->file_write_task = NULL;
       if (cleanup_result == ESP_OK) cleanup_result = ESP_FAIL; /* Indicate unclean shutdown */
  }


  /* Delete the queue (process remaining items first - done by task before exit) */
  if (manager->file_write_queue != NULL) {
    /* Items might remain if task was force-deleted or termination send failed */
    UBaseType_t items_remaining = uxQueueMessagesWaiting(manager->file_write_queue);
    if (items_remaining > 0) { /* Use log_warn */
        log_warn(TAG, "Cleanup Warning", "%u items remained in file write queue during cleanup. Data may be lost.", (unsigned int)items_remaining);
        /* Manually drain and free remaining items */
        file_write_request_t req;
        while(xQueueReceive(manager->file_write_queue, &req, 0) == pdTRUE) {
            priv_free_request_data(&req);
        }
    }
    log_info(TAG, "Queue Delete", "Deleting file write queue"); /* Use log_info */
    vQueueDelete(manager->file_write_queue);
    manager->file_write_queue = NULL;
  }

  /* Delete the exit semaphore */
   if (manager->task_exit_sem != NULL) {
      vSemaphoreDelete(manager->task_exit_sem);
      manager->task_exit_sem = NULL;
  }

  atomic_store(&manager->initialized, false); /* Mark as uninitialized */
  /* cleanup_requested flag remains true */

  log_info(TAG, "Cleanup Complete", "File write manager cleanup %s", /* Use log_info */
           (cleanup_result == ESP_OK) ? "completed successfully" : "completed with timeout/errors");

  return cleanup_result;
}

/* Private Functions **********************************************************/

/**
 * @brief Frees the dynamically allocated data within a file write request.
 */
static void priv_free_request_data(file_write_request_t* request) {
    if (request && request->data) {
        free(request->data);
        request->data = NULL;
    }
}

/**
 * @brief Simple validation for relative paths used by the file manager.
 * Disallows '..', starting '/', and potentially unsafe characters.
 */
static bool priv_validate_relative_path(const char* path) {
    if (!path || path[0] == '\0') return false; /* No NULL or empty paths */
    if (path[0] == '/') return false; /* Must be relative */
    if (strstr(path, "..")) return false; /* No directory traversal up */
    if (strstr(path, "//")) return false; /* No double slashes */
    if (strchr(path, '\\')) return false; /* No backslashes */

    /* Check for allowed characters */
    for (const char* p = path; *p; ++p) {
         if (!(isalnum((unsigned char)*p) || *p == '_' || *p == '-' || *p == '.' || *p == '/' || *p == ' ')) {
             /* Add more allowed characters if needed */
             log_error(TAG, "Path Validation", "Invalid character '%c' in relative path '%s'", *p, path);
             return false;
         }
    }
    return true;
}

/**
 * @brief Creates directories recursively. Assumes base path (mount point) exists.
 *
 * @param path Full path to create directories for.
 * @return ESP_OK or error code.
 */
static esp_err_t priv_create_directories_recursive(const char* path) {
    char tmp[CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH * 2]; /* Use larger buffer */
    char *p = NULL;
    struct stat st;
    size_t len;

    snprintf(tmp, sizeof(tmp), "%s", path);
    len = strlen(tmp);
    if (len == 0) return ESP_ERR_INVALID_ARG;

    /* Remove trailing slash if exists */
    if (tmp[len - 1] == '/') {
        tmp[len - 1] = 0;
    }

    /* Check if path already exists */
    if (stat(tmp, &st) == 0) {
        if (S_ISDIR(st.st_mode)) {
            return ESP_OK; /* Already exists as directory */
        } else {
            log_error(TAG, "Dir Create", "'%s' exists but is not a directory", tmp);
            return ESP_ERR_INVALID_STATE;
        }
    }

    /* Find the last slash to get the parent directory */
    p = strrchr(tmp, '/');
    if (p) {
        *p = 0; /* Null-terminate to get parent path */
        /* Recursively create parent directory */
        esp_err_t res = priv_create_directories_recursive(tmp);
        *p = '/'; /* Restore slash */
        if (res != ESP_OK) {
            return res; /* Propagate error */
        }
    }

    /* Create current directory */
    if (mkdir(path, 0755) != 0 && errno != EEXIST) {
        log_error(TAG, "Dir Create", "Failed to create directory '%s': %s (errno %d)", path, strerror(errno), errno);
        return ESP_FAIL;
    }

    log_debug(TAG, "Dir Create", "Ensured directory exists: %s", path);
    return ESP_OK;
}


/* Fix: Removed fsync, added path validation, uses relative path */
static esp_err_t priv_write_to_file(file_write_manager_t* manager,
                                    const char* const     file_path_relative, /* Path relative to mount point */
                                    const char* const     data)
{
  if (manager == NULL || file_path_relative == NULL || data == NULL) {
    log_error(TAG, "Write Error", "Invalid arguments: manager, file_path or data is NULL");
    return ESP_ERR_INVALID_ARG;
  }

#if !CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  log_error(TAG, "SD Card Disabled", "Cannot write to file: SD card support is disabled");
  return ESP_ERR_NOT_SUPPORTED;
#else
  /* Check if SD card is available via HAL function */
  if (!sd_card_is_available(manager->sd_card)) {
    log_error(TAG, "SD Card Error", "SD card not available, cannot write to file: %s", file_path_relative);
    return ESP_FAIL; /* Or a more specific error like ESP_ERR_NOT_FOUND */
  }

  /* Prepare the full path */
  char full_path[CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH + CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH]; /* Combine lengths for safety */
  int path_len = snprintf(full_path, sizeof(full_path), "%s/%s",
                          manager->sd_card->mount_path, file_path_relative);

  if (path_len < 0 || (size_t)path_len >= sizeof(full_path)) {
       log_error(TAG, "File Path Error", "Failed to construct full path or path too long for: %s", file_path_relative);
       return ESP_ERR_INVALID_ARG;
  }

  /* Re-validate the combined full path (primarily for length and within mount point) */
  /* Assuming sd_card->mount_path itself is validated on init */
   size_t mount_path_len = strlen(manager->sd_card->mount_path);
   if (strncmp(full_path, manager->sd_card->mount_path, mount_path_len) != 0 ||
       (strlen(full_path) > mount_path_len && full_path[mount_path_len] != '/'))
   {
        log_error(TAG, "File Path Error", "Constructed full path '%s' is outside mount point '%s'", full_path, manager->sd_card->mount_path);
        return ESP_ERR_INVALID_ARG;
   }
   /* Basic check for ".." again, although relative path check should have caught it */
    if (strstr(full_path + mount_path_len, "..") != NULL) {
         log_error(TAG, "File Path Error", "Constructed full path '%s' contains '..'", full_path);
         return ESP_ERR_INVALID_ARG;
    }


  /* Create directories if they don't exist */
  char* last_slash = strrchr(full_path, '/');
  if (last_slash && last_slash != full_path) { /* Ensure it's not the root '/' */
      *last_slash = '\0'; /* Temporarily terminate to get dir path */
      /* Use recursive directory creation */
      esp_err_t dir_ret = priv_create_directories_recursive(full_path);
      *last_slash = '/'; /* Restore full path */

      if (dir_ret != ESP_OK) {
        log_error(TAG, "Dir Create Error", "Failed to create directories for file: %s", full_path);
        return dir_ret;
      }
  }


  /* Open the file for writing (append mode) */
  FILE* file = fopen(full_path, "a"); /* Use full_path now */
  if (file == NULL) {
    log_error(TAG, "File Open Error", "Failed to open file '%s': %s (errno: %d)",
              full_path, strerror(errno), errno);
    return ESP_FAIL;
  }

  /* Get current timestamp */
  char      timestamp[CONFIG_PSTAR_KCONFIG_FILE_MANAGER_TIMESTAMP_BUFFER_SIZE];
  esp_err_t time_err = time_manager_get_timestamp(timestamp, sizeof(timestamp)); /* Use manager function */
  if (time_err != ESP_OK) {
       log_warn(TAG, "Timestamp Error", "Failed to get timestamp: %s. Writing log without timestamp.", esp_err_to_name(time_err));
       /* Write data without timestamp */
       fprintf(file, "%s\n", data);
  } else {
       /* Write timestamp and data to file */
       fprintf(file, "%s: %s\n", timestamp, data);
  }

  /* Ensure data is written to OS buffer */
  fflush(file);
  /* Fix: Removed fsync for performance */

  /* Close file */
  fclose(file);

  log_debug(TAG, "Write Success", "Data written to file: %s", file_path_relative);
  return ESP_OK;
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED */
}

/* Fix: Removed fsync, added path validation, uses relative path */
static esp_err_t priv_write_binary_to_file(file_write_manager_t* manager,
                                           const char* const     file_path_relative, /* Path relative to mount point */
                                           const void* const     data,
                                           uint32_t              data_length)
{
  if (manager == NULL || file_path_relative == NULL || data == NULL || data_length == 0) {
    log_error(TAG, "Binary Write Error", "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

#if !CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  log_error(TAG, "SD Card Disabled", "Cannot write binary data to file: SD card support is disabled");
  return ESP_ERR_NOT_SUPPORTED;
#else
  /* Check if SD card is available */
  if (!sd_card_is_available(manager->sd_card)) {
    log_error(TAG, "SD Card Error", "SD card not available, cannot write binary data to file: %s", file_path_relative);
    return ESP_FAIL; /* Or a more specific error */
  }

  /* Prepare the full path */
  char full_path[CONFIG_PSTAR_KCONFIG_FILE_MANAGER_MAX_PATH_LENGTH + CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH];
   int path_len = snprintf(full_path, sizeof(full_path), "%s/%s",
                          manager->sd_card->mount_path, file_path_relative);

  if (path_len < 0 || (size_t)path_len >= sizeof(full_path)) {
       log_error(TAG, "File Path Error", "Failed to construct full path or path too long for: %s", file_path_relative);
       return ESP_ERR_INVALID_ARG;
  }

   /* Re-validate the combined full path */
   size_t mount_path_len = strlen(manager->sd_card->mount_path);
   if (strncmp(full_path, manager->sd_card->mount_path, mount_path_len) != 0 ||
       (strlen(full_path) > mount_path_len && full_path[mount_path_len] != '/'))
   {
        log_error(TAG, "File Path Error", "Constructed full path '%s' is outside mount point '%s'", full_path, manager->sd_card->mount_path);
        return ESP_ERR_INVALID_ARG;
   }
    if (strstr(full_path + mount_path_len, "..") != NULL) {
         log_error(TAG, "File Path Error", "Constructed full path '%s' contains '..'", full_path);
         return ESP_ERR_INVALID_ARG;
    }


  /* Create directories if they don't exist */
   char* last_slash = strrchr(full_path, '/');
   if (last_slash && last_slash != full_path) {
        *last_slash = '\0';
        esp_err_t dir_ret = priv_create_directories_recursive(full_path); /* Use recursive create */
        *last_slash = '/';
        if (dir_ret != ESP_OK) {
            log_error(TAG, "Dir Create Error", "Failed to create directories for binary file: %s", full_path);
            return dir_ret;
        }
   }


  /* Open the file for writing (append binary mode) */
  FILE* file = fopen(full_path, "ab"); /* Use "ab" for append binary */
  if (file == NULL) {
    log_error(TAG, "Binary File Open Error", "Failed to open file '%s': %s (errno: %d)",
              full_path, strerror(errno), errno);
    return ESP_FAIL;
  }

  /* Write the binary data to the file */
  size_t bytes_written = fwrite(data, 1, data_length, file);

  /* Ensure data is written to OS buffer */
  fflush(file);
  /* Fix: Removed fsync for performance */

  /* Close the file */
  fclose(file);

  /* Check if write was successful */
  if (bytes_written != data_length) {
    log_error(TAG, "Binary Write Error", "Failed to write all data to '%s': %zu of %lu bytes written",
              file_path_relative, bytes_written, (unsigned long)data_length);
    return ESP_FAIL; /* Indicate partial write failure */
  }


  log_debug(TAG, "Binary Write Success", "Binary data written to file: %s (%lu bytes)",
            file_path_relative, (unsigned long)data_length);
  return ESP_OK;
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED */
}

/* Fix: Added handling for termination request and semaphore signaling */
static void priv_file_write_task(void* param)
{
  file_write_manager_t* manager = (file_write_manager_t*)param;
  /* Keep ESP_LOGE here as logger might not be available if manager is NULL */
  if (manager == NULL) {
    ESP_LOGE(TAG, "Task Error - Null manager pointer, exiting task"); /* Use ESP_LOGE */
    /* Cannot signal semaphore if manager is NULL */
    vTaskDelete(NULL);
    return;
  }

  log_info(TAG, "Task Start", "File write task started"); /* Use log_info */

  while (1) {
    /* Wait for a request from the queue */
    file_write_request_t request;

    /* Use portMAX_DELAY to block indefinitely until a message arrives */
    if (xQueueReceive(manager->file_write_queue, &request, portMAX_DELAY) == pdTRUE) {

      /* Check if this is a termination request */
      if (request.is_terminate_request) {
        log_info(TAG, "Task Terminate", "Received termination request, cleaning up remaining queue items and exiting"); /* Use log_info */
        /* Process any remaining items already fetched or in the queue quickly */
        while(xQueueReceive(manager->file_write_queue, &request, 0) == pdTRUE) {
             if (!request.is_terminate_request) { /* Don't process other terminate requests */
                 /* Attempt to process the request quickly, log errors */
                 esp_err_t write_err = ESP_FAIL;
                 #if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
                 /* Recheck init and SD status (atomic checks are quick) */
                 if (atomic_load(&manager->initialized) && sd_card_is_available(manager->sd_card)) {
                     if (request.is_binary) {
                         write_err = priv_write_binary_to_file(manager, request.file_path, request.data, request.data_length);
                     } else {
                         write_err = priv_write_to_file(manager, request.file_path, (const char*)request.data);
                     }
                 } else {
                     write_err = ESP_ERR_INVALID_STATE; /* SD not available or not init */
                 }
                 if (write_err != ESP_OK) {
                      log_error(TAG, "Cleanup Write Error", "Failed to write remaining request for '%s' during cleanup: %s", request.file_path, esp_err_to_name(write_err));
                 }
                 #endif
                 priv_free_request_data(&request); /* Free data regardless of success */
             }
        }
        break; /* Exit the task loop */
      }

#if !CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
      log_error(TAG, "SD Card Disabled", "Skipping file write request: SD card support is disabled");
      priv_free_request_data(&request); /* Free data even if skipped */
      continue;
#endif

      esp_err_t write_result = ESP_FAIL; /* Track write result */
      /* Check if this is a binary request */
      if (request.is_binary) {
        /* Process binary write request */
        write_result = priv_write_binary_to_file(manager, request.file_path, request.data, request.data_length);
      } else {
        /* Process text write request */
        write_result = priv_write_to_file(manager, request.file_path, (const char* const)request.data);
      }

      if(write_result != ESP_OK) {
          /* Log error (specific error logged within write functions) */
          log_error(TAG, "Task Write Error", "Failed write request for '%s'", request.file_path);
          /* Consider error handling - retry? report? */
      }

      /* Free the allocated data buffer */
      priv_free_request_data(&request);

    } /* End if xQueueReceive */
  } /* End while(1) */

  /* Task cleanup before exit */
  log_info(TAG, "Task Exit", "File write task exiting"); /* Use log_info */

  /* Signal that the task has exited */
  if(manager->task_exit_sem != NULL) {
      xSemaphoreGive(manager->task_exit_sem);
  }

  vTaskDelete(NULL); /* Delete self */
}
