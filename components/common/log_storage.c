/* components/common/log_storage.c */

#include "log_storage.h"
#include "log_handler.h"
#include "file_write_manager.h"
#include "esp_system.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <errno.h>

/* Constants ******************************************************************/

const char *log_storage_tag = "LOG_STORAGE";
const char *log_base_dir    = "logs";

/* Globals (Static) ***********************************************************/

static log_entry_t       s_log_buffer[LOG_BUFFER_SIZE]            = {0};   /* Buffer to store logs when SD card is not available */
static uint32_t          s_log_buffer_index                       = 0;     /* Current index in the buffer */
static bool              s_sd_card_available                      = false; /* Flag to track SD card availability */
static bool              s_log_storage_initialized                = false; /* Flag to track initialization status */
static char              s_current_log_file[MAX_FILE_PATH_LENGTH] = {0};   /* Current log file path */
static SemaphoreHandle_t s_log_mutex                              = NULL;  /* Mutex for thread-safe access */

/* Private Functions **********************************************************/

/**
 * @brief Converts a log level to its string representation
 * 
 * @param[in] level The log level
 * @return Const string representing the log level
 */
static const char *priv_log_level_to_string(esp_log_level_t level)
{
  switch (level) {
    case ESP_LOG_ERROR:   return "ERROR";
    case ESP_LOG_WARN:    return "WARN";
    case ESP_LOG_INFO:    return "INFO";
    case ESP_LOG_DEBUG:   return "DEBUG";
    case ESP_LOG_VERBOSE: return "VERBOSE";
    default:              return "UNKNOWN";
  }
}

/**
 * @brief Creates the log directory structure if it doesn't exist
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_create_log_directories(void)
{
  char dir_path[MAX_FILE_PATH_LENGTH];
  
  /* Create base log directory */
  snprintf(dir_path, sizeof(dir_path), "%s", log_base_dir);
  
  struct stat st;
  if (stat(dir_path, &st) != 0) {
    /* Directory doesn't exist, create it */
    if (mkdir(dir_path, 0755) != 0) { /* 0755: rwx for owner, rx for group and others */
      log_error(log_storage_tag, "Dir Create Failed", "Failed to create log directory: %s (errno: %d)", 
                dir_path, errno);
      return ESP_FAIL;
    }
    log_info(log_storage_tag, "Dir Created", "Created log directory: %s", dir_path);
  }
  
  /* Create date-based subdirectory */
  time_t    now = time(NULL);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  
  char date_str[32];
  snprintf(date_str, sizeof(date_str), DATE_FORMAT, FORMAT_DATE_ARGS(&timeinfo));
  
  snprintf(dir_path, sizeof(dir_path), "%s/%s", 
           log_base_dir, 
           date_str);
  
  if (stat(dir_path, &st) != 0) {
    /* Directory doesn't exist, create it */
    if (mkdir(dir_path, 0755) != 0) { /* 0755: rwx for owner, rx for group and others */
      log_error(log_storage_tag, "Dir Create Failed", "Failed to create date directory: %s (errno: %d)", 
                dir_path, errno);
      return ESP_FAIL;
    }
    log_info(log_storage_tag, "Dir Created", "Created date directory: %s", dir_path);
  }
  
  return ESP_OK;
}

/**
 * @brief Generates the current log file path based on date and time
 * 
 * @param[out] file_path Buffer to store the generated file path
 * @param[in] file_path_len Length of the file_path buffer
 */
static void priv_generate_log_file_path(char *file_path, size_t file_path_len)
{
  time_t    now = time(NULL);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  
  char date_str[32];
  snprintf(date_str, sizeof(date_str), DATE_FORMAT, FORMAT_DATE_ARGS(&timeinfo));
  
  snprintf(file_path, file_path_len, "%s/%s/" LOG_FILENAME_FORMAT, 
           log_base_dir,
           date_str,
           FORMAT_DATE_ARGS(&timeinfo),
           FORMAT_TIME_ARGS(&timeinfo));
}

/**
 * @brief Checks if the current log file needs rotation
 * 
 * @return true if rotation is needed, false otherwise
 */
static bool priv_check_log_rotation(void)
{
  if (s_current_log_file[0] == '\0') {
    return true; /* No current log file, need to create one */
  }
  
  struct stat st;
  char        full_path[MAX_FILE_PATH_LENGTH * 2];
  snprintf(full_path, sizeof(full_path), "%s/%s", sd_card_mount_path, s_current_log_file);
  
  if (stat(full_path, &st) != 0) {
    return true; /* File doesn't exist, need to create one */
  }
  
  if (st.st_size >= LOG_MAX_FILE_SIZE) {
    return true; /* File is too large, need rotation */
  }
  
  /* Check if day has changed */
  time_t    now = time(NULL);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  
  char date_str[32]; /* Increased from 20 to ensure enough space */
  snprintf(date_str, sizeof(date_str), DATE_FORMAT, FORMAT_DATE_ARGS(&timeinfo));
  
  if (strstr(s_current_log_file, date_str) == NULL) {
    return true; /* Day has changed, need new log file */
  }
  
  return false;
}

/**
 * @brief Rotates the log file if needed
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_rotate_log_file(void)
{
  if (!priv_check_log_rotation()) {
    return ESP_OK; /* No rotation needed */
  }
  
  /* Create directories if needed */
  if (priv_create_log_directories() != ESP_OK) {
    return ESP_FAIL;
  }
  
  /* Generate new log file path */
  priv_generate_log_file_path(s_current_log_file, sizeof(s_current_log_file));
  log_info(log_storage_tag, "Log Rotation", "Rotating to new log file: %s", s_current_log_file);
  
  return ESP_OK;
}

/**
 * @brief Flushes the buffered logs to the SD card
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_flush_log_buffer(void)
{
  if (s_log_buffer_index == 0) {
    return ESP_OK; /* Nothing to flush */
  }
  
  if (!s_sd_card_available) {
    log_warn(log_storage_tag, "Flush Skip", "SD card not available, keeping %lu logs in buffer", 
             s_log_buffer_index);
    return ESP_FAIL;
  }
  
  /* Ensure we have a valid log file */
  if (priv_rotate_log_file() != ESP_OK) {
    return ESP_FAIL;
  }
  
  /* Write each buffered log entry to the file */
  for (uint32_t i = 0; i < s_log_buffer_index; i++) {
    const char *level_str = priv_log_level_to_string(s_log_buffer[i].level);
    
    /* Format: [TIMESTAMP] [LEVEL] MESSAGE */
    char formatted_log[MAX_DATA_LENGTH * 2]; /* Doubled the size to ensure enough space */
    time_t log_time = s_log_buffer[i].timestamp / 1000000; /* Convert microseconds to seconds */
    struct tm timeinfo;
    localtime_r(&log_time, &timeinfo);
    
    uint64_t milliseconds = (s_log_buffer[i].timestamp % 1000000) / 1000; /* Milliseconds */
    
    /* Format the timestamp and log entry */
    snprintf(formatted_log, sizeof(formatted_log), 
             TIMESTAMP_FORMAT " [%s] %s",
             FORMAT_DATE_ARGS(&timeinfo),
             FORMAT_TIME_ARGS(&timeinfo),
             milliseconds,
             level_str,
             s_log_buffer[i].buffer);
    
    /* Enqueue the log for writing */
    esp_err_t ret = file_write_enqueue(s_current_log_file, formatted_log);
    if (ret != ESP_OK) {
      log_error(log_storage_tag, "Write Failed", "Failed to enqueue log for writing: %s", 
                esp_err_to_name(ret));
      return ESP_FAIL;
    }
  }
  
  /* Reset buffer index */
  s_log_buffer_index = 0;
  log_info(log_storage_tag, "Buffer Flushed", "Successfully flushed log buffer to SD card");
  
  return ESP_OK;
}

/* Public Functions ***********************************************************/

esp_err_t log_storage_init(void)
{
  if (s_log_storage_initialized) {
    log_warn(log_storage_tag, "Init Skip", "Log storage already initialized");
    return ESP_OK;
  }
  
  log_info(log_storage_tag, "Init Start", "Initializing log storage system");
  
  /* Create mutex for thread-safe access */
  s_log_mutex = xSemaphoreCreateMutex();
  if (s_log_mutex == NULL) {
    log_error(log_storage_tag, "Mutex Error", "Failed to create log storage mutex");
    return ESP_FAIL;
  }
  
  /* Initialize buffer */
  s_log_buffer_index = 0;
  memset(s_log_buffer, 0, sizeof(s_log_buffer));
  
  /* Check SD card availability */
  s_sd_card_available = true; /* Assume available initially */
  
  /* Generate initial log file path */
  s_current_log_file[0] = '\0'; /* Will be generated on first write */
  
  s_log_storage_initialized = true;
  log_info(log_storage_tag, "Init Complete", "Log storage system initialized successfully");
  
  return ESP_OK;
}

esp_err_t log_storage_set_sd_available(bool available)
{
  if (!s_log_storage_initialized) {
    return ESP_FAIL;
  }
  
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(log_storage_tag, "Mutex Error", "Failed to acquire mutex for SD card status update");
    return ESP_FAIL;
  }
  
  bool previous_state = s_sd_card_available;
  s_sd_card_available = available;
  
  if (!previous_state && available) {
    /* SD card became available, try to flush buffer */
    log_info(log_storage_tag, "SD Available", "SD card became available, flushing buffered logs");
    priv_flush_log_buffer();
  } else if (previous_state && !available) {
    log_warn(log_storage_tag, "SD Unavailable", "SD card became unavailable, logs will be buffered");
  }
  
  xSemaphoreGive(s_log_mutex);
  return ESP_OK;
}

esp_err_t log_storage_write(esp_log_level_t level, const char *message)
{
  if (!s_log_storage_initialized) {
    return ESP_FAIL;
  }
  
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    /* Can't log this error through normal channels as it might cause recursion */
    ESP_LOGE(log_storage_tag, "Failed to acquire mutex for log write");
    return ESP_FAIL;
  }
  
  /* Store log in buffer */
  if (s_log_buffer_index < LOG_BUFFER_SIZE) {
    s_log_buffer[s_log_buffer_index].level     = level;
    s_log_buffer[s_log_buffer_index].timestamp = esp_timer_get_time();
    strncpy(s_log_buffer[s_log_buffer_index].buffer, message, LOG_MAX_MESSAGE_LENGTH - 1);
    s_log_buffer[s_log_buffer_index].buffer[LOG_MAX_MESSAGE_LENGTH - 1] = '\0';
    s_log_buffer_index++;
  } else {
    /* Buffer is full, need to flush */
    log_warn(log_storage_tag, "Buffer Full", "Log buffer full, forcing flush");
    priv_flush_log_buffer();
    
    /* Store the current log */
    s_log_buffer[0].level     = level;
    s_log_buffer[0].timestamp = esp_timer_get_time();
    strncpy(s_log_buffer[0].buffer, message, LOG_MAX_MESSAGE_LENGTH - 1);
    s_log_buffer[0].buffer[LOG_MAX_MESSAGE_LENGTH - 1] = '\0';
    s_log_buffer_index                                 = 1;
  }
  
  /* If buffer is full or SD card is available, try to flush */
  if (s_log_buffer_index >= LOG_BUFFER_SIZE && s_sd_card_available) {
    priv_flush_log_buffer();
  }
  
  xSemaphoreGive(s_log_mutex);
  return ESP_OK;
}

esp_err_t log_storage_flush(void)
{
  if (!s_log_storage_initialized) {
    log_error(log_storage_tag, "Flush Error", "Log storage not initialized");
    return ESP_FAIL;
  }
  
  if (xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    log_error(log_storage_tag, "Mutex Error", "Failed to acquire mutex for log flush");
    return ESP_FAIL;
  }
  
  esp_err_t ret = priv_flush_log_buffer();
  
  xSemaphoreGive(s_log_mutex);
  log_info(log_storage_tag, "Flush Complete", "Log flush completed successfully");
  return ret;
} 