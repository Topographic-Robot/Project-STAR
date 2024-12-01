/* main/include/managers/file_write_manager.c */

/* TODO: Test this */

#include "file_write_manager.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

/* Constants ******************************************************************/

const char    *file_manager_tag   = "FILE_MANAGER";
const uint32_t max_pending_writes = 10; /* Maximum queued write operations */

/* Typedefs *******************************************************************/

typedef struct {
  char file_path[max_file_path_length];
  char data[max_data_length];
} file_write_request_t;

/* Globals (Static) ***********************************************************/

static QueueHandle_t file_write_queue;

/* Private Functions **********************************************************/

/**
 * @brief Formats the current date and time as a string.
 *
 * This function retrieves the current system time and formats i
 * as a string in the format `YYYY-MM-DD HH:MM:SS`.
 *
 * @param[out] buffer A buffer to store the formatted timestamp.
 * @param[in] buffer_len The length of the buffer.
 */
static void get_timestamp(char *buffer, size_t buffer_len)
{
  time_t    now = time(NULL);
  struct tm timeinfo;

  localtime_r(&now, &timeinfo);
  strftime(buffer, buffer_len, "%Y-%m-%d %H:%M:%S", &timeinfo);
}

/**
 * @brief File writing task to handle queued write requests.
 */
static void file_write_task(void *param)
{
  file_write_request_t request;

  while (1) {
    if (xQueueReceive(file_write_queue, &request, portMAX_DELAY) == pdTRUE) {
      FILE *file = fopen(request.file_path, "a");
      if (file == NULL) {
        ESP_LOGE(file_manager_tag, "Failed to open file: %s", request.file_path);
        continue;
      }

      size_t bytes_written = fwrite(request.data, 1, strlen(request.data), file);
      fclose(file);

      if (bytes_written != strlen(request.data)) {
        ESP_LOGE(file_manager_tag, "Failed to write all data to file: %s", request.file_path);
      } else {
        ESP_LOGI(file_manager_tag, "Data written to file: %s", request.file_path);
      }
    }
  }
}

/* Public Functions ***********************************************************/

esp_err_t file_write_manager_init(void)
{
  file_write_queue = xQueueCreate(max_pending_writes, sizeof(file_write_request_t));
  if (file_write_queue == NULL) {
    ESP_LOGE(file_manager_tag, "Failed to create file write queue");
    return ESP_FAIL;
  }

  xTaskCreate(file_write_task, "file_write_task", 4096, NULL, 5, NULL);
  ESP_LOGI(file_manager_tag, "File write manager initialized");
  return ESP_OK;
}

esp_err_t file_write_enqueue(const char *file_path, const char *data)
{
  if (file_path == NULL || data == NULL) {
    ESP_LOGE(file_manager_tag, "Invalid file path or data");
    return ESP_ERR_INVALID_ARG;
  }

  file_write_request_t request;

  char timestamp[32];
  get_timestamp(timestamp, sizeof(timestamp));
  snprintf(request.file_path, max_file_path_length, "%s", file_path);
  snprintf(request.data, max_data_length, "%s %s\n", timestamp, data);

  if (xQueueSend(file_write_queue, &request, 0) != pdTRUE) {
    ESP_LOGE(file_manager_tag, "File write queue is full");
    return ESP_FAIL;
  }

  ESP_LOGI(file_manager_tag, "Write request queued for file: %s", file_path);
  return ESP_OK;
}

