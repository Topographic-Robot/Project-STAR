/* main/include/tasks/webserver_tasks.c */

#include "webserver_tasks.h"
#include "webserver_info.h"
#include "system_tasks.h"
#include "esp_http_client.h"
#include "esp_log.h"

/* Public Functions ***********************************************************/

esp_err_t send_sensor_data_to_webserver(const char *json_string) 
{
  if (json_string == NULL) {
    ESP_LOGE(system_tag, "JSON string is NULL.");
    return ESP_ERR_INVALID_ARG;
  }

  esp_http_client_config_t config = {
    .url = webserver_url,
    .method = HTTP_METHOD_POST,
  };
  esp_http_client_handle_t client = esp_http_client_init(&config);

  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_post_field(client, json_string, strlen(json_string));

  esp_err_t err = esp_http_client_perform(client);
  if (err == ESP_OK) {
    ESP_LOGI(system_tag, "Data sent successfully.");
  } else {
    ESP_LOGE(system_tag, "Failed to send data: %s", esp_err_to_name(err));
  }

  esp_http_client_cleanup(client);
  return err;
}

