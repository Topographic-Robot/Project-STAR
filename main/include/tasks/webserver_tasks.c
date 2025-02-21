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
    ESP_LOGE(system_tag, "- Data transmission failed - JSON payload is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if the network is ready */
  if (wifi_check_connection() != ESP_OK) {
    ESP_LOGE(system_tag, "- Data transmission failed - network connection unavailable");
    return ESP_FAIL;
  }

  ESP_LOGI(system_tag, "- Starting data transmission - preparing HTTP client");

  esp_http_client_config_t config = {
    .url    = webserver_url,
    .method = HTTP_METHOD_POST,
  };

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == NULL) {
    ESP_LOGE(system_tag, "- Data transmission failed - HTTP client initialization error");
    return ESP_FAIL;
  }

  if (esp_http_client_set_header(client, "Content-Type", "application/json") != ESP_OK) {
    ESP_LOGE(system_tag, "- Data transmission failed - could not set Content-Type header");
    esp_http_client_cleanup(client);
    return ESP_FAIL;
  }

  if (esp_http_client_set_post_field(client, json_string, strlen(json_string)) != ESP_OK) {
    ESP_LOGE(system_tag, "- Data transmission failed - could not set POST data");
    esp_http_client_cleanup(client);
    return ESP_FAIL;
  }

  ESP_LOGI(system_tag, "- Transmitting data - sending to server at %s", webserver_url);
  esp_err_t err = esp_http_client_perform(client);
  if (err == ESP_OK) {
    int status_code = esp_http_client_get_status_code(client);
    ESP_LOGI(system_tag, "- Data transmission successful - server responded with code %u", status_code);
  } else {
    ESP_LOGE(system_tag, "- Data transmission failed - HTTP error: %s", esp_err_to_name(err));
  }

  esp_http_client_cleanup(client);
  return err;
}

