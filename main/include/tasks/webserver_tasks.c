/* main/include/tasks/webserver_tasks.c */

#include "webserver_tasks.h"
#include "webserver_info.h"
#include "system_tasks.h"
#include "esp_http_client.h"
#include "log_handler.h"

/* Public Functions ***********************************************************/

esp_err_t send_sensor_data_to_webserver(const char* const json_string)
{
  if (json_string == NULL) {
    log_error(system_tag, "Data Error", "JSON payload is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if the network is ready */
  if (wifi_check_connection() != ESP_OK) {
    log_error(system_tag, 
              "Network Error", 
              "Network connection unavailable for data transmission");
    return ESP_FAIL;
  }

  log_info(system_tag, "HTTP Start", "Preparing HTTP client for data transmission");

  esp_http_client_config_t config = {
    .url    = WEBSERVER_URL,
    .method = HTTP_METHOD_POST,
  };

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == NULL) {
    log_error(system_tag, "Client Error", "Failed to initialize HTTP client");
    return ESP_FAIL;
  }

  if (esp_http_client_set_header(client, "Content-Type", "application/json") != ESP_OK) {
    log_error(system_tag, "Header Error", "Failed to set Content-Type header");
    esp_http_client_cleanup(client);
    return ESP_FAIL;
  }

  if (esp_http_client_set_post_field(client, json_string, strlen(json_string)) != ESP_OK) {
    log_error(system_tag, "Post Error", "Failed to set POST data");
    esp_http_client_cleanup(client);
    return ESP_FAIL;
  }

  log_info(system_tag, 
           "Send Start", 
           "Transmitting data to server at %s", 
           WEBSERVER_URL);
  esp_err_t err = esp_http_client_perform(client);
  if (err == ESP_OK) {
    int status_code = esp_http_client_get_status_code(client);
    log_info(system_tag, 
             "Send Success", 
             "Data transmitted successfully, server response code: %u", 
             status_code);
  } else {
    log_error(system_tag, 
              "Send Error", 
              "HTTP transmission failed: %s", 
              esp_err_to_name(err));
  }

  esp_http_client_cleanup(client);
  return err;
}

