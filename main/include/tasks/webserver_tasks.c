/* main/include/tasks/webserver_tasks.c */

#include "webserver_tasks.h"
#include "webserver_info.h"
#include "system_tasks.h"
#include "esp_http_client.h"
#include "esp_log.h"

/* Public Functions ***********************************************************/

#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_log.h"

esp_err_t send_sensor_data_to_webserver(const char *json_string)
{
  if (json_string == NULL) {
    ESP_LOGE(system_tag, "JSON string is NULL.");
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if the network is ready */
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (netif == NULL) {
    ESP_LOGE(system_tag, "Network interface not found.");
    return ESP_FAIL;
  }

  esp_netif_ip_info_t ip_info;
  if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK || ip_info.ip.addr == 0) {
    ESP_LOGW(system_tag, "No IP address. Wi-Fi might be disconnected.");
    return ESP_FAIL;
  }

  esp_http_client_config_t config = {
    .url    = webserver_url,
    .method = HTTP_METHOD_POST,
  };

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == NULL) {
    ESP_LOGE(system_tag, "Failed to initialize HTTP client.");
    return ESP_FAIL;
  }

  if (esp_http_client_set_header(client, "Content-Type", "application/json") != ESP_OK) {
    ESP_LOGE(system_tag, "Failed to set HTTP header.");
    esp_http_client_cleanup(client);
    return ESP_FAIL;
  }

  if (esp_http_client_set_post_field(client, json_string, strlen(json_string)) != ESP_OK) {
    ESP_LOGE(system_tag, "Failed to set HTTP POST field.");
    esp_http_client_cleanup(client);
    return ESP_FAIL;
  }

  esp_err_t err = esp_http_client_perform(client);
  if (err == ESP_OK) {
    ESP_LOGI(system_tag, "Data sent successfully.");
  } else {
    ESP_LOGE(system_tag, "Failed to send data: %s", esp_err_to_name(err));
  }

  esp_http_client_cleanup(client);
  return err;
}

