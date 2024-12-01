/* main/include/tasks/wifi_tasks.c */

#include "wifi_tasks.h"
#include <string.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/* Constants ******************************************************************/

const char   *wifi_tag          = "WiFi";
const uint8_t wifi_max_retry    = 10;
const uint8_t wifi_ssid_max_len = 32;
const uint8_t wifi_pass_max_len = 32;

/* Globals (Static) ***********************************************************/

/* This is a global event handler used for event event management and
 * synchronization between tasks. An event group is a collection of event bits
 * (flags) that tasks can set, clear, or wait on */
static EventGroupHandle_t s_wifi_event_group = NULL;

/* Private (Static) Functions *************************************************/

/**
 * @brief Handles various events related to WiFi and IP connectivity.
 *
 * This function processes two main categories of events:
 *   1. WiFi Events: Manages WiFi station start and disconnection events.
 *      - WIFI_EVENT_STA_START: Initiates connection to the Access Point (AP).
 *      - WIFI_EVENT_STA_DISCONNECTED: Attempts to reconnect to the AP on
 *        disconnection, with a maximum retry limit.
 *   2. IP Events: Manages the event when the station receives an IP address.
 *      - IP_EVENT_STA_GOT_IP: Logs the obtained IP address and sets a bit
 *        indicating successful connection.
 *
 * @param arg A pointer to the argument provided during event registration.
 * @param event_base The base identifier for the event (e.g., WIFI_EVENT or
 *                   IP_EVENT).
 * @param event_id The specific event identifier (e.g., WIFI_EVENT_STA_START).
 * @param event_data A pointer to the event data specific to the event type.
 */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
  static uint8_t s_retry_num = 0;

  /* NOTE:
   * esp_event_base_t is a `const char *` and WIFI_EVENT is a
   * `const char *`, but in C you cannot compare two `const char *` with ==
   * However, to save memory esp has made it that the event_base will be set to
   * the address of WIFI_EVENT when the event is a WiFi event. This is why we
   * can compare with == and we don't need to use strcmp/strncmp.
   *
   * The same is for the other events such as IP events and this is also how
   * event_id's are programmed */

  /* Check for WiFi events */
  if (event_base == WIFI_EVENT) {
    /* Check if the station is started */
    if (event_id == WIFI_EVENT_STA_START) {
      esp_wifi_connect(); /* We can now connect to the network */
      ESP_LOGI(wifi_tag, "Try to connect to the AP");
    }
    /* Check if we have been disconnected */
    if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
      ESP_LOGI(wifi_tag, "Connect to the AP failed");
      /* Try and reconnect */
      if (s_retry_num < wifi_max_retry) {
        esp_wifi_connect();
        s_retry_num++;
        ESP_LOGI(wifi_tag, "Retry to connect to the AP");
      } else {
        ESP_LOGI(wifi_tag, "Failed to connect to the AP after maximum retries");
        /* We can set the bit to express that it failed */
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
      }
    }
  }

  /* Check for IP events */
  if (event_base == IP_EVENT) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(wifi_tag, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    /* An IP event was recieved so the wifi is connected, set the bit */
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

/* Public Functions ***********************************************************/

esp_err_t wifi_init_sta(void)
{
  ESP_LOGI(wifi_tag, "Starting WiFi initialization in station mode.");

  /* Create a new RTOS event group for WiFi events */
  s_wifi_event_group = xEventGroupCreate();
  if (!s_wifi_event_group) {
    ESP_LOGE(wifi_tag, "Failed to create event group. Insufficient FreeRTOS heap.");
    return ESP_ERR_NO_MEM;
  }

  /* Initialize the underlying TCP/IP stack */
  ESP_LOGI(wifi_tag, "Initializing TCP/IP stack.");
  esp_err_t ret = esp_netif_init();
  if (ret != ESP_OK) {
    ESP_LOGE(wifi_tag, "Failed to initialize TCP/IP stack: %d", ret);
    return ret;
  }

  /* Initialize and create a default event loop */
  ESP_LOGI(wifi_tag, "Creating default event loop.");
  ret = esp_event_loop_create_default();
  if (ret != ESP_OK) {
    ESP_LOGE(wifi_tag, "Failed to create event loop: %d", ret);
    return ret;
  }

  /* Create the default WiFi station (STA) interface */
  ESP_LOGI(wifi_tag, "Creating default WiFi station interface.");
  esp_netif_create_default_wifi_sta();

  /* Initialize the WiFi configuration structure with default values */
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  /* Initialize the WiFi driver with the configuration */
  ESP_LOGI(wifi_tag, "Initializing WiFi driver.");
  ret = esp_wifi_init(&cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(wifi_tag, "WiFi driver initialization failed: %d", ret);
    return ret;
  }

  /* Register event handlers for WiFi and IP events */
  ESP_LOGI(wifi_tag, "Registering event handlers for WiFi and IP events.");
  esp_event_handler_instance_t instance_any_id, instance_got_ip;
  ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                            &event_handler, NULL, &instance_any_id);
  if (ret != ESP_OK) {
    ESP_LOGE(wifi_tag, "Failed to register WiFi event handler: %d", ret);
    return ret;
  }

  ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                            &event_handler, NULL, &instance_got_ip);
  if (ret != ESP_OK) {
    ESP_LOGE(wifi_tag, "Failed to register IP event handler: %d", ret);
    return ret;
  }

  /* Configure the WiFi with the SSID and password */
  ESP_LOGI(wifi_tag, "Configuring WiFi with SSID and password.");
  wifi_config_t wifi_config = {};

  size_t ssid_len = strlen((const char *)wifi_ssid);
  if (ssid_len >= wifi_ssid_max_len) {
    ESP_LOGE(wifi_tag, "SSID length exceeds %d bytes", wifi_ssid_max_len - 1);
  } else {
    memcpy(wifi_config.sta.ssid, wifi_ssid, ssid_len);
    wifi_config.sta.ssid[ssid_len] = '\0';
    ESP_LOGI(wifi_tag, "SSID set successfully.");
  }

  size_t pass_len = strlen((const char *)wifi_pass);
  if (pass_len >= wifi_pass_max_len) {
    ESP_LOGE(wifi_tag, "Password length exceeds %d bytes", wifi_pass_max_len - 1);
  } else {
    memcpy(wifi_config.sta.password, wifi_pass, pass_len);
    wifi_config.sta.password[pass_len] = '\0';
    ESP_LOGI(wifi_tag, "Password set successfully.");
  }

  /* Set WiFi to station mode */
  ESP_LOGI(wifi_tag, "Setting WiFi mode to station.");
  ret = esp_wifi_set_mode(WIFI_MODE_STA);
  if (ret != ESP_OK) {
    ESP_LOGE(wifi_tag, "Failed to set WiFi mode: %d", ret);
    return ret;
  }

  /* Apply the configuration */
  ESP_LOGI(wifi_tag, "Applying WiFi configuration.");
  ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  if (ret != ESP_OK) {
    ESP_LOGE(wifi_tag, "Failed to set WiFi configuration: %d", ret);
    return ret;
  }

  /* Start the WiFi driver */
  ESP_LOGI(wifi_tag, "Starting WiFi driver.");
  ret = esp_wifi_start();
  if (ret != ESP_OK) {
    ESP_LOGE(wifi_tag, "Failed to start WiFi driver: %d", ret);
    return ret;
  }

  ESP_LOGI(wifi_tag, "WiFi initialization completed. Waiting for connection.");

  /* Wait for connection or failure */
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  /* Check the connection status */
  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(wifi_tag, "Successfully connected to AP SSID:%s", wifi_ssid);
    return ESP_OK;
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGW(wifi_tag, "Failed to connect to AP SSID:%s, password:%s", wifi_ssid, wifi_pass);
    return ESP_FAIL;
  } else {
    ESP_LOGE(wifi_tag, "Unexpected event occurred while connecting.");
    return ESP_FAIL;
  }
}

