/* main/include/tasks/wifi_tasks.c */

#include "wifi_tasks.h"
#include <string.h>
#include <stdbool.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "error_handler.h"

/* Constants ******************************************************************/

const char    *wifi_tag                    = "WiFi";
const uint8_t  wifi_max_retry              = 10;
const uint8_t  wifi_ssid_max_len           = 32;
const uint8_t  wifi_pass_max_len           = 32;
const uint32_t wifi_connect_timeout_ms     = 30000;
const uint32_t wifi_initial_retry_interval = pdMS_TO_TICKS(1000);   /* 1 second */
const uint32_t wifi_max_retry_interval     = pdMS_TO_TICKS(30000);  /* 30 seconds */
const uint32_t wifi_backoff_interval       = pdMS_TO_TICKS(60000);  /* 1 minute */
const uint32_t wifi_max_backoff_interval   = pdMS_TO_TICKS(300000); /* 5 minutes */

/* Globals (Static) ***********************************************************/

/* This is a global event handler used for event event management and
 * synchronization between tasks. An event group is a collection of event bits
 * (flags) that tasks can set, clear, or wait on */
static EventGroupHandle_t s_wifi_event_group   = NULL;
static TimerHandle_t      s_wifi_connect_timer = NULL;
static error_handler_t    s_wifi_error_handler = {};
static TaskHandle_t       s_wifi_task_handle   = NULL;

/* Private (Static) Functions *************************************************/

/**
 * @brief Timer callback for handling WiFi connection timeout.
 *
 * Stops the WiFi connection process if the timer expires.
 *
 * @param[in] xTimer Timer handle (unused in this implementation).
 */
static void priv_wifi_connect_timeout_cb(TimerHandle_t xTimer)
{
  ESP_LOGW(wifi_tag, "WiFi connection timeout reached. Stopping connection attempts.");
  esp_wifi_stop();
  xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); /* Signal connection failure */
}

/**
 * @brief Handles Wi-Fi and IP events for connection management.
 *
 * Responds to Wi-Fi and IP-related events, managing actions such as connecting 
 * to an access point, retrying on disconnection, and handling IP acquisition. 
 * Ensures connection retries are limited and logs status updates.
 *
 * @param[in] arg         Pointer to user-defined data passed during event registration.
 * @param[in] event_base  Event base (e.g., `WIFI_EVENT`, `IP_EVENT`) that triggered the handler.
 * @param[in] event_id    Specific event ID within the event base.
 * @param[in] event_data  Pointer to event-specific data (type varies by event).
 *
 * @note 
 * - Intended for internal use as part of the ESP-IDF event handling system.
 * - Registered as an event handler during Wi-Fi initialization.
 */
static void priv_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT) {
    if (event_id == WIFI_EVENT_STA_START) {
      esp_wifi_connect();
      ESP_LOGI(wifi_tag, "Trying to connect to the AP");
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
      wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
      
      /* Record the error and attempt recovery */
      esp_err_t error = error_handler_record_error(&s_wifi_error_handler, ESP_ERR_WIFI_NOT_CONNECT);
      if (error == ESP_OK) {
        /* Error handler says we can retry */
        esp_wifi_connect();
        ESP_LOGI(wifi_tag, "Retry to connect to the AP");
      } else {
        ESP_LOGI(wifi_tag, "Maximum retries reached or in backoff period");
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
      }
    }
  }

  if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(wifi_tag, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    
    /* Reset error handler state on successful connection */
    error_handler_reset(&s_wifi_error_handler);
    
    xTimerStop(s_wifi_connect_timer, 0);
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

static esp_err_t priv_wifi_reset(void *context) 
{
  ESP_LOGI(wifi_tag, "Attempting WiFi reset");
  
  /* Stop WiFi */
  esp_wifi_stop();
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  /* Clear any existing event bits */
  if (s_wifi_event_group) {
    xEventGroupClearBits(s_wifi_event_group, 
                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
  }
  
  /* Restart WiFi */
  esp_err_t ret = esp_wifi_start();
  if (ret != ESP_OK) {
    ESP_LOGE(wifi_tag, "Failed to restart WiFi: %s", esp_err_to_name(ret));
    return ret;
  }
  
  return ESP_OK;
}

/* Public Functions ***********************************************************/

esp_err_t wifi_check_connection(void)
{
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (netif == NULL) {
    ESP_LOGE(wifi_tag, "Network interface not found.");
    error_handler_record_error(&s_wifi_error_handler, ESP_ERR_INVALID_STATE);
    return ESP_FAIL;
  }

  esp_netif_ip_info_t ip_info;
  if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK || ip_info.ip.addr == 0) {
    ESP_LOGW(wifi_tag, "No IP address. Wi-Fi might be disconnected.");
    esp_err_t error = error_handler_record_error(&s_wifi_error_handler, ESP_ERR_WIFI_NOT_CONNECT);
    if (error == ESP_OK) {
      /* Attempt to reconnect */
      esp_wifi_connect();
    }
    return ESP_FAIL;
  }

  /* Connection is good, reset error state */
  error_handler_reset(&s_wifi_error_handler);
  return ESP_OK;
}

esp_err_t wifi_init_sta(void)
{
  ESP_LOGI(wifi_tag, "Starting WiFi initialization in station mode.");

  /* Initialize error handler */
  error_handler_init(&s_wifi_error_handler,
                     wifi_tag,
                     wifi_max_retry,
                     wifi_initial_retry_interval,
                     wifi_max_retry_interval,
                     priv_wifi_reset,
                     NULL,
                     wifi_backoff_interval,
                     wifi_max_backoff_interval);

  s_wifi_event_group = xEventGroupCreate();
  if (!s_wifi_event_group) {
    ESP_LOGE(wifi_tag, "Failed to create event group.");
    error_handler_record_error(&s_wifi_error_handler, ESP_ERR_NO_MEM);
    return ESP_ERR_NO_MEM;
  }

  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &priv_event_handler, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &priv_event_handler, NULL, NULL);

  wifi_config_t wifi_config = {};
  strncpy((char *)wifi_config.sta.ssid, wifi_ssid, wifi_ssid_max_len - 1);
  strncpy((char *)wifi_config.sta.password, wifi_pass, wifi_pass_max_len - 1);
  if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK || 
      esp_wifi_set_config(WIFI_IF_STA, &wifi_config) != ESP_OK) {
    error_handler_record_error(&s_wifi_error_handler, ESP_ERR_WIFI_NOT_INIT);
    return ESP_FAIL;
  }
  esp_wifi_start();

  ESP_LOGI(wifi_tag, "Starting connection timeout timer.");
  s_wifi_connect_timer = xTimerCreate("WiFiConnectTimer",
                                      pdMS_TO_TICKS(wifi_connect_timeout_ms),
                                      pdFALSE,
                                      NULL,
                                      priv_wifi_connect_timeout_cb);
  if (!s_wifi_connect_timer) {
    ESP_LOGE(wifi_tag, "Failed to create connection timeout timer.");
    if (s_wifi_event_group) {
      vEventGroupDelete(s_wifi_event_group);
      s_wifi_event_group = NULL;
    }
    error_handler_record_error(&s_wifi_error_handler, ESP_ERR_NO_MEM);
    return ESP_ERR_NO_MEM;
  }
  xTimerStart(s_wifi_connect_timer, 0);

  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(wifi_tag, "Successfully connected to AP.");
    return ESP_OK;
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGW(wifi_tag, "Failed to connect to AP within timeout.");
    return ESP_FAIL;
  } else {
    ESP_LOGE(wifi_tag, "Unexpected event occurred.");
    return ESP_FAIL;
  }
}

static void wifi_task(void *pvParameters) 
{
  /* Initialize time with default values first */
  time_manager_init(); /* This will set default time if WiFi is not available */
    
  while (1) {
    if (wifi_init_sta() == ESP_OK) {
      ESP_LOGI(wifi_tag, "WiFi connected successfully");
            
      /* Now that WiFi is connected, re-initialize time to get actual time from NTP */
      if (time_manager_init() != ESP_OK) {
        ESP_LOGE(wifi_tag, "Time synchronization failed, will retry after WiFi reconnection");
      } else {
        ESP_LOGI(wifi_tag, "Time synchronized successfully");
      }
            
      /* Monitor connection */
      while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); /* Check every 10 seconds */
        if (wifi_check_connection() != ESP_OK) {
          ESP_LOGW(wifi_tag, "WiFi connection lost, will attempt reconnection");
          break;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5000)); /* Wait before retry */
  }
}

esp_err_t wifi_task_start(void) 
{
  BaseType_t ret = xTaskCreate(wifi_task,
                               "wifi_task",
                               4096,
                               NULL,
                               5, /* Priority */ /* TODO: Verify priority */
                               &s_wifi_task_handle);
    
  if (ret != pdPASS) {
    ESP_LOGE(wifi_tag, "Failed to create WiFi task");
    return ESP_FAIL;
  }
    
  return ESP_OK;
}
