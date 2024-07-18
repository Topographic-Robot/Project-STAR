/* wifi-sta, sta is short for station, station means that this will connect to
 * another wifi network such as a mobile hotspot */
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_wifi_types_generic.h"
#include "freertos/event_groups.h"
#include "nvs.h"
#include "nvs_flash.h"

#define WIFI_MAX_RETRY 10
/* Creds for my local coffee shop */
#define WIFI_SSID      "EpochCoffee"
#define WIFI_PASS      "epochcoffee"

/* The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "WiFi Station";

/* This is a global event handler used for event event management and
 * synchronization between tasks. An event group is a collection of event bits
 * (flags) that tasks can set, clear, or wait on */
static EventGroupHandle_t s_wifi_event_group = NULL;

/*
 * Function: event_handler
 * ------------------------
 * Handles various events related to WiFi and IP connectivity.
 *
 * Parameters:
 *   arg         - A pointer to the argument provided during event registration.
 *   event_base  - The base identifier for the event (e.g., WIFI_EVENT or
 *                 IP_EVENT).
 *   event_id    - The specific event identifier (e.g., WIFI_EVENT_STA_START).
 *   event_data  - A pointer to the event data specific to the event type.
 *
 * This function handles two main categories of events:
 *   1. WiFi Events: Handles WiFi station start and disconnection events.
 *      - WIFI_EVENT_STA_START: Initiates connection to the Access Point (AP).
 *      - WIFI_EVENT_STA_DISCONNECTED: Attempts to reconnect to the AP on
 *        disconnection, with a maximum retry limit.
 *   2. IP Events: Handles the event when the station gets an IP address.
 *      - IP_EVENT_STA_GOT_IP: Logs the obtained IP address and sets a bit
 *        indicating successful connection.
 */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  /* the s_ prefix is common in c to express the variable is static */
  static uint8_t s_retry_num = 0;

  /* NOTE:
   * esp_event_base_t is a `const char *` and WIFI_EVENT is a
   * `const char *`, but in C you cannot compare two `const char *` with ==
   * However, since we are using a microcontroller, to save memory: esp has made
   * it that the event_base will be set to the address of WIFI_EVENT when the
   * event is a WiFi event. This is why we can compare with == and we don't need
   * to use strcmp.
   *
   * The same is for the other events such as IP events and this is also how
   * event_id's are programmed */

  /* Check for WiFi events */
  if (event_base == WIFI_EVENT) {
    /* Check if the station is started */
    if (event_id == WIFI_EVENT_STA_START) {
      esp_wifi_connect(); /* We can now connect to the network */
      ESP_LOGI(TAG, "try to connect to the AP");
    }
    /* Check if we have been disconnected */
    if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
      ESP_LOGI(TAG, "connect to the AP fail");
      /* Try and reconnect */
      if (s_retry_num < WIFI_MAX_RETRY) {
        esp_wifi_connect();
        s_retry_num++;
        ESP_LOGI(TAG, "retry to connect to the AP");
      } else {
        ESP_LOGI(TAG, "Failed to connect to the AP after maximum retries");
        /* We can set the bit to express that it failed */
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
      }
    }
  }

  /* Check for IP events */
  if (event_base == IP_EVENT) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    /* An IP event was recieved so the wifi is connected, set the bit */
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

/*
 * Function: wifi_init_sta
 * ------------------------
 * Initializes the WiFi in station mode and connects to the specified Access
 * Point (AP).
 *
 * This function performs the following steps:
 * 1.  Creates an event group to manage WiFi events.
 * 2.  Initializes the underlying TCP/IP stack.
 * 3.  Creates the default event loop required for handling events.
 * 4.  Sets up the default WiFi station (STA) network interface.
 * 5.  Initializes the WiFi driver with the default configuration.
 * 6.  Registers event handlers for WiFi events and IP events to handle
 *     connection status and IP acquisition.
 * 7.  Configures the WiFi with the specified SSID and password, including
 *     authentication mode and SAE settings.
 * 8.  Sets the WiFi mode to station (STA) mode.
 * 9.  Starts the WiFi driver.
 * 10. Waits for the WiFi connection to be established or to fail, and logs the
 *     connection status.
 *
 * The function uses several ESP-IDF APIs to perform these tasks, and it checks
 * for errors at each step using the ESP_ERROR_CHECK macro. If any
 * initialization step fails, the program will log an error and terminate.
 *
 * The function also handles WiFi events through the event handler, which sets
 * the appropriate bits in the event group to signal connection success or
 * failure.
 */
void wifi_init_sta(void) {
  /* creates a new RTOS event group, and returns a handle by which the newly
   * created event group can be referenced */
  s_wifi_event_group = xEventGroupCreate();

  if (!s_wifi_event_group) {
    /* The event group was not created because there was insufficient FreeRTOS
     * heap available. */
    ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
  }

  /* Initialize the underlying TCP/IP stack. This function should be called
   * exactly once from application code, when the application starts up */
  ESP_ERROR_CHECK(esp_netif_init());

  /* the esp_event_loop_create_default function initializes and creates a
   * default event loop in the system. This default event loop can then be used
   * to register and handle events, allowing different parts of the application
   * to communicate asynchronously through events */
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  /* The API creates esp_netif object with default WiFi station config, attaches
   * the netif to wifi and registers wifi handlers to the default event loop.
   * This API uses assert() to check for potential errors, so it could abort the
   * program. (Note that the default event loop needs to be created prior to
   * calling this API) */
  esp_netif_create_default_wifi_sta();

  /* Always use WIFI_INIT_CONFIG_DEFAULT macro to initialize the configuration
   * to default values, this can guarantee all the fields get correct value when
   * more fields are added into wifi_init_config_t in future release. If you
   * want to set your own initial values, overwrite the default values which are
   * set by WIFI_INIT_CONFIG_DEFAULT. Please be notified that the field
   * 'magic' of wifi_init_config_t should always be WIFI_INIT_CONFIG_MAGIC! */
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  /* Initialize the WiFi driver with the provided configuration. This function
   * sets up the WiFi driver based on the configuration specified in the
   * wifi_init_config_t structure, which includes settings for various WiFi
   * parameters */
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  /* Declare event handler instances for different WiFi events */
  esp_event_handler_instance_t instance_any_id, instance_got_ip;

  /* Register an event handler instance for any WiFi event. This function
   * associates the specified event handler function with any event ID under the
   * WiFi event base */
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));

  /* Register an event handler instance for the IP event when the station gets
   * an IP address. This function associates the specified event handler
   * function with the event ID for when the station (STA) gets an IP address */
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

  /* wifi_config_t is a union, so it can handle sta, ap, and nan */
  wifi_config_t wifi_config = {
      /* set up sta */
      .sta =
          {
              .ssid     = WIFI_SSID,
              .password = WIFI_PASS,
          },
  };

  /* set the WiFi mode to station (STA) mode */
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

  /* set the WiFi configuration with the provided wifi_config struct */
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

  /* start the WiFi with the current config */
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or
   * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT). The
   * bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we
   * can test which event actually happened. */
  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", WIFI_SSID, WIFI_PASS);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", WIFI_SSID,
             WIFI_PASS);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
}

/*
 * Function: app_main
 * ------------------
 * Main application entry point. Initializes non-volatile storage (NVS) and sets
 * up the WiFi station
 *
 * This function performs the following steps:
 * 1. Initializes the NVS (non-volatile storage), which is used to store data
 *    that persists across reboots
 * 2. Checks the return value of the NVS initialization to handle specific
 *    errors
 * 3. Calls the `wifi_init_sta` function to initialize the WiFi station mode and
 *    connect to the specified Access Point (AP)
 */
void app_main(void) {
  /* Initializes NVS (non volitile stoage) */
  esp_err_t ret = nvs_flash_init();

  /* No free pages means that the partition is full so it cannot allocate any
   * more data. New version means that the nvs needs to be updated to the newer
   * version that was found */
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    /* This macro checks the return value of a function provided by esp,
     * anything other than ESP_OK, the macro will print an error message and
     * abort the program */
    ESP_ERROR_CHECK(nvs_flash_erase()); /* Erase the NVS before init again */
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret); /* like before, check to see if nvs is good, but this
                         * time crash if its not good */

  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

  wifi_init_sta(); /* Initialize the wifi station */
}
