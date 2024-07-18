/* wifi-sta, sta is short for station, station means that this will connect to
 * another wifi network such as a mobile hotspot */
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "freertos/event_groups.h"
#include "nvs.h"
#include "nvs_flash.h"

/* This is a global event handler used for event event management and
 * synchronization between tasks. An event group is a collection of event bits
 * (flags) that tasks can set, clear, or wait on */
static EventGroupHandle_t s_wifi_event_group = NULL;

void wifi_init_sta(void) {
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
}

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

  wifi_init_sta(); /* Initialize the wifi station */
}
