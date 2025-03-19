/* main/main.c */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

/* For Project Star components */
#include "log_handler.h"
#include "error_handler.h"

static const char* main_tag = "main";

void app_main(void)
{
 /* Initialize NVS (required for many components) */
 esp_err_t ret = nvs_flash_init();
 if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
   /* NVS partition was truncated and needs to be erased */
   ESP_ERROR_CHECK(nvs_flash_erase());
   ret = nvs_flash_init();
 }
 ESP_ERROR_CHECK(ret);
 
 ESP_LOGI(main_tag, "Project Star starting up");
 
 ESP_LOGI(main_tag, "Project Star initialization complete");
}
