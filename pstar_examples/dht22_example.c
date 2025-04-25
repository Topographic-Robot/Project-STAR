/* pstar_examples/dht22_example.c */

#include "sdkconfig.h"

/* Only compile if DHT22 component is enabled */
#ifdef CONFIG_PSTAR_KCONFIG_DHT22_ENABLED

#include "pstar_dht22_hal.h"
#include "pstar_pin_validator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

#include "dht22_example.h"
#include "esp_check.h" /* For ESP_ERROR_CHECK */
#include "esp_log.h"

static const char* TAG_EX = "DHT22_Example";

void example_dht22_app_main(void)
{
  ESP_LOGI(TAG_EX, "Running: DHT22 Example");
  esp_err_t                ret            = ESP_OK;
  pstar_dht22_hal_handle_t dht22_handle   = NULL;
  bool                     validator_used = false; /* Track if validator was active */

/* --- Setup Step 1: Register Example Pins (if validator enabled) --- */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  validator_used = true;
  ESP_LOGI(TAG_EX, "Registering DHT22 example pins...");
  /* Use Kconfig pins for the example */
  ret = pstar_dht22_register_kconfig_pins();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to register DHT22 pins: %s", esp_err_to_name(ret));
    goto example_fail; /* Treat as fatal */
  }
  /* --- Setup Step 2: Validate Registered Pins --- */
  ESP_LOGI(TAG_EX, "Validating example pins...");
  ret = pstar_validate_pins();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Pin validation failed for example: %s", esp_err_to_name(ret));
    goto example_fail; /* Treat as fatal */
  }
  ESP_LOGI(TAG_EX, "Example Pin validation successful.");
#else
  ESP_LOGW(TAG_EX, "Pin Validator disabled. Skipping pin checks for example.");
  validator_used = false;
#endif

  /* --- Setup Step 3: Create DHT22 device --- */
  ESP_LOGI(TAG_EX, "Initializing DHT22 HAL...");
  /* This function is already guarded internally by the HAL C file */
  ret = pstar_dht22_hal_create_kconfig_default(&dht22_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to create DHT22 default: %s", esp_err_to_name(ret));
    goto example_fail;
  }
  ESP_LOGI(TAG_EX, "DHT22 Initialized Successfully.");

  /* --- Setup Step 4: Example Loop --- */
  ESP_LOGI(TAG_EX, "Starting example loop...");
  while (1) {
    float temperature = NAN;
    float humidity    = NAN;

    ret = pstar_dht22_hal_read_data(dht22_handle, &humidity, &temperature);
    if (ret == ESP_OK) {
      ESP_LOGI(TAG_EX, "Temperature: %.1f C, Humidity: %.1f %%", temperature, humidity);
    } else {
      ESP_LOGE(TAG_EX, "Failed to read DHT22 data: %s (%d)", esp_err_to_name(ret), ret);
      /* Specific handling for common errors */
      if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG_EX, "Sensor timeout. Check wiring and pull-up resistor.");
      } else if (ret == ESP_ERR_INVALID_CRC) {
        ESP_LOGW(TAG_EX, "Checksum error. Possible interference or timing issue.");
      }
    }

    /* Delay between reads (DHT22 minimum is ~2 seconds) */
    vTaskDelay(pdMS_TO_TICKS(3000));
  }

example_fail:
  /* Cleanup resources if setup failed or loop exited unexpectedly */
  ESP_LOGE(TAG_EX, "Example failed or ending. Cleaning up and halting.");
  if (dht22_handle) {
    esp_err_t deinit_ret = pstar_dht22_hal_deinit(dht22_handle);
    if (deinit_ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "DHT22 HAL deinit failed: %s", esp_err_to_name(deinit_ret));
    }
  }
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  if (validator_used) {
    ESP_LOGI(TAG_EX, "Freeing pin validator resources...");
    esp_err_t pin_val_ret = pstar_free_pin_validator();
    if (pin_val_ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "Pin validator free failed: %s", esp_err_to_name(pin_val_ret));
    }
  }
#endif
  /* Halt */
  ESP_LOGI(TAG_EX, "Example finished cleanup. Halting task.");
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}

#else /* CONFIG_PSTAR_KCONFIG_DHT22_ENABLED */

/* Provide a stub function if the component is disabled */
#include "esp_log.h"
/* Include FreeRTOS headers needed ONLY for the halt loop in the stub */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG_EX_STUB = "DHT22_Example";
void               example_dht22_app_main(void)
{
  ESP_LOGE(TAG_EX_STUB,
           "DHT22 Example is selected in Kconfig, but the DHT22 component is disabled!");
  ESP_LOGE(
    TAG_EX_STUB,
    "Please enable 'PSTAR_KCONFIG_DHT22_ENABLED' or select a different example/Main Application.");
  while (1) {
    vTaskDelay(portMAX_DELAY); /* Halt */
  }
}

#endif /* CONFIG_PSTAR_KCONFIG_DHT22_ENABLED */
