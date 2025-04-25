/* pstar_examples/bh1750_example.c */

#include "sdkconfig.h"

#ifdef CONFIG_PSTAR_KCONFIG_BH1750_ENABLED

#include "pstar_bh1750_hal.h"
#include "pstar_bus_manager.h"
#include "pstar_pin_validator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bh1750_example.h"
#include "esp_log.h"

static const char* TAG_EX = "BH1750_Example";

void example_bh1750_app_main(void)
{
  ESP_LOGI(TAG_EX, "Running: BH1750 Example");
  esp_err_t           ret = ESP_OK;
  pstar_bus_manager_t example_bus_manager;
  bh1750_hal_handle_t bh1750_handle       = NULL;
  bool                manager_initialized = false;
  bool                validator_used      = false; /* Track if validator was active */

/* --- Setup Step 1: Register Example Pins (if validator enabled) --- */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  validator_used = true;
  ESP_LOGI(TAG_EX, "Registering BH1750 example pins...");
  ret = pstar_bh1750_register_kconfig_pins(); /* Use Kconfig pins for the example */
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to register BH1750 pins: %s", esp_err_to_name(ret));
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

  /* --- Setup Step 3: Init Bus Manager (local to this example) --- */
  ESP_LOGI(TAG_EX, "Initializing Example Bus Manager...");
  ret = pstar_bus_manager_init(&example_bus_manager, "BH1750_Example_BusMgr");
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to init example bus manager: %s", esp_err_to_name(ret));
    goto example_fail; /* Cannot proceed */
  }
  manager_initialized = true;

  /* --- Setup Step 4: Create BH1750 device --- */
  ESP_LOGI(TAG_EX, "Initializing BH1750 HAL...");
  /* This function is already guarded internally by the HAL C file */
  ret = pstar_bh1750_hal_create_kconfig_default(&example_bus_manager, &bh1750_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to create BH1750 default: %s", esp_err_to_name(ret));
    goto example_fail;
  }
  ESP_LOGI(TAG_EX, "BH1750 Initialized Successfully.");

  /* --- Setup Step 5: Example Loop --- */
  ESP_LOGI(TAG_EX, "Starting example loop...");
  while (1) {
    float lux;
    ret = pstar_bh1750_hal_read_lux(bh1750_handle, &lux);
    if (ret == ESP_OK) {
      ESP_LOGI(TAG_EX, "Ambient Light: %.2f Lux", lux);
    } else {
      ESP_LOGE(TAG_EX, "Failed to read BH1750: %s", esp_err_to_name(ret));
      /* Optional: Implement error handling/retry based on the error */
    }
    /* The Kconfig macro is only used if the component is enabled, so this is safe */
    vTaskDelay(pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_BH1750_READ_INTERVAL_MS));
  }

example_fail:
  /* Cleanup resources if setup failed */
  ESP_LOGE(TAG_EX, "Example setup failed or ending. Cleaning up and halting.");
  if (bh1750_handle) {
    pstar_bh1750_hal_deinit(bh1750_handle, false);
  }
  if (manager_initialized) {
    pstar_bus_manager_deinit(&example_bus_manager);
  }
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  if (validator_used) {
    ESP_LOGI(TAG_EX, "Freeing pin validator resources...");
    pstar_free_pin_validator();
  }
#endif
  /* Halt */
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}

#else /* CONFIG_PSTAR_KCONFIG_BH1750_ENABLED */

/* Provide a stub function if the component is disabled */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static const char* TAG_EX_STUB = "BH1750_Example";
void               example_bh1750_app_main(void)
{
  ESP_LOGE(TAG_EX_STUB,
           "BH1750 Example is selected in Kconfig, but the BH1750 component is disabled!");
  ESP_LOGE(
    TAG_EX_STUB,
    "Please enable 'PSTAR_KCONFIG_BH1750_ENABLED' or select a different example/Main Application.");
  /* Halt */
  while (1) {
    /* Include FreeRTOS headers needed for vTaskDelay OR use a simple busy loop */
    /* For safety, let's include the headers needed just for this simple halt */
    vTaskDelay(portMAX_DELAY);
  }
}

#endif /* CONFIG_PSTAR_KCONFIG_BH1750_ENABLED */