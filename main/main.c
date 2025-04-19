/* main/main.c */

#include "pstar_bh1750_hal.h"
#include "pstar_bus_manager.h"
#include "pstar_jtag.h"
#include "pstar_pin_validator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>

#include "esp_check.h"
#include "esp_log.h"

static const char* TAG = "Project Star";

/* --- Global Bus Manager Instance --- */
static pstar_bus_manager_t g_bus_manager;

void app_main(void)
{
  ESP_LOGI(TAG, "Application Start: BH1750 Example with Bus Manager");

  esp_err_t           ret                 = ESP_OK;
  bh1750_hal_handle_t bh1750_handle       = NULL;
  bool                manager_initialized = false;
  bool                hal_initialized     = false;

  /* 1. Initialize the Bus Manager */
  ESP_LOGI(TAG, "Initializing Bus Manager...");
  ret = pstar_bus_manager_init(&g_bus_manager, "MainBusMgr");
  ESP_GOTO_ON_ERROR(ret,
                    fatal_error,
                    TAG,
                    "Failed to initialize Bus Manager: %s",
                    esp_err_to_name(ret));
  manager_initialized = true;

  /* 2. Register Pins for Specific Components (if validator enabled) */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  ESP_LOGI(TAG, "Registering component pins with validator...");

  /* Register BH1750 pins (function handles internal check for BH1750 enabled) */
#if CONFIG_PSTAR_KCONFIG_BH1750_ENABLED
  ret = register_bh1750_pins();
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to register BH1750 pins: %s", esp_err_to_name(ret));
#endif

  /* --- Example: Register JTAG pins if enabled --- */
#if CONFIG_PSTAR_KCONFIG_JTAG_ENABLED
  pstar_jtag_t jtag_pins;
  ret = pstar_get_jtag_pins(&jtag_pins);
  if (ret == ESP_OK) {
    /* Register JTAG pins (usually not shared) */
    ret = pstar_register_pin(jtag_pins.tck, "JTAG TCK", false);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed to register JTAG TCK pin: %s",
                      esp_err_to_name(ret));

    ret = pstar_register_pin(jtag_pins.tms, "JTAG TMS", false);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed to register JTAG TMS pin: %s",
                      esp_err_to_name(ret));

    ret = pstar_register_pin(jtag_pins.tdi, "JTAG TDI", false);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed to register JTAG TDI pin: %s",
                      esp_err_to_name(ret));

    ret = pstar_register_pin(jtag_pins.tdo, "JTAG TDO", false);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed to register JTAG TDO pin: %s",
                      esp_err_to_name(ret));

    ESP_LOGI(TAG, "JTAG pins registered.");
  } else if (ret != ESP_ERR_NOT_SUPPORTED) { /* Ignore error if JTAG just isn't enabled */
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed to get JTAG pins for registration: %s",
                      esp_err_to_name(ret));
  }
#endif /* CONFIG_PSTAR_KCONFIG_JTAG_ENABLED */
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED */

  /* 3. Create and Initialize BH1750 using default KConfig values */
#if CONFIG_PSTAR_KCONFIG_BH1750_ENABLED
  ret = bh1750_hal_create_default(&g_bus_manager, &bh1750_handle);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to create and initialize BH1750: %s",
                    esp_err_to_name(ret));
  hal_initialized = true;
#endif /* CONFIG_PSTAR_KCONFIG_BH1750_ENABLED */

  /* 4. Validate All Registered Pins (if validator enabled) */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  ESP_LOGI(TAG, "Validating all registered pins...");
  ret = pstar_validate_pins();
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Pin validation failed: %s", esp_err_to_name(ret));
  ESP_LOGI(TAG, "Pin validation successful.");
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED */

  /* 5. Main Application Logic */
  ESP_LOGI(TAG, "Setup complete. Entering main loop.");
  float current_lux = 0.0f;
  while (1) {
#if CONFIG_PSTAR_KCONFIG_BH1750_ENABLED
    ret = bh1750_hal_read_lux(bh1750_handle, &current_lux);
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "BH1750 Light Level: %.2f Lux", current_lux);
    } else {
      ESP_LOGE(TAG, "Failed to read BH1750 data: %s (%d)", esp_err_to_name(ret), ret);
      /* Consider adding error handling/recovery here using pstar_error_handler */
    }
#else
    ESP_LOGI(TAG, "BH1750 is disabled. Doing nothing in main loop.");
#endif /* CONFIG_PSTAR_KCONFIG_BH1750_ENABLED */

    /* Use the Kconfig value for the read interval directly */
    vTaskDelay(pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_BH1750_READ_INTERVAL_MS));
  }

cleanup:
  ESP_LOGW(TAG, "Starting cleanup sequence due to error or exit...");

#if CONFIG_PSTAR_KCONFIG_BH1750_ENABLED
  if (hal_initialized) {
    ESP_LOGI(TAG, "Deinitializing BH1750 HAL...");
    /* Use the Kconfig value directly here too */
    bool power_down = false;
#ifdef CONFIG_PSTAR_KCONFIG_BH1750_POWER_SAVE_MODE
    power_down = true;
#endif
    esp_err_t deinit_ret = bh1750_hal_deinit(bh1750_handle, power_down);
    if (deinit_ret != ESP_OK) {
      ESP_LOGE(TAG, "BH1750 HAL deinit failed: %s", esp_err_to_name(deinit_ret));
    }
  }
#endif /* CONFIG_PSTAR_KCONFIG_BH1750_ENABLED */

  /* Deinitialize Bus Manager (this will deinit/destroy buses added by create_default) */
  if (manager_initialized) {
    ESP_LOGI(TAG, "Deinitializing Bus Manager...");
    esp_err_t deinit_ret = pstar_bus_manager_deinit(&g_bus_manager);
    if (deinit_ret != ESP_OK) {
      ESP_LOGE(TAG, "Bus Manager deinit reported errors: %s", esp_err_to_name(deinit_ret));
    }
  }

  /* Free pin validator resources (should be done after bus manager deinit potentially) */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  ESP_LOGI(TAG, "Freeing pin validator resources...");
  esp_err_t pin_validator_ret = pstar_free_pin_validator();
  if (pin_validator_ret != ESP_OK) {
    ESP_LOGE(TAG, "Pin validator free failed: %s", esp_err_to_name(pin_validator_ret));
  }
#endif

fatal_error: /* Label used by ESP_GOTO_ON_ERROR */
  ESP_LOGE(TAG, "Application ending.");
  /* Infinite loop or restart logic */
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}