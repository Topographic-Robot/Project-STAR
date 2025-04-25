/* pstar_examples/hd44780_example.c */

/* Only compile if HD44780 component is enabled */
#include "sdkconfig.h"
/* Keep this top-level check */
#ifdef CONFIG_PSTAR_KCONFIG_HD44780_ENABLED

#include "pstar_bus_manager.h"
#include "pstar_hd44780_hal.h"
#include "pstar_pin_validator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h" /* For ESP_ERROR_CHECK */
#include "esp_log.h"
#include "hd44780_example.h"

static const char* TAG_EX = "HD44780_Example";

void example_hd44780_app_main(void)
{
#if CONFIG_PSTAR_KCONFIG_HD44780_MODE_I2C
  ESP_LOGI(TAG_EX, "Running: HD44780 Example (I2C Mode)");
  const char* mode_str = "I2C";
#else
  ESP_LOGI(TAG_EX, "Running: HD44780 Example (Parallel Mode)");
  const char* mode_str = "Parallel";
#endif

  esp_err_t                  ret = ESP_OK;
  pstar_bus_manager_t        example_bus_manager; /* Needed even for parallel to pass to init */
  pstar_hd44780_hal_handle_t lcd_handle          = NULL;
  bool                       manager_initialized = false;
  bool                       validator_used      = false;

  /* --- Setup Step 1: Register Example Pins (if validator enabled) --- */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  validator_used = true;
  ESP_LOGI(TAG_EX, "Registering HD44780 example pins (%s mode)...", mode_str);
  /* Let the HAL's Kconfig registration function handle mode logic */
  ret = pstar_hd44780_register_kconfig_pins();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to register HD44780 pins: %s", esp_err_to_name(ret));
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

  /* --- Setup Step 3: Init Bus Manager (Required for Kconfig default creation) --- */
  ESP_LOGI(TAG_EX, "Initializing Example Bus Manager...");
  ret = pstar_bus_manager_init(&example_bus_manager, "HD44780_Example_BusMgr");
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to init example bus manager: %s", esp_err_to_name(ret));
    goto example_fail; /* Cannot proceed */
  }
  manager_initialized = true;

  /* --- Setup Step 4: Create HD44780 device using Kconfig defaults --- */
  /* This function now handles both Parallel and I2C mode based on Kconfig */
  ESP_LOGI(TAG_EX, "Initializing HD44780 HAL from Kconfig (%s mode)...", mode_str);
  /* This function is already guarded internally by the HAL C file */
  ret = pstar_hd44780_hal_create_kconfig_default(&example_bus_manager, &lcd_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to create HD44780 default: %s", esp_err_to_name(ret));
    goto example_fail;
  }
  ESP_LOGI(TAG_EX, "HD44780 Initialized Successfully.");

  /* --- Setup Step 5: Example Loop --- */
  ESP_LOGI(TAG_EX, "Starting example loop...");
  int  count           = 0;
  bool backlight_state = true; /* Only relevant for I2C */

  while (1) {
    ESP_LOGD(TAG_EX, "Loop iteration %d", count);

    /* Clear Display */
    ESP_LOGD(TAG_EX, "Clearing display...");
    ret = pstar_hd44780_hal_clear(lcd_handle);
    ESP_ERROR_CHECK(ret);          /* Halt on error in the loop */
    vTaskDelay(pdMS_TO_TICKS(50)); /* Short delay after clear */

    /* Set cursor and print */
    ESP_LOGD(TAG_EX, "Printing Line 1...");
    ret = pstar_hd44780_hal_set_cursor(lcd_handle, 0, 0);
    ESP_ERROR_CHECK(ret);
    ret = pstar_hd44780_hal_print_string(lcd_handle, "Project-STAR");
    ESP_ERROR_CHECK(ret);

    ESP_LOGD(TAG_EX, "Printing Line 2...");
    ret = pstar_hd44780_hal_set_cursor(lcd_handle, 0, 1);
    ESP_ERROR_CHECK(ret);
    ret = pstar_hd44780_hal_printf(lcd_handle, "%s Mode Cnt:%d", mode_str, count);
    ESP_ERROR_CHECK(ret);

    /* Toggle backlight (I2C mode only) */
#if CONFIG_PSTAR_KCONFIG_HD44780_MODE_I2C
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (backlight_state) {
      ESP_LOGD(TAG_EX, "Turning backlight OFF");
      ret = pstar_hd44780_hal_backlight_off(lcd_handle);
    } else {
      ESP_LOGD(TAG_EX, "Turning backlight ON");
      ret = pstar_hd44780_hal_backlight_on(lcd_handle);
    }
    ESP_ERROR_CHECK(ret);
    backlight_state = !backlight_state;
#endif /* CONFIG_PSTAR_KCONFIG_HD44780_MODE_I2C */

    count++;
    vTaskDelay(pdMS_TO_TICKS(3000)); /* Wait before next update */

  } /* End while(1) */

example_fail:
  /* Cleanup resources if setup failed or loop exited unexpectedly */
  ESP_LOGE(TAG_EX, "Example failed or ending. Cleaning up and halting.");
  if (lcd_handle) {
    /* Attempt clear before deinit, ignore error */
    pstar_hd44780_hal_clear(lcd_handle);
    esp_err_t deinit_ret = pstar_hd44780_hal_deinit(lcd_handle, false);
    if (deinit_ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "LCD HAL deinit failed: %s", esp_err_to_name(deinit_ret));
    }
  }
  if (manager_initialized) {
    esp_err_t bm_deinit_ret = pstar_bus_manager_deinit(&example_bus_manager);
    if (bm_deinit_ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "Bus Manager deinit reported errors: %s", esp_err_to_name(bm_deinit_ret));
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
    vTaskDelay(portMAX_DELAY); /* This is safe now as headers are included */
  }
}

#else /* CONFIG_PSTAR_KCONFIG_HD44780_ENABLED */

/* Provide a stub function if the component is disabled */
#include "esp_log.h"
/* Include FreeRTOS headers needed ONLY for the halt loop in the stub */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG_EX_STUB = "HD44780_Example";
void               example_hd44780_app_main(void)
{
  ESP_LOGE(TAG_EX_STUB,
           "HD44780 Example is selected in Kconfig, but the HD44780 component is disabled!");
  ESP_LOGE(
    TAG_EX_STUB,
    "Please enable 'PSTAR_KCONFIG_HD44780_ENABLED' or select a different example/Main Application.");
  while (1) {
    vTaskDelay(portMAX_DELAY); /* Halt */
  }
}

#endif /* CONFIG_PSTAR_KCONFIG_HD44780_ENABLED */