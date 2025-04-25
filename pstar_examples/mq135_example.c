/* pstar_examples/mq135_example.c */

#include "sdkconfig.h"

/* Only compile if MQ135 component is enabled */
#ifdef CONFIG_PSTAR_KCONFIG_MQ135_ENABLED

#include "pstar_mq135_hal.h"
#include "pstar_pin_validator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h" /* For ESP_ERROR_CHECK */
#include "esp_log.h"
#include "mq135_example.h"

static const char* TAG_EX = "MQ135_Example";

void example_mq135_app_main(void)
{
  ESP_LOGI(TAG_EX, "Running: MQ135 Example");
  esp_err_t                ret            = ESP_OK;
  pstar_mq135_hal_handle_t mq135_handle   = NULL;
  bool                     validator_used = false; /* Track if validator was active */

/* --- Setup Step 1: Register Example Pins (if validator enabled) --- */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  validator_used = true;
  ESP_LOGI(TAG_EX, "Registering MQ135 example pins...");
  /* Use Kconfig pins for the example */
  ret = pstar_mq135_register_kconfig_pins();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to register MQ135 pins: %s", esp_err_to_name(ret));
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

  /* --- Setup Step 3: Create MQ135 device --- */
  ESP_LOGI(TAG_EX, "Initializing MQ135 HAL...");
  /* This function is already guarded internally by the HAL C file */
  ret = pstar_mq135_hal_create_kconfig_default(&mq135_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to create MQ135 default: %s", esp_err_to_name(ret));
    goto example_fail;
  }
  ESP_LOGI(TAG_EX, "MQ135 Initialized Successfully.");

  /* --- Setup Step 4: Example Loop --- */
  ESP_LOGI(TAG_EX, "Starting example loop...");
  ESP_LOGI(TAG_EX, "Reading MQ135 sensor (Raw ADC and Voltage)...");
  ESP_LOGW(
    TAG_EX,
    "Note: MQ135 readings require calibration for accurate PPM conversion. This example shows raw/voltage only.");

  while (1) {
    int raw_value  = -1;
    int voltage_mv = -1;

    /* Read raw value */
    ret = pstar_mq135_hal_read_raw(mq135_handle, &raw_value);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "Failed to read MQ135 raw value: %s", esp_err_to_name(ret));
    }

    /* Read voltage (if calibration was successful during init) */
    ret = pstar_mq135_hal_read_voltage(mq135_handle, &voltage_mv);
    if (ret == ESP_OK) {
      ESP_LOGI(TAG_EX, "MQ135 Reading: Raw=%d, Voltage=%d mV", raw_value, voltage_mv);
    } else if (ret == ESP_ERR_INVALID_STATE) {
      ESP_LOGW(TAG_EX, "MQ135 Reading: Raw=%d, Voltage=N/A (Calibration Failed)", raw_value);
    } else {
      ESP_LOGE(TAG_EX, "Failed to read MQ135 voltage: %s", esp_err_to_name(ret));
      /* Log raw value even if voltage fails */
      ESP_LOGI(TAG_EX, "MQ135 Reading: Raw=%d, Voltage=Error", raw_value);
    }

    vTaskDelay(pdMS_TO_TICKS(2000)); /* Delay between readings */
  }

example_fail:
  /* Cleanup resources if setup failed or loop exited unexpectedly */
  ESP_LOGE(TAG_EX, "Example failed or ending. Cleaning up and halting.");
  if (mq135_handle) {
    esp_err_t deinit_ret = pstar_mq135_hal_deinit(mq135_handle);
    if (deinit_ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "MQ135 HAL deinit failed: %s", esp_err_to_name(deinit_ret));
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

#else /* CONFIG_PSTAR_KCONFIG_MQ135_ENABLED */

/* Provide a stub function if the component is disabled */
#include "esp_log.h"
/* Include FreeRTOS headers needed ONLY for the halt loop in the stub */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG_EX_STUB = "MQ135_Example";
void               example_mq135_app_main(void)
{
  ESP_LOGE(TAG_EX_STUB,
           "MQ135 Example is selected in Kconfig, but the MQ135 component is disabled!");
  ESP_LOGE(
    TAG_EX_STUB,
    "Please enable 'PSTAR_KCONFIG_MQ135_ENABLED' or select a different example/Main Application.");
  while (1) {
    vTaskDelay(portMAX_DELAY); /* Halt */
  }
}

#endif /* CONFIG_PSTAR_KCONFIG_MQ135_ENABLED */
