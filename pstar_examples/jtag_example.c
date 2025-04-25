/* pstar_examples/jtag_example.c */

#include "sdkconfig.h"

/* The JTAG utility itself depends on its Kconfig enable flag, */
/* so we don't need a separate top-level guard for the *example*, */
/* but we should still check inside the example logic. */

#include "pstar_jtag.h"          /* Include the JTAG utility header */
#include "pstar_pin_validator.h" /* Include for registration part */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "jtag_example.h" /* Include the example's own header */

static const char* TAG_EX = "JTAG_Example";

void example_jtag_app_main(void)
{
  ESP_LOGI(TAG_EX, "Running: JTAG Configuration Example");
  esp_err_t    ret;
  pstar_jtag_t jtag_pin_config;
  bool         validator_used = false;

/* --- Check if JTAG itself is enabled --- */
#if !CONFIG_PSTAR_KCONFIG_JTAG_ENABLED
  ESP_LOGE(TAG_EX, "JTAG component is disabled in Kconfig!");
  ESP_LOGE(TAG_EX, "This example requires JTAG support to be enabled.");
  goto example_halt; /* Halt if the core component is disabled */
#endif

/* --- Setup Step 1: Register JTAG Pins (if validator enabled) --- */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  validator_used = true;
  ESP_LOGI(TAG_EX, "Registering JTAG example pins...");
  /* This function is guarded internally */
  ret = pstar_jtag_register_kconfig_pins();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to register JTAG pins: %s", esp_err_to_name(ret));
    goto example_fail; /* Treat as fatal */
  }
  /* --- Setup Step 2: Validate Registered Pins --- */
  ESP_LOGI(TAG_EX, "Validating example pins...");
  ret = pstar_validate_pins();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Pin validation failed for JTAG example: %s", esp_err_to_name(ret));
    goto example_fail; /* Treat as fatal */
  }
  ESP_LOGI(TAG_EX, "Example Pin validation successful.");
#else
  ESP_LOGW(TAG_EX, "Pin Validator disabled. Skipping pin checks for JTAG example.");
  validator_used = false;
#endif

  /* --- Example Logic: Get and Print JTAG Pins --- */
  ESP_LOGI(TAG_EX, "Retrieving JTAG pin configuration from Kconfig...");

  /* This function is already guarded internally by the JTAG C file */
  ret = pstar_get_jtag_pins(&jtag_pin_config);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG_EX, "Retrieved JTAG Pins:");
    ESP_LOGI(TAG_EX, "  TCK: GPIO %d", jtag_pin_config.tck);
    ESP_LOGI(TAG_EX, "  TMS: GPIO %d", jtag_pin_config.tms);
    ESP_LOGI(TAG_EX, "  TDI: GPIO %d", jtag_pin_config.tdi);
    ESP_LOGI(TAG_EX, "  TDO: GPIO %d", jtag_pin_config.tdo);
    ESP_LOGI(TAG_EX, "(Values depend on target MCU and Kconfig mode - Default/Custom)");
  } else if (ret == ESP_ERR_NOT_SUPPORTED) {
    /* This case should already be caught by the #if check above, but handle defensively */
    ESP_LOGE(TAG_EX, "JTAG support is disabled in Kconfig. Cannot retrieve pins.");
  } else {
    ESP_LOGE(TAG_EX, "Failed to retrieve JTAG pins: %s", esp_err_to_name(ret));
    /* Could be ESP_ERR_INVALID_STATE if pins are invalid (-1) */
    goto example_fail;
  }

  ESP_LOGI(TAG_EX, "JTAG example finished successfully.");

example_fail:
  /* Cleanup resources */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  if (validator_used) {
    ESP_LOGI(TAG_EX, "Freeing pin validator resources...");
    esp_err_t pin_val_ret = pstar_free_pin_validator();
    if (pin_val_ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "Pin validator free failed: %s", esp_err_to_name(pin_val_ret));
    }
  }
#endif
}

/* Note: No stub function needed here as the example logic */
/* itself checks the JTAG Kconfig flag. */
