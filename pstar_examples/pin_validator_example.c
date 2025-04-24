/* pstar_examples/pin_validator_example.c */

#include "pin_validator_example.h"

#include "pstar_pin_validator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "sdkconfig.h" /* Needed for Kconfig checks */

static const char* TAG_EX = "PinValidator_Example";

void example_pin_validator_app_main(void)
{
  ESP_LOGI(TAG_EX, "Running: Pin Validator Example");
  esp_err_t ret;

  /* --- Prerequisite Check --- */
  /* Ensure the validator component itself is enabled in Kconfig */
#if !CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  ESP_LOGE(TAG_EX, "Pin Validator component is disabled in Kconfig!");
  ESP_LOGE(TAG_EX, "This example requires the Pin Validator to be enabled.");
  goto example_halt; /* Halt if the core component is disabled */
#endif

  /* --- Phase 1: Demonstrate Conflict Detection --- */
  ESP_LOGI(TAG_EX, "*** Phase 1: Demonstrating Conflict Detection ***");

  /* Simulate Component A registering a non-shareable pin */
  ESP_LOGI(TAG_EX, "Component A registering GPIO 18 (Non-Shareable)...");
  ret = pstar_register_pin(GPIO_NUM_18, "Component A - UART TX", false);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed initial registration for Comp A: %s", esp_err_to_name(ret));
    goto example_fail; /* Treat unexpected registration failure as fatal */
  }

  /* Simulate Component B registering a different, shareable pin */
  ESP_LOGI(TAG_EX, "Component B registering GPIO 19 (Shareable)...");
  ret = pstar_register_pin(GPIO_NUM_19, "Component B - I2C SDA", true);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed initial registration for Comp B: %s", esp_err_to_name(ret));
    goto example_fail;
  }

  /* Simulate Component C registering the SAME shareable pin as B */
  ESP_LOGI(TAG_EX, "Component C registering GPIO 19 (Shareable)...");
  ret = pstar_register_pin(GPIO_NUM_19, "Component C - Status LED (Shared I2C Pin)", true);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed registration for Comp C (shared): %s", esp_err_to_name(ret));
    /* This might fail if Component B didn't register as shareable correctly, treat as error */
    goto example_fail;
  }
  ESP_LOGI(TAG_EX, "GPIO 19 registration by Comp C succeeded (as expected for shared).");

  /* Simulate Component D *trying* to register the *same non-shareable* pin as A */
  ESP_LOGI(TAG_EX, "Component D attempting to register GPIO 18 (Non-Shareable)...");
  ret = pstar_register_pin(GPIO_NUM_18, "Component D - SPI CS", false);
  if (ret == ESP_OK) {
    ESP_LOGE(TAG_EX, "Error: Registration for Comp D succeeded unexpectedly!");
    /* This indicates a logic error in the validator or the example itself */
    goto example_fail;
  } else if (ret == ESP_ERR_INVALID_STATE) {
    ESP_LOGI(
      TAG_EX,
      "Registration for Comp D failed as expected (ESP_ERR_INVALID_STATE) because pin 18 is non-shareable.");
  } else {
    ESP_LOGE(TAG_EX,
             "Registration for Comp D failed with unexpected error: %s",
             esp_err_to_name(ret));
    goto example_fail;
  }

  /* Simulate Component E *trying* to register the *same non-shareable* pin as A, but *claiming* it's shareable */
  ESP_LOGI(TAG_EX, "Component E attempting to register GPIO 18 (Claiming Shareable)...");
  ret = pstar_register_pin(GPIO_NUM_18,
                           "Component E - Debug Pin (Shared!)",
                           true); /* Claiming shareable */
  if (ret == ESP_OK) {
    ESP_LOGE(TAG_EX, "Error: Registration for Comp E succeeded unexpectedly!");
    goto example_fail;
  } else if (ret == ESP_ERR_INVALID_STATE) {
    ESP_LOGI(
      TAG_EX,
      "Registration for Comp E failed as expected (ESP_ERR_INVALID_STATE) because pin 18 was first registered as non-shareable.");
  } else {
    ESP_LOGE(TAG_EX,
             "Registration for Comp E failed with unexpected error: %s",
             esp_err_to_name(ret));
    goto example_fail;
  }

  /* Validate pins - This SHOULD fail because of the conflict attempts on GPIO 18 */
  ESP_LOGI(TAG_EX, "Running pstar_validate_pins() after conflict attempts...");
  ret = pstar_validate_pins();
  if (ret == ESP_OK) {
    /* While individual registrations might have failed preventing a *stored* conflict, */
    /* it's still good practice to run validate. If it passes here, it means */
    /* the failed registrations correctly prevented the conflict state. */
    ESP_LOGW(TAG_EX,
             "pstar_validate_pins() passed. This is OK as conflicting registrations failed.");
  } else if (ret == ESP_ERR_INVALID_STATE) {
    ESP_LOGW(
      TAG_EX,
      "pstar_validate_pins() failed as expected (ESP_ERR_INVALID_STATE). Conflicts were likely stored despite registration failures (potential validator bug?).");
    /* This might indicate an issue if the validator stored conflicting users even if registration returned an error. */
  } else {
    ESP_LOGE(TAG_EX,
             "pstar_validate_pins() failed with unexpected error: %s",
             esp_err_to_name(ret));
    goto example_fail;
  }

  /* Free the validator to reset for the next phase */
  ESP_LOGI(TAG_EX, "Freeing validator state (Phase 1)...");
  ret = pstar_free_pin_validator();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to free validator (Phase 1): %s", esp_err_to_name(ret));
    /* Continue if possible, but log error */
  }

  /* --- Phase 2: Demonstrate Successful Validation --- */
  ESP_LOGI(TAG_EX, "\n*** Phase 2: Demonstrating Successful Validation ***");

  /* Register non-conflicting pins */
  ESP_LOGI(TAG_EX, "Registering GPIO 21 (Comp X, Non-Shareable)...");
  ret = pstar_register_pin(GPIO_NUM_21, "Component X - SPI MOSI", false);
  ESP_ERROR_CHECK(ret); /* Use CHECK for simplicity in this phase */

  ESP_LOGI(TAG_EX, "Registering GPIO 22 (Comp Y, Shareable)...");
  ret = pstar_register_pin(GPIO_NUM_22, "Component Y - I2C SCL", true);
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG_EX, "Registering GPIO 22 (Comp Z, Shareable)...");
  ret = pstar_register_pin(GPIO_NUM_22, "Component Z - Button Input (Shared SCL!)", true);
  ESP_ERROR_CHECK(ret);

  /* Validate pins - This SHOULD succeed */
  ESP_LOGI(TAG_EX, "Running pstar_validate_pins() after non-conflicting registrations...");
  ret = pstar_validate_pins();
  if (ret == ESP_OK) {
    ESP_LOGI(TAG_EX, "pstar_validate_pins() passed successfully as expected.");
  } else {
    ESP_LOGE(TAG_EX, "pstar_validate_pins() failed unexpectedly: %s", esp_err_to_name(ret));
    goto example_fail;
  }

  /* Free the validator again */
  ESP_LOGI(TAG_EX, "Freeing validator state (Phase 2)...");
  ret = pstar_free_pin_validator();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to free validator (Phase 2): %s", esp_err_to_name(ret));
  }

  ESP_LOGI(TAG_EX, "*** Pin Validator Example Completed Successfully ***");
  goto example_halt;

example_fail:
  ESP_LOGE(TAG_EX, "Example encountered an unexpected error. Cleaning up...");
  /* Attempt to free validator resources even on failure */
  pstar_free_pin_validator(); /* Ignore return value here */

example_halt:
  ESP_LOGI(TAG_EX, "Example finished or halted. Looping forever...");
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}