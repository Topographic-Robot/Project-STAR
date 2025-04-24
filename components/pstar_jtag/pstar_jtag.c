/* components/pstar_jtag/pstar_jtag.c */

#include "pstar_jtag.h"

#include "pstar_pin_validator.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h" /* Needed for CONFIG_ checks */

static const char* TAG = "PSTAR JTAG";

/* --- Functions --- */

esp_err_t pstar_get_jtag_pins(pstar_jtag_t* tag)
{
  ESP_RETURN_ON_FALSE(tag, ESP_ERR_INVALID_ARG, TAG, "get_jtag_pins: NULL argument");

/* --- Guard Block Start --- */
/* Only compile the following code if JTAG is enabled in Kconfig */
#if CONFIG_PSTAR_KCONFIG_JTAG_ENABLED

  /* Directly use the final Kconfig values determined by the logic in Kconfig.projbuild */
  tag->tck = CONFIG_PSTAR_KCONFIG_JTAG_PIN_TCK;
  tag->tms = CONFIG_PSTAR_KCONFIG_JTAG_PIN_TMS;
  tag->tdi = CONFIG_PSTAR_KCONFIG_JTAG_PIN_TDI;
  tag->tdo = CONFIG_PSTAR_KCONFIG_JTAG_PIN_TDO;

  /* Check if any pin is invalid (-1), which might happen if default mode is selected */
  /* for an unsupported target or if custom pins are somehow invalid. */
  if (tag->tck < 0 || tag->tms < 0 || tag->tdi < 0 || tag->tdo < 0) {
    ESP_LOGE(
      TAG,
      "Invalid JTAG pin configuration detected (Pin < 0). Check Kconfig defaults for target '%s' or custom settings.",
      CONFIG_IDF_TARGET);
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG,
           "JTAG pin config (Target: %s, Mode: %s): TCK=%d, TMS=%d, TDI=%d, TDO=%d",
           CONFIG_IDF_TARGET,
#if CONFIG_PSTAR_KCONFIG_JTAG_USE_DEFAULT
           "Default",
#else
           "Custom",
#endif
           tag->tck,
           tag->tms,
           tag->tdi,
           tag->tdo);

  return ESP_OK;

#else /* CONFIG_PSTAR_KCONFIG_JTAG_ENABLED is false */

  /* JTAG is disabled, return error and set invalid pins */
  ESP_LOGW(TAG, "JTAG is disabled via Kconfig. Cannot get JTAG pins.");
  tag->tck = -1; /* Indicate invalid pins */
  tag->tms = -1;
  tag->tdi = -1;
  tag->tdo = -1;
  return ESP_ERR_NOT_SUPPORTED;

#endif /* CONFIG_PSTAR_KCONFIG_JTAG_ENABLED */
  /* --- Guard Block End --- */
}

esp_err_t pstar_jtag_register_kconfig_pins(void)
{
/* --- Guard Block Start --- */
/* Only compile the following code if BOTH Pin Validator AND JTAG are enabled */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_JTAG_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering JTAG pins with validator (from KConfig)...");

  /* Get JTAG pins configuration */
  pstar_jtag_t jtag_pins;
  ret = pstar_get_jtag_pins(&jtag_pins);
  /* If get_jtag_pins fails (e.g., invalid config even if enabled), return error */
  ESP_RETURN_ON_ERROR(ret,
                      TAG,
                      "Failed to get JTAG pins for registration: %s",
                      esp_err_to_name(ret));

  /* Register all JTAG pins (usually not shared) */
  /* Check if pins are valid before registering */
  if (jtag_pins.tck >= 0) {
    ret = pstar_register_pin(jtag_pins.tck, "JTAG TCK", false);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register JTAG TCK pin (%d)", jtag_pins.tck);
  } else {
    ESP_LOGW(TAG, "Skipping registration of invalid TCK pin");
  }

  if (jtag_pins.tms >= 0) {
    ret = pstar_register_pin(jtag_pins.tms, "JTAG TMS", false);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register JTAG TMS pin (%d)", jtag_pins.tms);
  } else {
    ESP_LOGW(TAG, "Skipping registration of invalid TMS pin");
  }

  if (jtag_pins.tdi >= 0) {
    ret = pstar_register_pin(jtag_pins.tdi, "JTAG TDI", false);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register JTAG TDI pin (%d)", jtag_pins.tdi);
  } else {
    ESP_LOGW(TAG, "Skipping registration of invalid TDI pin");
  }

  if (jtag_pins.tdo >= 0) {
    ret = pstar_register_pin(jtag_pins.tdo, "JTAG TDO", false);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register JTAG TDO pin (%d)", jtag_pins.tdo);
  } else {
    ESP_LOGW(TAG, "Skipping registration of invalid TDO pin");
  }

  ESP_LOGI(TAG, "JTAG KConfig pins registered successfully.");
  return ESP_OK;

#else /* Pin Validator or JTAG is disabled */

  ESP_LOGD(TAG, "Pin validator or JTAG disabled, skipping JTAG pin registration.");
  return ESP_OK; /* Not an error if validator or JTAG is disabled */

#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_JTAG_ENABLED */
  /* --- Guard Block End --- */
}