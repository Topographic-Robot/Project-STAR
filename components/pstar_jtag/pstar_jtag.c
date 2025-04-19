/* components/pstar_jtag/pstar_jtag.c */

#include "pstar_jtag.h"

#include "pstar_pin_validator.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"

static const char* TAG = "PSTAR JTAG";

/* --- Functions --- */

esp_err_t pstar_get_jtag_pins(pstar_jtag_t* tag)
{
  if (tag == NULL) {
    ESP_LOGE(TAG, "get_jtag_pins: NULL argument");
    return ESP_ERR_INVALID_ARG;
  }

#if !CONFIG_PSTAR_KCONFIG_JTAG_ENABLED
  ESP_LOGW(TAG, "JTAG is disabled via Kconfig");
  return ESP_ERR_NOT_SUPPORTED;
#endif

#if CONFIG_PSTAR_KCONFIG_JTAG_USE_CUSTOM
  tag->tck = CONFIG_PSTAR_KCONFIG_JTAG_PIN_TCK;
  tag->tms = CONFIG_PSTAR_KCONFIG_JTAG_PIN_TMS;
  tag->tdi = CONFIG_PSTAR_KCONFIG_JTAG_PIN_TDI;
  tag->tdo = CONFIG_PSTAR_KCONFIG_JTAG_PIN_TDO;
#elif CONFIG_PSTAR_KCONFIG_JTAG_USE_DEFAULT

#if CONFIG_PSTAR_KCONFIG_JTAG_MCU_ESP32
  tag->tck = GPIO_NUM_13;
  tag->tms = GPIO_NUM_14;
  tag->tdi = GPIO_NUM_12;
  tag->tdo = GPIO_NUM_15;
#elif CONFIG_PSTAR_KCONFIG_JTAG_MCU_ESP32S2 || CONFIG_PSTAR_KCONFIG_JTAG_MCU_ESP32S3
  tag->tck = GPIO_NUM_39;
  tag->tms = GPIO_NUM_40;
  tag->tdi = GPIO_NUM_41;
  tag->tdo = GPIO_NUM_42;
#else
  ESP_LOGE(TAG, "No supported MCU selected for default JTAG pins");
  return ESP_ERR_INVALID_STATE;
#endif

#else
  ESP_LOGE(TAG, "Invalid JTAG configuration mode");
  return ESP_ERR_INVALID_STATE;
#endif

  ESP_LOGI(TAG,
           "JTAG pin config: TCK=%d, TMS=%d, TDI=%d, TDO=%d",
           tag->tck,
           tag->tms,
           tag->tdi,
           tag->tdo);

  return ESP_OK;
}

esp_err_t pstar_jtag_register_kconfig_pins(void)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_JTAG_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering JTAG pins with validator (from KConfig)...");

  /* Get JTAG pins configuration */
  pstar_jtag_t jtag_pins;
  ret = pstar_get_jtag_pins(&jtag_pins);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to get JTAG pins: %s", esp_err_to_name(ret));

  /* Register all JTAG pins (usually not shared) */
  ret = pstar_register_pin(jtag_pins.tck, "JTAG TCK", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register JTAG TCK pin (%d)", jtag_pins.tck);

  ret = pstar_register_pin(jtag_pins.tms, "JTAG TMS", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register JTAG TMS pin (%d)", jtag_pins.tms);

  ret = pstar_register_pin(jtag_pins.tdi, "JTAG TDI", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register JTAG TDI pin (%d)", jtag_pins.tdi);

  ret = pstar_register_pin(jtag_pins.tdo, "JTAG TDO", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register JTAG TDO pin (%d)", jtag_pins.tdo);

  ESP_LOGI(TAG, "JTAG KConfig pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator or JTAG disabled, skipping JTAG pin registration.");
  return ESP_OK; /* Not an error if validator or JTAG is disabled */
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_JTAG_ENABLED */
}