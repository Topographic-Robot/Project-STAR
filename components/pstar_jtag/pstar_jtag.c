/* components/pstar_jtag/pstar_jtag.c */

/* TODO: Add macro header for pstar logger */

#include "pstar_jtag.h"

#include "esp_err.h"
#include "esp_log.h"

#define TAG ("PSTAR JTAG")

/****************************** Functions ******************************/

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

  ESP_LOGI(TAG, "JTAG pin config: TCK=%d, TMS=%d, TDI=%d, TDO=%d", tag->tck, tag->tms, tag->tdi, tag->tdo);

  return ESP_OK;
}
