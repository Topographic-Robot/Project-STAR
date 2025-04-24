/* components/pstar_mq135_hal/pstar_mq135_hal.c */

#include "pstar_mq135_hal.h"

#include "pstar_pin_validator.h"

#include "freertos/FreeRTOS.h"

#include <stdlib.h>
#include <string.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

static const char* TAG = "MQ135 HAL";

/* --- Internal Handle Structure --- */
typedef struct pstar_mq135_hal_dev_t {
  adc_oneshot_unit_handle_t adc_handle;     /**< Handle for ADC oneshot unit */
  adc_cali_handle_t         cali_handle;    /**< Handle for ADC calibration */
  adc_channel_t             adc_channel;    /**< ADC channel used */
  adc_atten_t               attenuation;    /**< ADC attenuation */
  adc_bitwidth_t            bitwidth;       /**< ADC bitwidth */
  int                       num_samples;    /**< Number of samples for averaging */
  bool                      calibration_ok; /**< Flag indicating if calibration succeeded */
} pstar_mq135_hal_dev_t;

/* --- Helper Functions --- */

/**
 * @brief Attempts to initialize ADC calibration for a given unit and attenuation.
 * @param unit ADC unit.
 * @param atten ADC attenuation.
 * @param bitwidth ADC bitwidth.
 * @param[out] out_handle Pointer to store the calibration handle.
 * @return True if calibration was successful, false otherwise.
 */
static bool priv_adc_calibration_init(adc_unit_t         unit,
                                      adc_atten_t        atten,
                                      adc_bitwidth_t     bitwidth,
                                      adc_cali_handle_t* out_handle)
{
  adc_cali_handle_t handle     = NULL;
  esp_err_t         ret        = ESP_FAIL;
  bool              calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  ESP_LOGD(TAG, "Attempting calibration: Curve Fitting");
  adc_cali_curve_fitting_config_t cali_config = {
    .unit_id  = unit,
    .atten    = atten,
    .bitwidth = bitwidth,
  };
  ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
  if (ret == ESP_OK) {
    calibrated = true;
  } else {
    ESP_LOGW(TAG, "Curve Fitting calibration failed: %s", esp_err_to_name(ret));
  }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGD(TAG, "Attempting calibration: Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
      .unit_id  = unit,
      .atten    = atten,
      .bitwidth = bitwidth,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
      calibrated = true;
    } else {
      ESP_LOGW(TAG, "Line Fitting calibration failed: %s", esp_err_to_name(ret));
    }
  }
#endif

  *out_handle = handle;
  if (calibrated) {
    ESP_LOGI(TAG, "ADC Calibration initialized successfully.");
  } else {
    ESP_LOGW(TAG, "ADC Calibration failed for unit %d, atten %d.", unit, atten);
  }

  return calibrated;
}

/* --- Public Function Implementations --- */

esp_err_t pstar_mq135_hal_init(const pstar_mq135_hal_config_t* config,
                               pstar_mq135_hal_handle_t*       out_handle)
{
  ESP_RETURN_ON_FALSE(config && out_handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
  *out_handle = NULL;

  /* Allocate memory for the device handle */
  pstar_mq135_hal_handle_t dev = (pstar_mq135_hal_handle_t)malloc(sizeof(pstar_mq135_hal_dev_t));
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Failed to allocate device handle");
  memset(dev, 0, sizeof(pstar_mq135_hal_dev_t)); /* Initialize all fields to 0/NULL/false */

  dev->adc_channel = config->adc_channel;
  dev->attenuation = config->attenuation;
  dev->bitwidth    = config->bitwidth;
  dev->num_samples =
    (config->num_samples > 0) ? config->num_samples : 1; /* Ensure at least 1 sample */

  esp_err_t ret = ESP_OK;

  /* Initialize ADC Unit */
  adc_oneshot_unit_init_cfg_t init_config = {
    .unit_id  = config->adc_unit,
    .ulp_mode = ADC_ULP_MODE_DISABLE, /* ULP mode not typically needed for this sensor */
  };
  ret = adc_oneshot_new_unit(&init_config, &dev->adc_handle);
  ESP_GOTO_ON_ERROR(ret, init_fail, TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(ret));

  /* Configure ADC Channel */
  adc_oneshot_chan_cfg_t channel_config = {
    .bitwidth = config->bitwidth,
    .atten    = config->attenuation,
  };
  ret = adc_oneshot_config_channel(dev->adc_handle, config->adc_channel, &channel_config);
  ESP_GOTO_ON_ERROR(ret,
                    init_fail,
                    TAG,
                    "adc_oneshot_config_channel failed: %s",
                    esp_err_to_name(ret));

  /* Attempt ADC Calibration */
  dev->calibration_ok = priv_adc_calibration_init(config->adc_unit,
                                                  config->attenuation,
                                                  config->bitwidth,
                                                  &dev->cali_handle);
  if (!dev->calibration_ok) {
    ESP_LOGW(
      TAG,
      "ADC calibration failed. Voltage readings will be inaccurate. Raw readings are still available.");
    /* Continue initialization even if calibration fails */
  }

  ESP_LOGI(
    TAG,
    "MQ135 HAL initialized: Unit=%d, Channel=%d, Atten=%d, Width=%d, Samples=%d, Calibrated=%s",
    config->adc_unit,
    config->adc_channel,
    config->attenuation,
    config->bitwidth,
    dev->num_samples,
    dev->calibration_ok ? "Yes" : "No");
  *out_handle = dev;
  return ESP_OK;

init_fail:
  ESP_LOGE(TAG, "MQ135 HAL initialization failed.");
  /* Clean up partially initialized resources */
  if (dev->adc_handle) {
    adc_oneshot_del_unit(dev->adc_handle);
  }
  if (dev->cali_handle) {
    /* Assuming a generic deinit function exists or specific ones per scheme */
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_delete_scheme_curve_fitting(dev->cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_delete_scheme_line_fitting(dev->cali_handle);
#endif
  }
  free(dev);
  *out_handle = NULL;
  return ret;
}

esp_err_t pstar_mq135_hal_deinit(pstar_mq135_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGI(TAG, "Deinitializing MQ135 HAL...");

  esp_err_t ret = ESP_OK;

  if (handle->adc_handle) {
    ret = adc_oneshot_del_unit(handle->adc_handle);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "adc_oneshot_del_unit failed: %s", esp_err_to_name(ret));
      /* Continue cleanup */
    }
  }

  if (handle->cali_handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    esp_err_t cali_ret = adc_cali_delete_scheme_curve_fitting(handle->cali_handle);
    if (cali_ret != ESP_OK) {
      ESP_LOGE(TAG, "adc_cali_delete_scheme_curve_fitting failed: %s", esp_err_to_name(cali_ret));
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    esp_err_t cali_ret = adc_cali_delete_scheme_line_fitting(handle->cali_handle);
    if (cali_ret != ESP_OK) {
      ESP_LOGE(TAG, "adc_cali_delete_scheme_line_fitting failed: %s", esp_err_to_name(cali_ret));
    }
#endif
  }

  free(handle);
  return ret; /* Return the error from adc_oneshot_del_unit if it failed */
}

esp_err_t pstar_mq135_hal_read_raw(pstar_mq135_hal_handle_t handle, int* raw_value)
{
  ESP_RETURN_ON_FALSE(handle && raw_value,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Handle or raw_value pointer is NULL");
  ESP_RETURN_ON_FALSE(handle->adc_handle, ESP_ERR_INVALID_STATE, TAG, "ADC unit not initialized");

  int       adc_raw = 0;
  long long raw_sum = 0; /* Use long long for sum to prevent overflow */
  esp_err_t ret     = ESP_OK;

  for (int i = 0; i < handle->num_samples; ++i) {
    ret = adc_oneshot_read(handle->adc_handle, handle->adc_channel, &adc_raw);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "ADC read failed on sample %d: %s", i + 1, esp_err_to_name(ret));
      *raw_value = -1; /* Indicate error */
      return ret;
    }
    raw_sum += adc_raw;
  }

  *raw_value = (int)(raw_sum / handle->num_samples);
  ESP_LOGD(TAG, "Read Raw ADC: %d (Avg over %d samples)", *raw_value, handle->num_samples);
  return ESP_OK;
}

esp_err_t pstar_mq135_hal_read_voltage(pstar_mq135_hal_handle_t handle, int* voltage_mv)
{
  ESP_RETURN_ON_FALSE(handle && voltage_mv,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Handle or voltage_mv pointer is NULL");
  ESP_RETURN_ON_FALSE(handle->adc_handle, ESP_ERR_INVALID_STATE, TAG, "ADC unit not initialized");
  ESP_RETURN_ON_FALSE(handle->calibration_ok && handle->cali_handle,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "ADC calibration not available or failed");

  int       adc_raw     = 0;
  long long raw_sum     = 0;
  int       voltage     = 0;
  long long voltage_sum = 0;
  esp_err_t ret         = ESP_OK;

  for (int i = 0; i < handle->num_samples; ++i) {
    ret = adc_oneshot_read(handle->adc_handle, handle->adc_channel, &adc_raw);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "ADC read failed on sample %d: %s", i + 1, esp_err_to_name(ret));
      *voltage_mv = -1; /* Indicate error */
      return ret;
    }
    raw_sum += adc_raw;

    /* Convert raw value to voltage using calibration */
    ret = adc_cali_raw_to_voltage(handle->cali_handle, adc_raw, &voltage);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "ADC raw to voltage conversion failed: %s", esp_err_to_name(ret));
      *voltage_mv = -1; /* Indicate error */
      return ret;
    }
    voltage_sum += voltage;
  }

  *voltage_mv = (int)(voltage_sum / handle->num_samples);
  ESP_LOGD(TAG,
           "Read Voltage: %d mV (Avg Raw: %lld / %d)",
           *voltage_mv,
           raw_sum / handle->num_samples,
           handle->num_samples);
  return ESP_OK;
}

/* --- Convenience Creation Functions --- */

esp_err_t pstar_mq135_hal_create_kconfig_default(pstar_mq135_hal_handle_t* out_handle)
{
#if !CONFIG_PSTAR_KCONFIG_MQ135_ENABLED
  ESP_LOGE(TAG, "MQ135 component is disabled in Kconfig.");
  if (out_handle)
    *out_handle = NULL;
  return ESP_ERR_NOT_SUPPORTED;
#else
  ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_INVALID_ARG, TAG, "Output handle pointer is NULL");
  ESP_LOGI(TAG, "Creating MQ135 with KConfig default configuration");

  pstar_mq135_hal_config_t config = {
    .adc_unit    = CONFIG_PSTAR_KCONFIG_MQ135_ADC_UNIT,
    .adc_channel = CONFIG_PSTAR_KCONFIG_MQ135_ADC_CHANNEL,
    .attenuation = CONFIG_PSTAR_KCONFIG_MQ135_ADC_ATTEN,
    .bitwidth    = CONFIG_PSTAR_KCONFIG_MQ135_ADC_BITWIDTH,
    .num_samples = CONFIG_PSTAR_KCONFIG_MQ135_NUM_SAMPLES,
  };

  return pstar_mq135_hal_init(&config, out_handle);
#endif /* CONFIG_PSTAR_KCONFIG_MQ135_ENABLED */
}

esp_err_t pstar_mq135_hal_create_custom(adc_unit_t                adc_unit,
                                        adc_channel_t             adc_channel,
                                        adc_atten_t               attenuation,
                                        adc_bitwidth_t            bitwidth,
                                        int                       num_samples,
                                        pstar_mq135_hal_handle_t* out_handle)
{
  ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_INVALID_ARG, TAG, "Output handle pointer is NULL");
  ESP_LOGI(TAG, "Creating MQ135 with custom configuration");

  pstar_mq135_hal_config_t config = {
    .adc_unit    = adc_unit,
    .adc_channel = adc_channel,
    .attenuation = attenuation,
    .bitwidth    = bitwidth,
    .num_samples = num_samples,
  };

  return pstar_mq135_hal_init(&config, out_handle);
}

/* --- Pin Registration Functions --- */

esp_err_t pstar_mq135_register_kconfig_pins(void)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_MQ135_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering MQ135 pin with validator (from KConfig)...");

  gpio_num_t gpio_num = -1;
  /* Use the correct function for oneshot ADC */
  ret = adc_oneshot_channel_to_io(CONFIG_PSTAR_KCONFIG_MQ135_ADC_UNIT,
                                  CONFIG_PSTAR_KCONFIG_MQ135_ADC_CHANNEL,
                                  &gpio_num);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG,
             "Failed to get GPIO number for ADC Unit %d Channel %d: %s",
             CONFIG_PSTAR_KCONFIG_MQ135_ADC_UNIT,
             CONFIG_PSTAR_KCONFIG_MQ135_ADC_CHANNEL,
             esp_err_to_name(ret));
    return ret;
  }

  if (gpio_num == -1) {
    ESP_LOGE(TAG,
             "Invalid GPIO number obtained for ADC Unit %d Channel %d",
             CONFIG_PSTAR_KCONFIG_MQ135_ADC_UNIT,
             CONFIG_PSTAR_KCONFIG_MQ135_ADC_CHANNEL);
    return ESP_ERR_INVALID_ARG;
  }

  const char* desc = "MQ135 Analog Input";
  /* ADC pins are dedicated, cannot be shared */
  ret = pstar_register_pin(gpio_num, desc, false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register MQ135 pin (%d)", gpio_num);

  ESP_LOGI(TAG, "MQ135 KConfig pin (%d) registered successfully.", gpio_num);
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator or MQ135 disabled, skipping MQ135 KConfig pin registration.");
  return ESP_OK; /* Not an error if validator or component is disabled */
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_MQ135_ENABLED */
}

esp_err_t pstar_mq135_register_custom_pin(adc_unit_t adc_unit, adc_channel_t adc_channel)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG,
           "Registering MQ135 custom pin (ADC Unit %d Chan %d) with validator...",
           adc_unit,
           adc_channel);

  gpio_num_t gpio_num = -1;
  /* Use the correct function for oneshot ADC */
  ret = adc_oneshot_channel_to_io(adc_unit, adc_channel, &gpio_num);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG,
             "Failed to get GPIO number for custom ADC Unit %d Channel %d: %s",
             adc_unit,
             adc_channel,
             esp_err_to_name(ret));
    return ret;
  }

  if (gpio_num == -1) {
    ESP_LOGE(TAG,
             "Invalid GPIO number obtained for custom ADC Unit %d Channel %d",
             adc_unit,
             adc_channel);
    return ESP_ERR_INVALID_ARG;
  }

  const char* desc = "MQ135 Custom Analog Input";
  /* ADC pins are dedicated, cannot be shared */
  ret = pstar_register_pin(gpio_num, desc, false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register MQ135 custom pin (%d)", gpio_num);

  ESP_LOGI(TAG, "MQ135 custom pin (%d) registered successfully.", gpio_num);
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator disabled, skipping MQ135 custom pin registration.");
  return ESP_OK; /* Not an error if validator is disabled */
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED */
}