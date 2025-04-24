/* components/pstar_mq135_hal/include/pstar_mq135_hal.h */

#ifndef PSTAR_COMPONENT_MQ135_HAL_H
#define PSTAR_COMPONENT_MQ135_HAL_H

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- Configuration --- */

/**
 * @brief Configuration structure for the MQ135 HAL.
 */
typedef struct {
  adc_unit_t     adc_unit;    /**< ADC unit (ADC_UNIT_1 or ADC_UNIT_2). */
  adc_channel_t  adc_channel; /**< ADC channel connected to the MQ135 output. */
  adc_atten_t    attenuation; /**< ADC attenuation level. */
  adc_bitwidth_t bitwidth;    /**< ADC resolution (bit width). */
  int            num_samples; /**< Number of samples to average for reading. */
} pstar_mq135_hal_config_t;

/* --- Opaque Handle --- */
typedef struct pstar_mq135_hal_dev_t* pstar_mq135_hal_handle_t;

/* --- Public Functions --- */

/**
 * @brief Initialize the MQ135 sensor HAL.
 *
 * Configures the specified ADC channel and unit for reading the sensor's analog output.
 * Attempts to initialize ADC calibration for voltage conversion.
 *
 * @param[in] config Configuration specifying the ADC settings. Must not be NULL.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, errors otherwise (e.g., invalid ADC config, memory allocation failure, calibration failure).
 */
esp_err_t pstar_mq135_hal_init(const pstar_mq135_hal_config_t* config,
                               pstar_mq135_hal_handle_t*       out_handle);

/**
 * @brief Deinitialize the MQ135 HAL.
 *
 * Frees the handle resources, including ADC unit handle and calibration handle.
 *
 * @param[in] handle Handle obtained from pstar_mq135_hal_init.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
esp_err_t pstar_mq135_hal_deinit(pstar_mq135_hal_handle_t handle);

/**
 * @brief Read the raw ADC value from the MQ135 sensor.
 *
 * Reads the ADC channel and returns the raw digital value. If multiple samples
 * were configured during init, this value represents the average.
 *
 * @param[in] handle Handle obtained from pstar_mq135_hal_init.
 * @param[out] raw_value Pointer to store the raw ADC reading. Must not be NULL.
 * @return esp_err_t ESP_OK on success,
 *                   ESP_ERR_INVALID_ARG if handle or output pointer is NULL,
 *                   or errors from ADC read functions.
 */
esp_err_t pstar_mq135_hal_read_raw(pstar_mq135_hal_handle_t handle, int* raw_value);

/**
 * @brief Read the voltage from the MQ135 sensor output.
 *
 * Reads the ADC channel, applies calibration (if available), and returns the
 * calculated voltage in millivolts (mV). If multiple samples were configured
 * during init, this value represents the average voltage.
 *
 * @note ADC calibration must have succeeded during initialization for this
 *       function to return accurate voltage values. If calibration failed,
 *       this function will return ESP_ERR_INVALID_STATE.
 *
 * @param[in] handle Handle obtained from pstar_mq135_hal_init.
 * @param[out] voltage_mv Pointer to store the voltage reading in millivolts. Must not be NULL.
 * @return esp_err_t ESP_OK on success,
 *                   ESP_ERR_INVALID_ARG if handle or output pointer is NULL,
 *                   ESP_ERR_INVALID_STATE if ADC calibration is not available,
 *                   or errors from ADC read/conversion functions.
 */
esp_err_t pstar_mq135_hal_read_voltage(pstar_mq135_hal_handle_t handle, int* voltage_mv);

/**
 * @brief Initialize the MQ135 sensor with default configuration from KConfig.
 *
 * This convenience function creates and initializes an MQ135 sensor using the
 * configuration values defined in KConfig.
 *
 * @note This function will return ESP_ERR_NOT_SUPPORTED if the MQ135 component
 *       is disabled via KConfig (`CONFIG_PSTAR_KCONFIG_MQ135_ENABLED` is false).
 *
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_SUPPORTED if component disabled,
 *                   or another error code if initialization fails.
 */
esp_err_t pstar_mq135_hal_create_kconfig_default(pstar_mq135_hal_handle_t* out_handle);

/**
 * @brief Initialize the MQ135 sensor with custom configuration.
 *
 * This function creates and initializes an MQ135 sensor using custom ADC settings.
 *
 * @param[in] adc_unit ADC unit (ADC_UNIT_1 or ADC_UNIT_2).
 * @param[in] adc_channel ADC channel connected to the MQ135 output.
 * @param[in] attenuation ADC attenuation level.
 * @param[in] bitwidth ADC resolution (bit width).
 * @param[in] num_samples Number of samples to average for reading (e.g., 64).
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, or an error code if initialization fails.
 */
esp_err_t pstar_mq135_hal_create_custom(adc_unit_t                adc_unit,
                                        adc_channel_t             adc_channel,
                                        adc_atten_t               attenuation,
                                        adc_bitwidth_t            bitwidth,
                                        int                       num_samples,
                                        pstar_mq135_hal_handle_t* out_handle);

/**
 * @brief Registers the pin used by the MQ135 component with the pin validator using KConfig values.
 *
 * This function determines the GPIO pin associated with the configured ADC channel
 * and registers it. It should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if both the
 * pin validator component and the MQ135 component are enabled via Kconfig.
 *
 * @return esp_err_t ESP_OK on success (or if validator/component is disabled), or an error code on failure.
 */
esp_err_t pstar_mq135_register_kconfig_pins(void);

/**
 * @brief Registers a custom pin used by the MQ135 component with the pin validator.
 *
 * This function determines the GPIO pin associated with the specified ADC channel
 * and registers it. It should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if the
 * pin validator component is enabled via Kconfig.
 *
 * @param[in] adc_unit ADC unit (ADC_UNIT_1 or ADC_UNIT_2).
 * @param[in] adc_channel ADC channel whose corresponding GPIO pin should be registered.
 * @return esp_err_t ESP_OK on success (or if validator is disabled), or an error code on failure.
 */
esp_err_t pstar_mq135_register_custom_pin(adc_unit_t adc_unit, adc_channel_t adc_channel);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_MQ135_HAL_H */