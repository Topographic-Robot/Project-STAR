/* components/pstar_dht22_hal/include/pstar_dht22_hal.h */

#ifndef PSTAR_COMPONENT_DHT22_HAL_H
#define PSTAR_COMPONENT_DHT22_HAL_H

#include "driver/gpio.h"

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- Configuration --- */

/**
 * @brief Configuration structure for the DHT22 HAL.
 */
typedef struct {
  gpio_num_t gpio_pin; /**< GPIO pin connected to the DHT22 data line. */
} pstar_dht22_hal_config_t;

/* --- Opaque Handle --- */
typedef struct pstar_dht22_hal_dev_t* pstar_dht22_hal_handle_t;

/* --- Public Functions --- */

/**
 * @brief Initialize the DHT22 sensor HAL.
 *
 * Configures the specified GPIO pin for communication with the DHT22.
 *
 * @param[in] config Configuration specifying the GPIO pin. Must not be NULL.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, errors otherwise (e.g., invalid pin, memory allocation failure).
 */
esp_err_t pstar_dht22_hal_init(const pstar_dht22_hal_config_t* config,
                               pstar_dht22_hal_handle_t*       out_handle);

/**
 * @brief Deinitialize the DHT22 HAL.
 *
 * Frees the handle resources and resets the GPIO pin configuration.
 *
 * @param[in] handle Handle obtained from pstar_dht22_hal_init.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
esp_err_t pstar_dht22_hal_deinit(pstar_dht22_hal_handle_t handle);

/**
 * @brief Read temperature and humidity data from the DHT22 sensor.
 *
 * Initiates communication with the sensor, reads the 40-bit data stream,
 * validates the checksum, and returns the temperature and humidity.
 * This function involves timing-critical operations and may block for ~20-30ms.
 *
 * @param[in] handle Handle obtained from pstar_dht22_hal_init.
 * @param[out] humidity Pointer to store the relative humidity reading (in %). Must not be NULL.
 * @param[out] temperature Pointer to store the temperature reading (in Celsius). Must not be NULL.
 * @return esp_err_t ESP_OK on success,
 *                   ESP_ERR_INVALID_ARG if handle or output pointers are NULL,
 *                   ESP_ERR_TIMEOUT if sensor does not respond,
 *                   ESP_ERR_INVALID_CRC if data checksum fails,
 *                   or other errors from GPIO/timing functions.
 */
esp_err_t
pstar_dht22_hal_read_data(pstar_dht22_hal_handle_t handle, float* humidity, float* temperature);

/**
 * @brief Initialize the DHT22 sensor with default configuration from KConfig.
 *
 * This convenience function creates and initializes a DHT22 sensor using the
 * configuration values defined in KConfig.
 *
 * @note This function will return ESP_ERR_NOT_SUPPORTED if the DHT22 component
 *       is disabled via KConfig (`CONFIG_PSTAR_KCONFIG_DHT22_ENABLED` is false).
 *
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_SUPPORTED if component disabled,
 *                   or another error code if initialization fails.
 */
esp_err_t pstar_dht22_hal_create_kconfig_default(pstar_dht22_hal_handle_t* out_handle);

/**
 * @brief Initialize the DHT22 sensor with custom configuration.
 *
 * This function creates and initializes a DHT22 sensor using a custom GPIO pin.
 *
 * @param[in] gpio_pin GPIO pin number for the DHT22 data line.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, or an error code if initialization fails.
 */
esp_err_t pstar_dht22_hal_create_custom(gpio_num_t gpio_pin, pstar_dht22_hal_handle_t* out_handle);

/**
 * @brief Registers the pin used by the DHT22 component with the pin validator using KConfig values.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if both the
 * pin validator component and the DHT22 component are enabled via Kconfig.
 *
 * @return esp_err_t ESP_OK on success (or if validator/component is disabled), or an error code on failure.
 */
esp_err_t pstar_dht22_register_kconfig_pins(void);

/**
 * @brief Registers a custom pin used by the DHT22 component with the pin validator.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if the
 * pin validator component is enabled via Kconfig.
 *
 * @param[in] gpio_pin GPIO pin number for the DHT22 data line to register.
 * @return esp_err_t ESP_OK on success (or if validator is disabled), or an error code on failure.
 */
esp_err_t pstar_dht22_register_custom_pin(int gpio_pin);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_DHT22_HAL_H */