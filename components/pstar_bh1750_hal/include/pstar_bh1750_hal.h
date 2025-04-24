/* components/pstar_bh1750_hal/include/pstar_bh1750_hal.h */

#ifndef PSTAR_COMPONENT_BH1750_HAL_H
#define PSTAR_COMPONENT_BH1750_HAL_H

#include "pstar_bus_manager.h"

#include "driver/i2c.h" /* Added for i2c_port_t definition */

#include <stdbool.h> /* Added for bool */
#include <stdint.h>

#include "esp_err.h"
#include "sdkconfig.h" /* Needed for Kconfig macros */

#ifdef __cplusplus
extern "C" {
#endif

/* --- Configuration --- */

/**
 * @brief Configuration structure for the BH1750 HAL.
 */
typedef struct {
  const char* bus_name; /* Name of the pre-configured/initialized I2C bus in the manager */
                        /* The I2C address is part of the bus configuration */
} bh1750_hal_config_t;

/* --- Opaque Handle --- */
typedef struct bh1750_hal_dev_t* bh1750_hal_handle_t;

/* --- BH1750 Measurement Modes --- */
typedef enum {
  /* Continuous Measurement Modes */
  BH1750_MODE_CONTINUOUS_HIGH_RES  = 0x10, /* 1 lux resolution, 120ms typical measurement time */
  BH1750_MODE_CONTINUOUS_HIGH_RES2 = 0x11, /* 0.5 lux resolution, 120ms typical measurement time */
  BH1750_MODE_CONTINUOUS_LOW_RES   = 0x13, /* 4 lux resolution, 16ms typical measurement time */
  /* One-Time Measurement Modes (sensor powers down after measurement) */
  BH1750_MODE_ONE_TIME_HIGH_RES  = 0x20, /* 1 lux resolution, 120ms */
  BH1750_MODE_ONE_TIME_HIGH_RES2 = 0x21, /* 0.5 lux resolution, 120ms */
  BH1750_MODE_ONE_TIME_LOW_RES   = 0x23, /* 4 lux resolution, 16ms */
} bh1750_hal_mode_t;

/* --- Public Functions --- */

/**
 * @brief Initialize the BH1750 sensor using a bus managed by pstar_bus_manager.
 *
 * Powers on the sensor and sets an initial measurement mode.
 *
 * @param[in] manager Pointer to the initialized bus manager. Must not be NULL.
 * @param[in] config Configuration specifying the bus name. Must not be NULL.
 * @param[in] initial_mode The initial measurement mode to set.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, errors otherwise (e.g., bus not found, I2C comm error).
 */
esp_err_t pstar_bh1750_hal_init(const pstar_bus_manager_t* manager,
                                const bh1750_hal_config_t* config,
                                bh1750_hal_mode_t          initial_mode,
                                bh1750_hal_handle_t*       out_handle);

/**
 * @brief Deinitialize the BH1750 HAL (frees handle).
 *        Does NOT deinitialize the underlying I2C bus managed by pstar_bus_manager.
 *        Optionally attempts to power down the sensor.
 *
 * @param[in] handle Handle obtained from pstar_bh1750_hal_init.
 * @param[in] power_down If true, attempts to send the power down command before freeing handle.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL. Power down command error is logged but doesn't prevent deinit.
 */
esp_err_t pstar_bh1750_hal_deinit(bh1750_hal_handle_t handle, bool power_down);

/**
 * @brief Set the measurement mode of the BH1750.
 *
 * @param[in] handle Handle obtained from pstar_bh1750_hal_init.
 * @param[in] mode The desired measurement mode.
 * @return esp_err_t ESP_OK on success, errors from bus communication otherwise.
 */
esp_err_t pstar_bh1750_hal_set_mode(bh1750_hal_handle_t handle, bh1750_hal_mode_t mode);

/**
 * @brief Read the current light level from the BH1750 in Lux.
 *
 * Waits for the measurement time associated with the current mode before reading.
 * For one-time modes, the sensor needs to be re-awakened (e.g., by setting the mode again)
 * before the next reading. This function assumes continuous mode or that the sensor
 * is ready for reading.
 *
 * @param[in] handle Handle obtained from pstar_bh1750_hal_init.
 * @param[out] lux Pointer to store the calculated light level in Lux. Must not be NULL.
 * @return esp_err_t ESP_OK on success, errors from bus communication otherwise.
 */
esp_err_t pstar_bh1750_hal_read_lux(bh1750_hal_handle_t handle, float* lux);

/**
 * @brief Send the Power Down command to the BH1750.
 *        Measurement stops, reducing power consumption.
 *
 * @param[in] handle Handle obtained from pstar_bh1750_hal_init.
 * @return esp_err_t ESP_OK on success, errors from bus communication otherwise.
 */
esp_err_t pstar_bh1750_hal_power_down(bh1750_hal_handle_t handle);

/**
 * @brief Send the Power On command to the BH1750.
 *        Needed after power down before setting a mode or reading.
 *
 * @param[in] handle Handle obtained from pstar_bh1750_hal_init.
 * @return esp_err_t ESP_OK on success, errors from bus communication otherwise.
 */
esp_err_t pstar_bh1750_hal_power_on(bh1750_hal_handle_t handle);

/**
 * @brief Initialize the BH1750 sensor with default configuration from KConfig.
 *
 * This convenience function creates and initializes a BH1750 sensor using the
 * configuration values defined in KConfig. It performs the following steps:
 * 1. Creates an I2C bus configuration
 * 2. Adds the configuration to the bus manager
 * 3. Initializes the bus hardware
 * 4. Initializes the BH1750 HAL with the default mode
 *
 * @note This function will return ESP_ERR_NOT_SUPPORTED if the BH1750 component
 *       is disabled via KConfig (`CONFIG_PSTAR_KCONFIG_BH1750_ENABLED` is false).
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL.
 *                    This manager WILL be modified (a bus is added).
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_SUPPORTED if component disabled,
 *                   or another error code if any step fails.
 */
esp_err_t pstar_bh1750_hal_create_kconfig_default(pstar_bus_manager_t* manager,
                                                  bh1750_hal_handle_t* out_handle);

/**
 * @brief Initialize the BH1750 sensor with custom configuration.
 *
 * This function creates and initializes a BH1750 sensor using custom configuration values.
 * It performs the following steps:
 * 1. Creates an I2C bus configuration with provided parameters
 * 2. Adds the configuration to the bus manager
 * 3. Initializes the bus hardware
 * 4. Initializes the BH1750 HAL with the specified mode
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL.
 * @param[in] bus_name Name for the I2C bus configuration. Must not be NULL.
 * @param[in] port I2C port number to use.
 * @param[in] addr I2C address of the BH1750 sensor.
 * @param[in] sda_pin GPIO pin number for SDA.
 * @param[in] scl_pin GPIO pin number for SCL.
 * @param[in] freq_hz I2C bus frequency in Hz.
 * @param[in] mode Initial measurement mode for the sensor.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, or an error code if any step fails.
 */
esp_err_t pstar_bh1750_hal_create_custom(pstar_bus_manager_t* manager,
                                         const char*          bus_name,
                                         i2c_port_t           port,
                                         uint8_t              addr,
                                         int                  sda_pin,
                                         int                  scl_pin,
                                         uint32_t             freq_hz,
                                         bh1750_hal_mode_t    mode,
                                         bh1750_hal_handle_t* out_handle);

/**
 * @brief Registers the pins used by the BH1750 component with the pin validator using KConfig values.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if both the
 * pin validator component and the BH1750 component are enabled via Kconfig.
 *
 * @return esp_err_t ESP_OK on success (or if validator/component is disabled), or an error code on failure.
 */
esp_err_t pstar_bh1750_register_kconfig_pins(void);

/**
 * @brief Registers custom pins used by the BH1750 component with the pin validator.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if the
 * pin validator component is enabled via Kconfig.
 *
 * @param[in] sda_pin GPIO pin number for SDA to register.
 * @param[in] scl_pin GPIO pin number for SCL to register.
 * @return esp_err_t ESP_OK on success (or if validator is disabled), or an error code on failure.
 */
esp_err_t pstar_bh1750_register_custom_pins(int sda_pin, int scl_pin);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_BH1750_HAL_H */