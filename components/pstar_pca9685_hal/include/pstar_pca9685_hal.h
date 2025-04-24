/* components/pstar_pca9685_hal/include/pstar_pca9685_hal.h */

#ifndef PSTAR_COMPONENT_PCA9685_HAL_H
#define PSTAR_COMPONENT_PCA9685_HAL_H

#include "pstar_bus_manager.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h" /* Added for SemaphoreHandle_t */
#include "freertos/semphr.h"   /* Added for SemaphoreHandle_t */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- Constants --- */
#define PCA9685_NUM_CHANNELS 16    /**< Number of PWM channels per PCA9685 chip */
#define PCA9685_MAX_PWM_VALUE 4095 /**< Maximum value for PWM ON/OFF registers (12-bit) */

/* --- Configuration --- */

/**
 * @brief Configuration structure for the PCA9685 HAL.
 */
typedef struct {
  const char* bus_name; /**< Name of the pre-configured/initialized I2C bus in the manager */
} pstar_pca9685_hal_config_t;

/* --- Opaque Handle --- */
typedef struct pstar_pca9685_hal_dev_t* pstar_pca9685_hal_handle_t;

/* --- Public Functions --- */

/**
 * @brief Initialize the PCA9685 driver using a bus managed by pstar_bus_manager.
 *
 * Configures the PCA9685 chip with default settings (e.g., enables auto-increment),
 * sets an initial PWM frequency, and configures the Output Enable (OE) pin if enabled
 * via Kconfig for the default instance. Creates a mutex for thread-safe access to the handle.
 *
 * @param[in] manager Pointer to the initialized bus manager. Must not be NULL.
 * @param[in] config Configuration specifying the bus name. Must not be NULL.
 * @param[in] initial_freq_hz The initial PWM frequency to set (e.g., 50 for servos).
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, errors otherwise (e.g., bus not found, I2C comm error, GPIO config error, mutex creation error).
 */
esp_err_t pstar_pca9685_hal_init(const pstar_bus_manager_t*        manager,
                                 const pstar_pca9685_hal_config_t* config,
                                 float                             initial_freq_hz,
                                 pstar_pca9685_hal_handle_t*       out_handle);

/**
 * @brief Deinitialize the PCA9685 HAL.
 *
 * Frees the handle resources, including the mutex. Does NOT deinitialize the underlying I2C bus
 * managed by pstar_bus_manager. Optionally puts the chip into sleep mode and
 * disables the output via the OE pin (if configured).
 *
 * @param[in] handle Handle obtained from pstar_pca9685_hal_init.
 * @param[in] sleep If true, attempts to put the chip into sleep mode before freeing handle.
 * @param[in] disable_output If true, attempts to disable the output via the OE pin (if configured).
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL. Sleep/disable command errors are logged but don't prevent deinit.
 */
esp_err_t
pstar_pca9685_hal_deinit(pstar_pca9685_hal_handle_t handle, bool sleep, bool disable_output);

/**
 * @brief Set the PWM frequency for all channels.
 *
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_pca9685_hal_init.
 * @param[in] freq_hz The desired frequency in Hz (typically 24Hz to 1526Hz).
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_pca9685_hal_set_pwm_freq(pstar_pca9685_hal_handle_t handle, float freq_hz);

/**
 * @brief Set the raw ON and OFF times for a specific PWM channel.
 *
 * Allows precise control over the PWM pulse timing. This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_pca9685_hal_init.
 * @param[in] channel The channel number (0-15).
 * @param[in] on_value The time tick (0-4095) when the pulse should turn ON. Use 0x1000 for full ON.
 * @param[in] off_value The time tick (0-4095) when the pulse should turn OFF. Use 0x1000 for full OFF.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if channel invalid, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_pca9685_hal_set_channel_pwm_values(pstar_pca9685_hal_handle_t handle,
                                                   uint8_t                    channel,
                                                   uint16_t                   on_value,
                                                   uint16_t                   off_value);

/**
 * @brief Set the duty cycle for a specific PWM channel using a percentage.
 *
 * This is a convenience function that sets the ON time to 0 and calculates the OFF time
 * based on the desired percentage (0.0 to 100.0). Handles 0% and 100% duty cycles correctly
 * using the full ON/OFF bits. This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_pca9685_hal_init.
 * @param[in] channel The channel number (0-15).
 * @param[in] duty_cycle_percent The desired duty cycle (0.0 to 100.0).
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if channel or percentage invalid, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_pca9685_hal_set_channel_duty_cycle(pstar_pca9685_hal_handle_t handle,
                                                   uint8_t                    channel,
                                                   float                      duty_cycle_percent);

/**
 * @brief Set the PWM values for all channels simultaneously.
 *
 * Uses the ALL_LED registers for efficiency. This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_pca9685_hal_init.
 * @param[in] on_value The time tick (0-4095) when all pulses should turn ON. Use 0x1000 for full ON.
 * @param[in] off_value The time tick (0-4095) when all pulses should turn OFF. Use 0x1000 for full OFF.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_pca9685_hal_set_all_pwm_values(pstar_pca9685_hal_handle_t handle,
                                               uint16_t                   on_value,
                                               uint16_t                   off_value);

/**
 * @brief Set the angle for a specific servo motor channel.
 *
 * Calculates the required PWM OFF value based on the configured servo pulse width
 * range (min/max microseconds), angle range (degrees), and the current PWM frequency.
 * Assumes the servo pulse starts at time 0 (ON value = 0).
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_pca9685_hal_init.
 * @param[in] channel The channel number (0-15) connected to the servo.
 * @param[in] angle_degrees The desired angle in degrees. The value will be clamped
 *                          to the configured servo angle range (e.g., 0-180).
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if channel invalid,
 *                   ESP_ERR_TIMEOUT if mutex times out, errors from calculation or bus communication otherwise.
 */
esp_err_t pstar_pca9685_hal_set_servo_angle(pstar_pca9685_hal_handle_t handle,
                                            uint8_t                    channel,
                                            float                      angle_degrees);

/**
 * @brief Set the angle for all servo motor channels simultaneously.
 *
 * Calculates the required PWM OFF value based on the configured servo pulse width
 * range, angle range, and the current PWM frequency, then applies it to all channels
 * using the ALL_LED registers. Assumes all channels are connected to identical servos
 * and should be set to the same angle.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_pca9685_hal_init.
 * @param[in] angle_degrees The desired angle in degrees for all channels. The value will be clamped.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from calculation or bus communication otherwise.
 */
esp_err_t pstar_pca9685_hal_set_all_servos_angle(pstar_pca9685_hal_handle_t handle,
                                                 float                      angle_degrees);

/**
 * @brief Restart the PCA9685 chip's PWM operation.
 *
 * Required after setting the PWM frequency or waking from sleep to ensure
 * PWM outputs resume correctly. Toggles the RESTART bit in MODE1.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_pca9685_hal_init.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_pca9685_hal_restart(pstar_pca9685_hal_handle_t handle);

/**
 * @brief Put the PCA9685 chip into sleep mode.
 *
 * Sets the SLEEP bit in MODE1 to reduce power consumption. The oscillator is stopped.
 * Note: This does NOT affect the OE pin state. Use `pstar_pca9685_hal_output_disable`
 * to disable outputs via the OE pin.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_pca9685_hal_init.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_pca9685_hal_sleep(pstar_pca9685_hal_handle_t handle);

/**
 * @brief Wake up the PCA9685 chip from sleep mode.
 *
 * Clears the SLEEP bit in MODE1. The oscillator is started.
 * Needs a small delay (typically >500us) before outputs are resumed. Consider calling
 * pstar_pca9685_hal_restart() after wakeup if needed.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_pca9685_hal_init.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_pca9685_hal_wakeup(pstar_pca9685_hal_handle_t handle);

/**
 * @brief Enable the PWM outputs using the Output Enable (OE) pin.
 *
 * Sets the configured OE GPIO pin to the active level (low or high based on Kconfig/custom setup).
 * Does nothing if OE pin control is not configured for this handle.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_pca9685_hal_init.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if OE pin not configured, ESP_ERR_TIMEOUT if mutex times out, or error from gpio_set_level.
 */
esp_err_t pstar_pca9685_hal_output_enable(pstar_pca9685_hal_handle_t handle);

/**
 * @brief Disable the PWM outputs using the Output Enable (OE) pin.
 *
 * Sets the configured OE GPIO pin to the inactive level (high or low based on Kconfig/custom setup).
 * Does nothing if OE pin control is not configured for this handle.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_pca9685_hal_init.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if OE pin not configured, ESP_ERR_TIMEOUT if mutex times out, or error from gpio_set_level.
 */
esp_err_t pstar_pca9685_hal_output_disable(pstar_pca9685_hal_handle_t handle);

/**
 * @brief Initialize the *first* default PCA9685 driver instance based on KConfig.
 *
 * This convenience function creates and initializes *one* PCA9685 driver using the primary
 * configuration values defined in KConfig (I2C settings, PWM freq, OE pin settings).
 * It performs the following steps:
 * 1. Creates an I2C bus configuration using KConfig values.
 * 2. Adds the configuration to the bus manager.
 * 3. Initializes the bus hardware.
 * 4. Initializes the PCA9685 HAL (including OE pin if configured).
 *
 * Use `pstar_pca9685_hal_create_multiple_defaults` to initialize multiple boards automatically.
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL.
 * This manager WILL be modified (a bus config is added).
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, or an error code if any step fails.
 */
esp_err_t pstar_pca9685_hal_create_kconfig_default(pstar_bus_manager_t*        manager,
                                                   pstar_pca9685_hal_handle_t* out_handle);

/**
 * @brief Initialize multiple PCA9685 driver instances based on KConfig defaults.
 *
 * This function initializes multiple PCA9685 boards based on the Kconfig settings:
 * - `PSTAR_KCONFIG_PCA9685_DEFAULT_INIT_COUNT`: Number of boards to initialize.
 * - `PSTAR_KCONFIG_PCA9685_DEFAULT_ADDR_INCREMENT`: Whether to increment I2C address.
 * - Uses the primary Kconfig settings (port, pins, frequencies, starting address, OE pin) for all boards.
 *   NOTE: Assumes all default boards share the same OE pin connection if OE is enabled.
 *
 * It creates unique bus names (e.g., "pca9685_bus_0", "pca9685_bus_1") for each instance.
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL.
 * This manager WILL be modified (bus configs are added).
 * @param[in] num_boards The number of boards to initialize. Should ideally match
 * `PSTAR_KCONFIG_PCA9685_DEFAULT_INIT_COUNT` but allows runtime override.
 * If 0, the function does nothing and returns ESP_OK.
 * @param[out] out_handles An array allocated by the caller to store the created HAL handles.
 * The array must be large enough to hold `num_boards` handles.
 * Handles will be NULL on error for that specific board index.
 * @return esp_err_t ESP_OK if all requested boards initialized successfully. Returns the first
 * error encountered during initialization of any board, but attempts to
 * initialize all `num_boards`. Check individual handles in `out_handles`
 * for NULL in case of partial failure. ESP_ERR_INVALID_ARG if manager,
 * out_handles is NULL, or num_boards exceeds a reasonable limit.
 * ESP_ERR_NOT_SUPPORTED if the PCA9685 component is disabled in Kconfig.
 */
esp_err_t pstar_pca9685_hal_create_multiple_defaults(pstar_bus_manager_t*       manager,
                                                     uint8_t                    num_boards,
                                                     pstar_pca9685_hal_handle_t out_handles[]);

/**
 * @brief Initialize a PCA9685 driver instance with custom configuration.
 *
 * This function creates and initializes a PCA9685 driver using custom configuration values,
 * including optional Output Enable (OE) pin control.
 * It performs the following steps:
 * 1. Creates an I2C bus configuration with provided parameters.
 * 2. Adds the configuration to the bus manager.
 * 3. Initializes the bus hardware.
 * 4. Initializes the PCA9685 HAL with the specified initial PWM frequency and OE pin settings.
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL.
 * @param[in] bus_name Name for the I2C bus configuration (e.g., "PCA9685_Servo"). Must not be NULL.
 * @param[in] port I2C port number to use.
 * @param[in] addr I2C address of the PCA9685 chip.
 * @param[in] sda_pin GPIO pin number for SDA.
 * @param[in] scl_pin GPIO pin number for SCL.
 * @param[in] i2c_freq_hz I2C bus frequency in Hz.
 * @param[in] initial_pwm_freq_hz Initial PWM frequency for the chip.
 * @param[in] oe_pin GPIO pin number for the Output Enable pin. Set to -1 or GPIO_NUM_NC if not used.
 * @param[in] oe_active_low True if OE pin is active low (0=enabled), false if active high (1=enabled). Ignored if oe_pin is < 0.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, or an error code if any step fails.
 */
esp_err_t pstar_pca9685_hal_create_custom(pstar_bus_manager_t*        manager,
                                          const char*                 bus_name,
                                          i2c_port_t                  port,
                                          uint8_t                     addr,
                                          int                         sda_pin,
                                          int                         scl_pin,
                                          uint32_t                    i2c_freq_hz,
                                          float                       initial_pwm_freq_hz,
                                          gpio_num_t                  oe_pin,
                                          bool                        oe_active_low,
                                          pstar_pca9685_hal_handle_t* out_handle);

/**
 * @brief Registers the I2C and OE pins used by the *first* default PCA9685 component with the pin validator using KConfig values.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if the
 * pin validator component and the PCA9685 component are enabled via Kconfig.
 * It registers the I2C pins and the OE pin (if enabled and valid in Kconfig)
 * defined for the primary default instance. Assumes cascaded default boards share
 * the same I2C and OE pins.
 *
 * @return esp_err_t ESP_OK on success (or if validator/component is disabled), or an error code on failure.
 */
esp_err_t pstar_pca9685_register_kconfig_pins(void);

/**
 * @brief Registers custom I2C and OE pins used by a PCA9685 component instance with the pin validator.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if the
 * pin validator component is enabled via Kconfig. Use this when initializing
 * boards with `pstar_pca9685_hal_create_custom`.
 *
 * @param[in] sda_pin GPIO pin number for SDA to register.
 * @param[in] scl_pin GPIO pin number for SCL to register.
 * @param[in] oe_pin GPIO pin number for OE to register. Set to -1 or GPIO_NUM_NC if not used.
 * @return esp_err_t ESP_OK on success (or if validator is disabled), or an error code on failure.
 */
esp_err_t pstar_pca9685_register_custom_pins(int sda_pin, int scl_pin, int oe_pin);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_PCA9685_HAL_H */