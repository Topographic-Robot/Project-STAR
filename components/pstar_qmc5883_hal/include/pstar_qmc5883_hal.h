/* components/pstar_qmc5883_hal/include/pstar_qmc5883_hal.h */

#ifndef PSTAR_COMPONENT_QMC5883_HAL_H
#define PSTAR_COMPONENT_QMC5883_HAL_H

#include "pstar_bus_manager.h"

#include "driver/i2c.h"        /* For i2c_port_t */
#include "freertos/FreeRTOS.h" /* For SemaphoreHandle_t */
#include "freertos/semphr.h"   /* For SemaphoreHandle_t */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "sdkconfig.h" /* Needed for Kconfig macros */

#ifdef __cplusplus
extern "C" {
#endif

/* --- Constants --- */
#define QMC5883_I2C_ADDR 0x0D /**< Default (and typically only) I2C address */

/* --- Enums --- */

/** @brief Output Data Rate (ODR) options */
typedef enum {
  QMC5883_ODR_10HZ  = 0x00, /**< 10 Hz */
  QMC5883_ODR_50HZ  = 0x01, /**< 50 Hz */
  QMC5883_ODR_100HZ = 0x02, /**< 100 Hz */
  QMC5883_ODR_200HZ = 0x03  /**< 200 Hz */
} qmc5883_odr_t;

/** @brief Full Scale Range (FSR) options */
typedef enum {
  QMC5883_FSR_2G = 0x00, /**< +/- 2 Gauss */
  QMC5883_FSR_8G = 0x01  /**< +/- 8 Gauss */
} qmc5883_fsr_t;

/** @brief Over Sampling Rate (OSR) options */
typedef enum {
  QMC5883_OSR_512 = 0x00, /**< 512 samples */
  QMC5883_OSR_256 = 0x01, /**< 256 samples */
  QMC5883_OSR_128 = 0x02, /**< 128 samples */
  QMC5883_OSR_64  = 0x03  /**< 64 samples */
} qmc5883_osr_t;

/** @brief Operating Mode options */
typedef enum {
  QMC5883_MODE_STANDBY    = 0x00, /**< Standby mode */
  QMC5883_MODE_CONTINUOUS = 0x01  /**< Continuous measurement mode */
} qmc5883_mode_t;

/* --- Data Structures --- */

/** @brief Structure to hold raw magnetometer data (16-bit signed) */
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} qmc5883_raw_data_t;

/** @brief Structure to hold scaled magnetometer data (in Gauss) */
typedef struct {
  float x;
  float y;
  float z;
} qmc5883_data_t;

/* --- Configuration --- */

/**
 * @brief Configuration structure for the QMC5883 HAL.
 */
typedef struct {
  const char*    bus_name; /**< Name of the pre-configured/initialized I2C bus in the manager */
  qmc5883_odr_t  odr;      /**< Initial Output Data Rate */
  qmc5883_fsr_t  fsr;      /**< Initial Full Scale Range */
  qmc5883_osr_t  osr;      /**< Initial Over Sampling Rate */
  qmc5883_mode_t mode;     /**< Initial Operating Mode */
} pstar_qmc5883_hal_config_t;

/* --- Opaque Handle --- */
typedef struct pstar_qmc5883_hal_dev_t* pstar_qmc5883_hal_handle_t;

/* --- Public Functions --- */

/**
 * @brief Initialize the QMC5883 sensor using a bus managed by pstar_bus_manager.
 *
 * Performs the following:
 * - Checks if the device responds on the expected address (soft check).
 * - Resets the sensor (optional, via Control Register 2).
 * - Sets the initial ODR, FSR, OSR, and Mode via Control Register 1.
 * - Creates a mutex for thread-safe access.
 *
 * @param[in] manager Pointer to the initialized bus manager. Must not be NULL.
 * @param[in] config Configuration specifying the bus name and initial sensor settings. Must not be NULL.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, errors otherwise (e.g., bus not found, I2C comm error, mutex creation error).
 */
esp_err_t pstar_qmc5883_hal_init(const pstar_bus_manager_t*        manager,
                                 const pstar_qmc5883_hal_config_t* config,
                                 pstar_qmc5883_hal_handle_t*       out_handle);

/**
 * @brief Deinitialize the QMC5883 HAL.
 *
 * Frees the handle resources, including the mutex. Does NOT deinitialize the underlying I2C bus
 * managed by pstar_bus_manager. Optionally puts the chip into standby mode.
 *
 * @param[in] handle Handle obtained from pstar_qmc5883_hal_init.
 * @param[in] standby If true, attempts to put the chip into standby mode before freeing handle.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL. Standby command error is logged but doesn't prevent deinit.
 */
esp_err_t pstar_qmc5883_hal_deinit(pstar_qmc5883_hal_handle_t handle, bool standby);

/**
 * @brief Set the operating mode (Standby or Continuous).
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_qmc5883_hal_init.
 * @param[in] mode The desired operating mode.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_qmc5883_hal_set_mode(pstar_qmc5883_hal_handle_t handle, qmc5883_mode_t mode);

/**
 * @brief Set the Output Data Rate (ODR).
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_qmc5883_hal_init.
 * @param[in] odr The desired output data rate.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_qmc5883_hal_set_odr(pstar_qmc5883_hal_handle_t handle, qmc5883_odr_t odr);

/**
 * @brief Set the Full Scale Range (FSR).
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_qmc5883_hal_init.
 * @param[in] fsr The desired full scale range (+/- 2G or +/- 8G).
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_qmc5883_hal_set_fsr(pstar_qmc5883_hal_handle_t handle, qmc5883_fsr_t fsr);

/**
 * @brief Set the Over Sampling Rate (OSR).
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_qmc5883_hal_init.
 * @param[in] osr The desired over sampling rate.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_qmc5883_hal_set_osr(pstar_qmc5883_hal_handle_t handle, qmc5883_osr_t osr);

/**
 * @brief Check if new data is available by reading the status register.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_qmc5883_hal_init.
 * @param[out] data_ready Pointer to store the data ready status (true if ready, false otherwise). Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_qmc5883_hal_is_data_ready(pstar_qmc5883_hal_handle_t handle, bool* data_ready);

/**
 * @brief Read raw magnetometer data (X, Y, Z axes).
 * Reads 6 bytes starting from the X LSB register.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_qmc5883_hal_init.
 * @param[out] raw_data Pointer to store the raw 16-bit signed integer data. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_qmc5883_hal_read_raw_data(pstar_qmc5883_hal_handle_t handle,
                                          qmc5883_raw_data_t*        raw_data);

/**
 * @brief Read scaled magnetometer data in Gauss.
 * Reads the raw data and converts it based on the currently configured Full Scale Range (FSR).
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_qmc5883_hal_init.
 * @param[out] data Pointer to store the scaled float data (in Gauss). Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication or calculation otherwise.
 */
esp_err_t pstar_qmc5883_hal_read_data(pstar_qmc5883_hal_handle_t handle, qmc5883_data_t* data);

/**
 * @brief Read raw temperature data (optional, may not be accurate).
 * Reads 2 bytes from the temperature registers.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_qmc5883_hal_init.
 * @param[out] raw_temp Pointer to store the raw 16-bit signed integer temperature data. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_qmc5883_hal_read_raw_temp(pstar_qmc5883_hal_handle_t handle, int16_t* raw_temp);

/**
 * @brief Perform a software reset of the sensor.
 * Writes to Control Register 2. The sensor requires re-configuration after reset.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_qmc5883_hal_init.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_qmc5883_hal_reset(pstar_qmc5883_hal_handle_t handle);

/**
 * @brief Initialize the QMC5883 sensor with default configuration from KConfig.
 *
 * This convenience function creates and initializes a QMC5883 sensor using the
 * configuration values defined in KConfig. It performs the following steps:
 * 1. Creates an I2C bus configuration using KConfig values.
 * 2. Adds the configuration to the bus manager.
 * 3. Initializes the bus hardware.
 * 4. Initializes the QMC5883 HAL with the default settings from KConfig.
 *
 * @note This function will return ESP_ERR_NOT_SUPPORTED if the QMC5883 component
 *       is disabled via KConfig (`CONFIG_PSTAR_KCONFIG_QMC5883_ENABLED` is false).
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL.
 *                    This manager WILL be modified (a bus config is added).
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_SUPPORTED if component disabled,
 *                   or another error code if any step fails.
 */
esp_err_t pstar_qmc5883_hal_create_kconfig_default(pstar_bus_manager_t*        manager,
                                                   pstar_qmc5883_hal_handle_t* out_handle);

/**
 * @brief Initialize the QMC5883 sensor with custom configuration.
 *
 * This function creates and initializes a QMC5883 sensor using custom configuration values.
 * It performs the following steps:
 * 1. Creates an I2C bus configuration with provided parameters.
 * 2. Adds the configuration to the bus manager.
 * 3. Initializes the bus hardware.
 * 4. Initializes the QMC5883 HAL with the specified settings.
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL.
 * @param[in] bus_name Name for the I2C bus configuration (e.g., "MAG_I2C"). Must not be NULL.
 * @param[in] port I2C port number to use.
 * @param[in] sda_pin GPIO pin number for SDA.
 * @param[in] scl_pin GPIO pin number for SCL.
 * @param[in] i2c_freq_hz I2C bus frequency in Hz (e.g., 400000).
 * @param[in] odr Initial Output Data Rate.
 * @param[in] fsr Initial Full Scale Range.
 * @param[in] osr Initial Over Sampling Rate.
 * @param[in] mode Initial Operating Mode.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, or an error code if any step fails.
 */
esp_err_t pstar_qmc5883_hal_create_custom(pstar_bus_manager_t*        manager,
                                          const char*                 bus_name,
                                          i2c_port_t                  port,
                                          int                         sda_pin,
                                          int                         scl_pin,
                                          uint32_t                    i2c_freq_hz,
                                          qmc5883_odr_t               odr,
                                          qmc5883_fsr_t               fsr,
                                          qmc5883_osr_t               osr,
                                          qmc5883_mode_t              mode,
                                          pstar_qmc5883_hal_handle_t* out_handle);

/**
 * @brief Registers the I2C pins used by the QMC5883 component with the pin validator using KConfig values.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if both the
 * pin validator component and the QMC5883 component are enabled via Kconfig.
 *
 * @return esp_err_t ESP_OK on success (or if validator/component is disabled), or an error code on failure.
 */
esp_err_t pstar_qmc5883_register_kconfig_pins(void);

/**
 * @brief Registers custom I2C pins used by the QMC5883 component with the pin validator.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if the
 * pin validator component is enabled via Kconfig.
 *
 * @param[in] sda_pin GPIO pin number for SDA to register.
 * @param[in] scl_pin GPIO pin number for SCL to register.
 * @return esp_err_t ESP_OK on success (or if validator is disabled), or an error code on failure.
 */
esp_err_t pstar_qmc5883_register_custom_pins(int sda_pin, int scl_pin);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_QMC5883_HAL_H */