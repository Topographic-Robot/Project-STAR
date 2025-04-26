/* components/pstar_mpu6050_hal/include/pstar_mpu6050_hal.h */

#ifndef PSTAR_COMPONENT_MPU6050_HAL_H
#define PSTAR_COMPONENT_MPU6050_HAL_H

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
#define MPU6050_I2C_ADDR_LOW 0x68  /**< Default I2C address when AD0 is low */
#define MPU6050_I2C_ADDR_HIGH 0x69 /**< I2C address when AD0 is high */

/* --- Enums --- */

/** @brief Accelerometer full-scale range options */
typedef enum {
  MPU6050_ACCEL_FS_2G  = 0, /**< +/- 2g */
  MPU6050_ACCEL_FS_4G  = 1, /**< +/- 4g */
  MPU6050_ACCEL_FS_8G  = 2, /**< +/- 8g */
  MPU6050_ACCEL_FS_16G = 3  /**< +/- 16g */
} mpu6050_accel_fs_t;

/** @brief Gyroscope full-scale range options */
typedef enum {
  MPU6050_GYRO_FS_250DPS  = 0, /**< +/- 250 degrees/sec */
  MPU6050_GYRO_FS_500DPS  = 1, /**< +/- 500 degrees/sec */
  MPU6050_GYRO_FS_1000DPS = 2, /**< +/- 1000 degrees/sec */
  MPU6050_GYRO_FS_2000DPS = 3  /**< +/- 2000 degrees/sec */
} mpu6050_gyro_fs_t;

/** @brief Digital Low Pass Filter (DLPF) configuration options */
typedef enum {
  MPU6050_DLPF_BW_260 = 0, /**< Accel BW 260Hz, Gyro BW 256Hz */
  MPU6050_DLPF_BW_184 = 1, /**< Accel BW 184Hz, Gyro BW 188Hz */
  MPU6050_DLPF_BW_94  = 2, /**< Accel BW 94Hz, Gyro BW 98Hz */
  MPU6050_DLPF_BW_44  = 3, /**< Accel BW 44Hz, Gyro BW 42Hz */
  MPU6050_DLPF_BW_21  = 4, /**< Accel BW 21Hz, Gyro BW 20Hz */
  MPU6050_DLPF_BW_10  = 5, /**< Accel BW 10Hz, Gyro BW 10Hz */
  MPU6050_DLPF_BW_5   = 6, /**< Accel BW 5Hz, Gyro BW 5Hz */
  /* MPU6050_DLPF_RESERVED = 7 */
} mpu6050_dlpf_bw_t;

/* --- Data Structures --- */

/** @brief Structure to hold raw accelerometer data (16-bit signed) */
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} mpu6050_raw_accel_t;

/** @brief Structure to hold raw gyroscope data (16-bit signed) */
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} mpu6050_raw_gyro_t;

/** @brief Structure to hold scaled accelerometer data (in g) */
typedef struct {
  float x;
  float y;
  float z;
} mpu6050_accel_t;

/** @brief Structure to hold scaled gyroscope data (in degrees/sec) */
typedef struct {
  float x;
  float y;
  float z;
} mpu6050_gyro_t;

/* --- Configuration --- */

/**
 * @brief Configuration structure for the MPU6050 HAL.
 */
typedef struct {
  const char*        bus_name; /**< Name of the pre-configured/initialized I2C bus in the manager */
  mpu6050_accel_fs_t accel_fs_sel; /**< Initial Accelerometer Full Scale Range */
  mpu6050_gyro_fs_t  gyro_fs_sel;  /**< Initial Gyroscope Full Scale Range */
  mpu6050_dlpf_bw_t  dlpf_bw;      /**< Digital Low Pass Filter Bandwidth */
} pstar_mpu6050_hal_config_t;

/* --- Opaque Handle --- */
typedef struct pstar_mpu6050_hal_dev_t* pstar_mpu6050_hal_handle_t;

/* --- Public Functions --- */

/**
 * @brief Initialize the MPU6050 sensor using a bus managed by pstar_bus_manager.
 *
 * Performs the following:
 * - Checks WHO_AM_I register.
 * - Wakes the sensor from sleep mode.
 * - Sets the clock source (PLL with X gyro reference recommended).
 * - Sets the initial accelerometer and gyroscope full-scale ranges.
 * - Configures the Digital Low Pass Filter (DLPF).
 * - Creates a mutex for thread-safe access.
 *
 * @param[in] manager Pointer to the initialized bus manager. Must not be NULL.
 * @param[in] config Configuration specifying the bus name and initial sensor settings. Must not be NULL.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, errors otherwise (e.g., bus not found, I2C comm error, wrong WHO_AM_I, mutex creation error).
 */
esp_err_t pstar_mpu6050_hal_init(const pstar_bus_manager_t*        manager,
                                 const pstar_mpu6050_hal_config_t* config,
                                 pstar_mpu6050_hal_handle_t*       out_handle);

/**
 * @brief Deinitialize the MPU6050 HAL.
 *
 * Frees the handle resources, including the mutex. Does NOT deinitialize the underlying I2C bus
 * managed by pstar_bus_manager. Optionally puts the chip into sleep mode.
 *
 * @param[in] handle Handle obtained from pstar_mpu6050_hal_init.
 * @param[in] sleep If true, attempts to put the chip into sleep mode before freeing handle.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL. Sleep command error is logged but doesn't prevent deinit.
 */
esp_err_t pstar_mpu6050_hal_deinit(pstar_mpu6050_hal_handle_t handle, bool sleep);

/**
 * @brief Read raw accelerometer data.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_mpu6050_hal_init.
 * @param[out] raw_data Pointer to store the raw 16-bit signed integer data. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_mpu6050_hal_read_raw_accel(pstar_mpu6050_hal_handle_t handle,
                                           mpu6050_raw_accel_t*       raw_data);

/**
 * @brief Read raw gyroscope data.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_mpu6050_hal_init.
 * @param[out] raw_data Pointer to store the raw 16-bit signed integer data. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_mpu6050_hal_read_raw_gyro(pstar_mpu6050_hal_handle_t handle,
                                          mpu6050_raw_gyro_t*        raw_data);

/**
 * @brief Read raw temperature data.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_mpu6050_hal_init.
 * @param[out] raw_temp Pointer to store the raw 16-bit signed integer temperature data. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_mpu6050_hal_read_raw_temp(pstar_mpu6050_hal_handle_t handle, int16_t* raw_temp);

/**
 * @brief Read scaled accelerometer data in g (gravity units).
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_mpu6050_hal_init.
 * @param[out] accel_data Pointer to store the scaled float data (in g). Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication or calculation otherwise.
 */
esp_err_t pstar_mpu6050_hal_read_accel(pstar_mpu6050_hal_handle_t handle,
                                       mpu6050_accel_t*           accel_data);

/**
 * @brief Read scaled gyroscope data in degrees per second.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_mpu6050_hal_init.
 * @param[out] gyro_data Pointer to store the scaled float data (in deg/s). Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication or calculation otherwise.
 */
esp_err_t pstar_mpu6050_hal_read_gyro(pstar_mpu6050_hal_handle_t handle, mpu6050_gyro_t* gyro_data);

/**
 * @brief Read scaled temperature data in degrees Celsius.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_mpu6050_hal_init.
 * @param[out] temp_c Pointer to store the scaled float temperature (in deg C). Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication or calculation otherwise.
 */
esp_err_t pstar_mpu6050_hal_read_temp(pstar_mpu6050_hal_handle_t handle, float* temp_c);

/**
 * @brief Set the accelerometer full-scale range.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_mpu6050_hal_init.
 * @param[in] range The desired accelerometer range.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_mpu6050_hal_set_accel_range(pstar_mpu6050_hal_handle_t handle,
                                            mpu6050_accel_fs_t         range);

/**
 * @brief Set the gyroscope full-scale range.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_mpu6050_hal_init.
 * @param[in] range The desired gyroscope range.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_mpu6050_hal_set_gyro_range(pstar_mpu6050_hal_handle_t handle,
                                           mpu6050_gyro_fs_t          range);

/**
 * @brief Set the Digital Low Pass Filter (DLPF) bandwidth.
 * This function is thread-safe.
 *
 * @param[in] handle Handle obtained from pstar_mpu6050_hal_init.
 * @param[in] bandwidth The desired DLPF bandwidth setting.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex times out, errors from bus communication otherwise.
 */
esp_err_t pstar_mpu6050_hal_set_dlpf_bandwidth(pstar_mpu6050_hal_handle_t handle,
                                               mpu6050_dlpf_bw_t          bandwidth);

/**
 * @brief Initialize the MPU6050 sensor with default configuration from KConfig.
 *
 * This convenience function creates and initializes an MPU6050 sensor using the
 * configuration values defined in KConfig. It performs the following steps:
 * 1. Creates an I2C bus configuration using KConfig values.
 * 2. Adds the configuration to the bus manager.
 * 3. Initializes the bus hardware.
 * 4. Initializes the MPU6050 HAL with the default settings from KConfig.
 *
 * @note This function will return ESP_ERR_NOT_SUPPORTED if the MPU6050 component
 *       is disabled via KConfig (`CONFIG_PSTAR_KCONFIG_MPU6050_ENABLED` is false).
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL.
 *                    This manager WILL be modified (a bus config is added).
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_SUPPORTED if component disabled,
 *                   or another error code if any step fails.
 */
esp_err_t pstar_mpu6050_hal_create_kconfig_default(pstar_bus_manager_t*        manager,
                                                   pstar_mpu6050_hal_handle_t* out_handle);

/**
 * @brief Initialize the MPU6050 sensor with custom configuration.
 *
 * This function creates and initializes an MPU6050 sensor using custom configuration values.
 * It performs the following steps:
 * 1. Creates an I2C bus configuration with provided parameters.
 * 2. Adds the configuration to the bus manager.
 * 3. Initializes the bus hardware.
 * 4. Initializes the MPU6050 HAL with the specified settings.
 *
 * @param[in] manager Pointer to an initialized bus manager. Must not be NULL.
 * @param[in] bus_name Name for the I2C bus configuration (e.g., "IMU_I2C"). Must not be NULL.
 * @param[in] port I2C port number to use.
 * @param[in] addr I2C address of the MPU6050 sensor (MPU6050_I2C_ADDR_LOW or MPU6050_I2C_ADDR_HIGH).
 * @param[in] sda_pin GPIO pin number for SDA.
 * @param[in] scl_pin GPIO pin number for SCL.
 * @param[in] i2c_freq_hz I2C bus frequency in Hz (e.g., 400000).
 * @param[in] accel_fs Initial accelerometer full-scale range.
 * @param[in] gyro_fs Initial gyroscope full-scale range.
 * @param[in] dlpf_bw Digital Low Pass Filter bandwidth setting.
 * @param[out] out_handle Pointer to store the created HAL handle. Must not be NULL.
 * @return esp_err_t ESP_OK on success, or an error code if any step fails.
 */
esp_err_t pstar_mpu6050_hal_create_custom(pstar_bus_manager_t*        manager,
                                          const char*                 bus_name,
                                          i2c_port_t                  port,
                                          uint8_t                     addr,
                                          int                         sda_pin,
                                          int                         scl_pin,
                                          uint32_t                    i2c_freq_hz,
                                          mpu6050_accel_fs_t          accel_fs,
                                          mpu6050_gyro_fs_t           gyro_fs,
                                          mpu6050_dlpf_bw_t           dlpf_bw,
                                          pstar_mpu6050_hal_handle_t* out_handle);

/**
 * @brief Registers the I2C pins used by the MPU6050 component with the pin validator using KConfig values.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if both the
 * pin validator component and the MPU6050 component are enabled via Kconfig.
 *
 * @return esp_err_t ESP_OK on success (or if validator/component is disabled), or an error code on failure.
 */
esp_err_t pstar_mpu6050_register_kconfig_pins(void);

/**
 * @brief Registers custom I2C pins used by the MPU6050 component with the pin validator.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if the
 * pin validator component is enabled via Kconfig.
 *
 * @param[in] sda_pin GPIO pin number for SDA to register.
 * @param[in] scl_pin GPIO pin number for SCL to register.
 * @return esp_err_t ESP_OK on success (or if validator is disabled), or an error code on failure.
 */
esp_err_t pstar_mpu6050_register_custom_pins(int sda_pin, int scl_pin);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_MPU6050_HAL_H */