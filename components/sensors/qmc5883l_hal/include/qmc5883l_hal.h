/* components/sensors/qmc5883l_hal/include/qmc5883l_hal.h */

#ifndef TOPOROBO_QMC5883L_HAL_H
#define TOPOROBO_QMC5883L_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"

/* Constants ******************************************************************/

extern const uint8_t    qmc5883l_i2c_address;            /**< I2C address for the QMC5883L sensor. */
extern const i2c_port_t qmc5883l_i2c_bus;                /**< I2C bus number used by the ESP32 for QMC5883L communication. */
extern const char      *qmc5883l_tag;                    /**< Tag for ESP_LOG messages related to the QMC5883L sensor. */
extern const uint8_t    qmc5883l_scl_io;                 /**< GPIO pin for I2C Serial Clock Line (SCL) for QMC5883L. */
extern const uint8_t    qmc5883l_sda_io;                 /**< GPIO pin for I2C Serial Data Line (SDA) for QMC5883L. */
extern const uint32_t   qmc5883l_i2c_freq_hz;            /**< I2C bus frequency for QMC5883L communication in Hz. */
extern const uint32_t   qmc5883l_polling_rate_ticks;     /**< Polling rate for QMC5883L sensor reads in system ticks. */
extern const uint8_t    qmc5883l_odr_setting;            /**< Output Data Rate (ODR) setting for the QMC5883L sensor. */
extern const uint8_t    qmc5883l_max_retries;            /**< Maximum retry attempts for QMC5883L reinitialization. */
extern const uint32_t   qmc5883l_initial_retry_interval; /**< Initial retry interval for QMC5883L reinitialization in ticks. */
extern const uint32_t   qmc5883l_max_backoff_interval;   /**< Maximum backoff interval for QMC5883L retries in ticks. */

/* Enums **********************************************************************/

/**
 * @brief Enumeration of I2C commands for the QMC5883L sensor.
 *
 * Defines the possible I2C commands used to configure, control, and read data
 * from the QMC5883L magnetometer sensor. These commands include register addresses
 * for configuration, data output, status checks, and temperature readings.
 */
typedef enum : uint8_t {
  /* Power and Reset Commands */
  k_qmc5883l_reset_cmd       = 0x0B, /**< Command to reset the sensor. */
  k_qmc5883l_set_reset_cmd   = 0x0B, /**< Command to set/reset the sensor period. */

  /* Control and Configuration Commands */
  k_qmc5883l_ctrl1_cmd       = 0x09, /**< Control Register 1: Mode, ODR, Range, OSR. */
  k_qmc5883l_ctrl2_cmd       = 0x0A, /**< Control Register 2: Soft reset and standby mode. */

  /* Data Output Registers */
  k_qmc5883l_data_xout_l_cmd = 0x00, /**< X-axis low byte data. */
  k_qmc5883l_data_xout_h_cmd = 0x01, /**< X-axis high byte data. */
  k_qmc5883l_data_yout_l_cmd = 0x02, /**< Y-axis low byte data. */
  k_qmc5883l_data_yout_h_cmd = 0x03, /**< Y-axis high byte data. */
  k_qmc5883l_data_zout_l_cmd = 0x04, /**< Z-axis low byte data. */
  k_qmc5883l_data_zout_h_cmd = 0x05, /**< Z-axis high byte data. */

  /* Status and Temperature Commands */
  k_qmc5883l_status_cmd      = 0x06, /**< Status register: DRDY, overflow, and data overflow. */
  k_qmc5883l_temp_l_cmd      = 0x07, /**< Temperature low byte. */
  k_qmc5883l_temp_h_cmd      = 0x08, /**< Temperature high byte. */

  /* Control Register 1 Configuration Values */
  k_qmc5883l_mode_standby    = 0x00, /**< Standby mode. */
  k_qmc5883l_mode_continuous = 0x01, /**< Continuous measurement mode. */

  /* Output Data Rate (ODR) Configurations */
  k_qmc5883l_odr_10hz        = 0x00, /**< Output Data Rate: 10 Hz. */
  k_qmc5883l_odr_50hz        = 0x04, /**< Output Data Rate: 50 Hz. */
  k_qmc5883l_odr_100hz       = 0x08, /**< Output Data Rate: 100 Hz. */
  k_qmc5883l_odr_200hz       = 0x0C, /**< Output Data Rate: 200 Hz. */

  /* Full-Scale Range Configurations */
  k_qmc5883l_range_2g        = 0x00, /**< Full-scale range: ±2 Gauss. */
  k_qmc5883l_range_8g        = 0x10, /**< Full-scale range: ±8 Gauss. */

  /* Over-Sampling Ratio (OSR) Configurations */
  k_qmc5883l_osr_512         = 0x00, /**< Over-sampling ratio: 512. */
  k_qmc5883l_osr_256         = 0x40, /**< Over-sampling ratio: 256. */
  k_qmc5883l_osr_128         = 0x80, /**< Over-sampling ratio: 128. */
  k_qmc5883l_osr_64          = 0xC0  /**< Over-sampling ratio: 64. */
} qmc5883l_commands_t;

/**
 * @brief Enumeration of QMC5883L sensor states.
 *
 * Defines the possible operational states for the QMC5883L magnetometer sensor,
 * including readiness, data updates, initialization status, and error conditions.
 */
typedef enum : uint8_t {
  k_qmc5883l_ready          = 0x00, /**< Sensor is initialized and ready to read data. */
  k_qmc5883l_data_updated   = 0x01, /**< New sensor data has been updated. */
  k_qmc5883l_uninitialized  = 0x10, /**< Sensor is not initialized. */
  k_qmc5883l_error          = 0xF0, /**< General catch-all error state. */
  k_qmc5883l_power_on_error = 0xA1, /**< Error occurred during power-on initialization. */
  k_qmc5883l_reset_error    = 0xA2, /**< Error occurred during reset operation. */
} qmc5883l_states_t;

/* Structs ********************************************************************/

/**
 * @brief Structure to hold the scaling configuration for the QMC5883L magnetometer.
 *
 * Contains the register value for setting the full-scale range and the corresponding
 * scaling factor for converting raw magnetic field data to microteslas (µT).
 */
typedef struct {
  uint8_t range; /**< Register value for setting the full-scale range in the QMC5883L. */
  float   scale; /**< Scaling factor to convert raw data to magnetic field strength in µT. */
} qmc5883l_scale_t;

/**
 * @brief Structure to store QMC5883L sensor data.
 *
 * Holds data related to the QMC5883L magnetometer sensor, including I2C communication details,
 * magnetic field measurements along the X, Y, and Z axes, the calculated heading (yaw), and 
 * state management fields for error handling and retries.
 */
typedef struct {
  uint8_t    i2c_address;        /**< I2C address used for communication with the sensor. */
  uint8_t    i2c_bus;            /**< I2C bus number used for communication. */
  float      mag_x;              /**< Measured X-axis magnetic field strength in µT. */
  float      mag_y;              /**< Measured Y-axis magnetic field strength in µT. */
  float      mag_z;              /**< Measured Z-axis magnetic field strength in µT. */
  float      heading;            /**< Calculated heading (yaw) in degrees. */
  uint8_t    state;              /**< Current operational state of the sensor (see `qmc5883l_states_t`). */
  uint8_t    retry_count;        /**< Number of consecutive reinitialization attempts. */
  uint32_t   retry_interval;     /**< Current interval between reinitialization attempts, in ticks. */
  TickType_t last_attempt_ticks; /**< Tick count of the last reinitialization attempt. */
} qmc5883l_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Convert QMC5883L data to JSON.
 *
 * @param[in] sensor_data Pointer to `qmc5883l_data_t` structure
 */
char *qmc5883l_data_to_json(const qmc5883l_data_t *data);

/**
 * @brief Initializes the QMC5883L sensor for continuous measurement mode.
 *
 * This function initializes the I2C driver for the QMC5883L sensor, powering on the device,
 * resetting it, and configuring it to start taking measurements. The sensor configuration
 * settings include the operating mode, output data rate, and range.
 *
 * @param[in,out] sensor_data Pointer to the `qmc5883l_data_t` structure containing:
 *                            - `i2c_address`: Address for I2C communication (input).
 *                            - `i2c_bus`: Bus number for I2C communication (input).
 *                            - `state`: Updated to indicate initialization status (output).
 *
 * @return
 * - `ESP_OK` on successful initialization.
 * - An error code from `esp_err_t` if initialization fails.
 *
 * @note Call this function once during setup to handle initial sensor configuration.
 */
esp_err_t qmc5883l_init(void *sensor_data);

/**
 * @brief Reads magnetic field data from the QMC5883L sensor.
 *
 * This function retrieves magnetic field data from the QMC5883L sensor, converting
 * the raw output values from the sensor's X, Y, and Z registers into microtesla units.
 * Additionally, it calculates the heading (yaw) based on the X and Y components.
 *
 * @param[in,out] sensor_data Pointer to `qmc5883l_data_t` structure that contains:
 *                            - `mag_x`, `mag_y`, `mag_z`: Updated with the magnetic field data in µT (output).
 *                            - `heading`: Updated with the calculated heading in degrees (output).
 *                            - `state`: Updated to indicate data retrieval status (output).
 *
 * @return
 * - `ESP_OK` on successful read.
 * - `ESP_FAIL` on unsuccessful read.
 *
 * @note Ensure the QMC5883L sensor is initialized with `qmc5883l_init` before calling this function.
 */
esp_err_t qmc5883l_read(qmc5883l_data_t *sensor_data);

/**
 * @brief Manages exponential backoff and retries for QMC5883L sensor reinitialization on error.
 *
 * This function checks the sensor's state for errors. If an error is present, it initiates
 * a reinitialization attempt, applying an exponential backoff mechanism that increases the
 * retry interval after each failure up to a maximum interval. On successful reinitialization,
 * it resets the retry counter and interval.
 *
 * **Logic and Flow:**
 * - On error (`state` set to non-zero), the function checks the time since the
 *   last reinitialization attempt.
 * - If the retry interval has elapsed, a reinitialization attempt is made.
 *   - On success, `state` is set to `k_qmc5883l_ready`, and the retry counter and
 *     interval are reset.
 *   - On failure, `retry_count` is incremented.
 * - When the max retries (`qmc5883l_max_retries`) are reached, `retry_count` resets,
 *   and the interval doubles up to a limit (`qmc5883l_max_backoff_interval`).
 *
 * @param[in,out] sensor_data Pointer to `qmc5883l_data_t` structure containing:
 *                            - `state`: Sensor state (input/output), where a non-zero
 *                              value indicates an error.
 *                            - `retry_count`: Retry attempt counter (input/output).
 *                            - `retry_interval`: Retry interval in ticks (input/output).
 *                            - `last_attempt_ticks`: Time of last reinitialization attempt (input/output).
 *
 * @note This function should be called within `qmc5883l_tasks` to ensure regular
 *       error monitoring and recovery attempts with exponential backoff.
 */
void qmc5883l_reset_on_error(qmc5883l_data_t *sensor_data);

/**
 * @brief Executes periodic tasks for the QMC5883L sensor, including data reading and error handling.
 *
 * The `qmc5883l_tasks` function is intended to run continuously in a loop. It periodically
 * reads data from the QMC5883L sensor at intervals defined by `qmc5883l_polling_rate_ticks`,
 * checking for errors and using `qmc5883l_reset_on_error` to manage retries with backoff.
 *
 * @param[in,out] sensor_data Pointer to `qmc5883l_data_t` structure containing:
 *                            - `mag_x`, `mag_y`, `mag_z`, `heading`: Holds the latest sensor data (output).
 *                            - `state`, `retry_count`, `retry_interval`: Managed for error recovery (input/output).
 *
 * @note Execute this function as part of a FreeRTOS task to maintain continuous
 *       data acquisition and error management for the QMC5883L sensor.
 */
void qmc5883l_tasks(void *sensor_data);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_QMC5883L_HAL_H */
