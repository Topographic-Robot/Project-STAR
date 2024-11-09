/* components/sensors/qmc5883l_hal/include/qmc5883l_hal.h */

#ifndef TOPOROBO_QMC5883L_HAL_H
#define TOPOROBO_QMC5883L_HAL_H

/* QMC5883L 3-axis Magnetometer Sensor IC */
/* Communicates over I2C protocol with a configurable address 0x0D */

/*******************************************************************************
 *
 *     +-----------------------+
 *     |       QMC5883L        |
 *     |-----------------------|
 *     | VCC  | 3.3V or 5V     |----------> VCC
 *     | GND  | Ground         |----------> GND
 *     | SCL  | I2C Clock      |----------> GPIO_NUM_22 (100,000Hz)
 *     | SDA  | I2C Data       |----------> GPIO_NUM_21 (100,000Hz)
 *     | DRDY | Data Ready Pin |----------> Floating (optional)
 *     +-----------------------+
 *
 *     Block Diagram for Wiring
 *
 *     +----------------------------------------------------+
 *     |                    QMC5883L                        |
 *     |                                                    |
 *     |   +----------------+    +-------------------+      |
 *     |   | Magnetometer   |--->| Signal Processing |      |
 *     |   | Sensor         |    | Unit              |      |
 *     |   +----------------+    +-------------------+      |
 *     |                                                    |
 *     |   +------------------+                             |
 *     |   | I2C Interface    |<----------------------------|
 *     |   | (SDA, SCL)       |                             |
 *     |   +------------------+                             |
 *     |                                                    |
 *     |   +------------------+                             |
 *     |   | Power Supply Unit|                             |
 *     |   | (PSU)            |                             |
 *     |   +------------------+                             |
 *     +----------------------------------------------------+
 *
 *     Internal Structure
 *
 ******************************************************************************/

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/* Constants ******************************************************************/

extern const uint8_t  qmc5883l_i2c_address;        /**< I2C address for QMC5883L */
extern const uint8_t  qmc5883l_i2c_bus;            /**< I2C bus which the ESP32 uses */
extern const char    *qmc5883l_tag;                /**< Tag for logs */
extern const uint8_t  qmc5883l_scl_io;             /**< GPIO pin for I2C Serial Clock Line */
extern const uint8_t  qmc5883l_sda_io;             /**< GPIO pin for I2C Serial Data Line */
extern const uint32_t qmc5883l_i2c_freq_hz;        /**< I2C Bus Frequency in Hz */
extern const uint32_t qmc5883l_polling_rate_ticks; /**< Polling rate */
extern const uint8_t  qmc5883l_odr_setting;        /**< Output Data Rate setting */

/* Enums **********************************************************************/

/**
 * @enum qmc5883l_commands_t
 * @brief Enum to represent the I2C commands for the QMC5883L sensor.
 *
 * This enum defines the possible I2C commands for the QMC5883L sensor.
 */
typedef enum : uint8_t {
  /* Below are power and reset commands: */
  k_qmc5883l_reset_cmd       = 0x0B, /**< Command to reset the sensor */
  k_qmc5883l_set_reset_cmd   = 0x0B, /**< Command to set/reset period */
  /* Below are control and configuration commands: */
  k_qmc5883l_ctrl1_cmd       = 0x09, /**< Control Register 1: Mode, ODR, Range, OSR */
  k_qmc5883l_ctrl2_cmd       = 0x0A, /**< Control Register 2: Soft reset and standby mode */
  /* Below are data output registers: */
  k_qmc5883l_data_xout_l_cmd = 0x00, /**< X-axis low byte data */
  k_qmc5883l_data_xout_h_cmd = 0x01, /**< X-axis high byte data */
  k_qmc5883l_data_yout_l_cmd = 0x02, /**< Y-axis low byte data */
  k_qmc5883l_data_yout_h_cmd = 0x03, /**< Y-axis high byte data */
  k_qmc5883l_data_zout_l_cmd = 0x04, /**< Z-axis low byte data */
  k_qmc5883l_data_zout_h_cmd = 0x05, /**< Z-axis high byte data */
  /* Below are status and temperature commands: */
  k_qmc5883l_status_cmd      = 0x06, /**< Status register: DRDY, overflow, and data overflow */
  k_qmc5883l_temp_l_cmd      = 0x07, /**< Temperature low byte */
  k_qmc5883l_temp_h_cmd      = 0x08, /**< Temperature high byte */
  /* Below are specific configuration values for Control Register 1: */
  k_qmc5883l_mode_standby    = 0x00, /**< Standby mode */
  k_qmc5883l_mode_continuous = 0x01, /**< Continuous measurement mode */
  /* Output Data Rate (ODR) configurations */
  k_qmc5883l_odr_10hz        = 0x00, /**< Output Data Rate: 10 Hz */
  k_qmc5883l_odr_50hz        = 0x04, /**< Output Data Rate: 50 Hz */
  k_qmc5883l_odr_100hz       = 0x08, /**< Output Data Rate: 100 Hz */
  k_qmc5883l_odr_200hz       = 0x0C, /**< Output Data Rate: 200 Hz */
  /* Full-scale range configurations */
  k_qmc5883l_range_2g        = 0x00, /**< Full-scale range: ±2 Gauss */
  k_qmc5883l_range_8g        = 0x10, /**< Full-scale range: ±8 Gauss */
  /* Over-sampling ratio (OSR) configurations */
  k_qmc5883l_osr_512         = 0x00, /**< Over-sampling ratio: 512 */
  k_qmc5883l_osr_256         = 0x40, /**< Over-sampling ratio: 256 */
  k_qmc5883l_osr_128         = 0x80, /**< Over-sampling ratio: 128 */
  k_qmc5883l_osr_64          = 0xC0  /**< Over-sampling ratio: 64 */
} qmc5883l_commands_t;

/**
 * @enum qmc5883l_states_t
 * @brief Enum to represent the state of the BH1750 sensor.
 *
 * This enum defines the possible states for the BH1750 sensor.
 */
typedef enum : uint8_t {
  k_qmc5883l_ready              = 0x00, /**< Sensor is ready to read data. */
  k_qmc5883l_data_updated       = 0x01, /**< Sensor data has been updated. */
  k_qmc5883l_uninitialized      = 0x10, /**< Sensor is not initialized. */
  k_qmc5883l_error              = 0xF0, /**< A general catch all error */
  k_qmc5883l_power_on_error     = 0xA1, /**< An error occurred during power on. */
  k_qmc5883l_reset_error        = 0xA2, /**< An error occurred during reset. */
} qmc5883l_states_t;

/* Structs ********************************************************************/

/**
 * @struct qmc5883l_scale_t
 * @brief Structure that holds both the register value and the scaling factor for 
 *        the QMC5883L magnetometer.
 *
 * The QMC5883L outputs 16-bit signed integers representing the magnetic field in 
 * the X, Y, and Z axes. To convert the raw data into meaningful values (in microteslas), 
 * we use a scaling factor based on the selected range.
 */
typedef struct {
  uint8_t range; /**< Register value to set the full-scale range in QMC5883L */
  float   scale; /**< Scaling factor for converting raw data to microteslas (µT) */
} qmc5883l_scale_t;

/**
 * @struct qmc5883l_data_t
 * @brief Structure to store QMC5883L sensor data.
 *
 * This structure holds the I2C bus number used for communication,
 * the magnetic field strength in the X, Y, and Z axes measured by the QMC5883L sensor,
 * the calculated heading (yaw), and a state flag used to track the sensor's status.
 */
typedef struct {
  uint8_t           i2c_address;  /**< I2C address used for communication */
  uint8_t           i2c_bus;      /**< I2C bus number used for communication */
  float             mag_x;        /**< Measured X-axis magnetic field in µT */
  float             mag_y;        /**< Measured Y-axis magnetic field in µT */
  float             mag_z;        /**< Measured Z-axis magnetic field in µT */
  float             heading;      /**< Calculated heading (yaw) in degrees */
  uint8_t           state;        /**< Sensor state */
} qmc5883l_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initialize the QMC5883L sensor over I2C.
 *
 * This function initializes the I2C driver for the QMC5883L sensor and sets
 * it up for measurement mode. It powers on the device, resets it, and configures
 * the sensor for magnetic field data collection.
 *
 * @param[in,out] sensor_data Pointer to the `qmc5883l_data_t` structure that
 *   will hold the I2C bus number.
 *
 * @return
 *   - ESP_OK on success.
 *   - An error code from the `esp_err_t` enumeration on failure.
 */
esp_err_t qmc5883l_init(void *sensor_data);

/**
 * @brief Reads magnetic field data from the QMC5883L sensor.
 *
 * This function reads data from the QMC5883L sensor and converts the
 * raw data into magnetic field values in microteslas, and calculates the heading (yaw).
 *
 * @param[in,out] sensor_data Pointer to a `qmc5883l_data_t` struct that contains:
 *   - `i2c_bus`: The I2C bus number to use for communication (input).
 *   - `mag`: Will be updated with the magnetic field data (output).
 *   - `heading`: Will be updated with the calculated heading (output).
 */
void qmc5883l_read(qmc5883l_data_t *sensor_data);

/**
 * @brief Executes periodic tasks for the QMC5883L sensor.
 *
 * This function reads magnetic field data from the QMC5883L sensor and
 * checks for any errors in the sensor state. If an error is detected, the sensor
 * is reset to attempt recovery.
 *
 * @param[in,out] sensor_data Pointer to the `qmc5883l_data_t` structure that
 *                            contains sensor data used for reading and error checking.
 *
 * @note The delay between data reads is controlled by the `qmc5883l_polling_rate_ticks` 
 *       global variable, which defines the polling rate in system ticks.
 */
void qmc5883l_tasks(void *sensor_data);

#endif /* TOPOROBO_QMC5883L_HAL_H */

