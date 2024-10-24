#ifndef TOPOROBO_BH1750_HAL_H
#define TOPOROBO_BH1750_HAL_H

/* Digital 16bit Serial Output Type Ambient Light Sensor IC */
/* Uses I2C for Communication with address 0x23 */

/*******************************************************************************
 *
 *    +----------------------------------------+
 *    |                BH1750                  |
 *    |                                        |
 *    |   +---------------------------------+  |
 *    |   |                                 |  |
 *    |   |   +-----------------------------+  |
 *    |   |   |        +--------------------+  |
 *    |   |   | VCC    |  2.4V to 3.6V      |--------->| VCC
 *    |   |   +-----------------------------+  |
 *    |   |   | GND    | Ground             |--------->| GND
 *    |   |   +-----------------------------+  |
 *    |   |   | SCL    | Serial Clock       |--------->| GPIO_NUM_22 (100,000Hz)
 *    |   |   +-----------------------------+  |
 *    |   |   | SDA    | Serial Data        |--------->| GPIO_NUM_21 (100,000Hz)
 *    |   |   +-----------------------------+  |
 *    |   |   | ADDR   | I2C Address Select |--------->| GND
 *    |   |   +-----------------------------+  |
 *    |   |                                 |  |
 *    |   +---------------------------------+  |
 *    +----------------------------------------+
 *    
 *    Block Diagram for wiring
 *    
 *    +----------------------------------------------------+
 *    |                    BH1750                          |
 *    |                                                    |
 *    |   +------------+     +-------------------+         |
 *    |   | Photodiode |---->| Transimpedance    |         |
 *    |   | (Sensor)   |     | Amplifier (TIA)   |         |
 *    |   +------------+     +-------------------+         |
 *    |                                                    |
 *    |   +------------------+      +----------------+     |
 *    |   | Analog-to-Digital|----->| Control Logic  |     |
 *    |   | Converter (ADC)  |      |                |     |
 *    |   +------------------+      +----------------+     |
 *    |                                                    |
 *    |   +---------------------+                          |
 *    |   | Serial Interface    |<-------------------------|
 *    |   | (I2C)               |                          |
 *    |   +---------------------+                          |
 *    |                                                    |
 *    |   +---------------------+                          |
 *    |   | Power Supply Unit   |                          |
 *    |   | (PSU)               |                          |
 *    |   +---------------------+                          |
 *    +----------------------------------------------------+
 *    
 *    Internal Structure
 *
 ******************************************************************************/

#include <stdint.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/* Constants ******************************************************************/

extern const uint8_t  bh1750_i2c_address;        /**< I2C address for BH1750 */
extern const char    *bh1750_tag;                /**< Tag for logs */
extern const uint8_t  bh1750_scl_io;             /**< GPIO pin for I2C Serial Clock Line */
extern const uint8_t  bh1750_sda_io;             /**< GPIO pin for I2C Serial Data Line */
extern const uint32_t bh1750_i2c_freq_hz;        /**< I2C Bus Frequency in Hz */
extern const uint32_t bh1750_polling_rate_ticks; /**< Polling rate (5 seconds) */

/* Enums **********************************************************************/

/**
 * @enum bh1750_commands_t
 * @brief Enum to represent the I2C commadns for the BH1750 sensor.
 *
 * This enum defines the possible I2C commands for the BH1750 sensor.
 */
typedef enum : uint8_t {
  k_bh1750_power_down_cmd                = 0x00, /**< Power down command */
  k_bh1750_power_on_cmd                  = 0x01, /**< Power on command */
  k_bh1750_reset_cmd                     = 0x07, /**< Reset command */
  k_bh1750_cont_low_res_mode_cmd         = 0x13, /**< Continuously low-resolution mode command */
  /* Below are currently unused: */                                                 
  k_bh1750_cont_high_res_mode_cmd        = 0x10, /**< Continuously high-resolution mode command */
  k_bh1750_cont_high_res_mode2_cmd       = 0x11, /**< Continuously high-resolution mode 2 command */
  k_bh1750_one_time_high_res_mode_cmd    = 0x20, /**< One-time high-resolution mode command */
  k_bh1750_one_time_high_res_mode2_cmd   = 0x21, /**< One-time high-resolution mode 2 command */
  k_bh1750_one_time_low_res_mode_cmd     = 0x23, /**< One-time low-resolution mode command */
  k_bh1750_change_meas_time_high_bit_cmd = 0x40, /**< Change measurement time high bit command */
  k_bh1750_change_meas_time_low_bit_cmd  = 0x60, /**< Change measurement time low bit command */
} bh1750_commands_t;

/**
 * @enum bh1750_states_t
 * @brief Enum to represent the state of the BH1750 sensor.
 *
 * This enum defines the possible states for the BH1750 sensor.
 */
typedef enum : uint8_t {
  k_bh1750_ready              = 0x00, /**< Sensor is ready to read data. */
  k_bh1750_data_updated       = 0x01, /**< Sensor data has been updated. */
  k_bh1750_uninitialized      = 0x10, /**< Sensor is not initialized. */
  k_bh1750_error              = 0xF0, /**< A general catch all error */
  k_bh1750_power_on_error     = 0xA1, /**< An error occurred during power on. */
  k_bh1750_reset_error        = 0xA2, /**< An error occurred during reset. */
  k_bh1750_cont_low_res_error = 0xA3, /**< An error occurred setting continuous high-resolution mode */
  k_bh1750_power_cycle_error  = 0xA4, /**< An error occurred when trying to power cycle */
} bh1750_states_t;

/* Structs ********************************************************************/

/**
 * @struct bh1750_data_t
 * @brief Structure to store BH1750 sensor data.
 *
 * This structure holds the I2C bus number used for communication, 
 * the light intensity value measured by the BH1750 sensor, and a 
 * state flag used to track the sensor's status.
 * 
 * These flags are set in the `bh1750_states_t` enum, always verify 
 * with the enum.
 * 
 * Additionally, the structure contains a mutex (`sensor_mutex`) to ensure
 * thread-safe access when the sensor data is being read or updated.
 */
typedef struct {
  uint8_t           i2c_bus;      /**< I2C bus number used for communication. */
  float             lux;          /**< Measured light intensity in lux. */
  uint8_t           state;        /**< Sensor state, set in `bh1750_states_t` enum. */
  SemaphoreHandle_t sensor_mutex; /**< Mutex for protecting access to sensor data. */
} bh1750_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initialize the BH1750 sensor over I2C.
 *
 * This function initializes the I2C driver for the BH1750 sensor and sets
 * it up for continuous high-resolution measurement mode. It powers on the 
 * device, resets it, and configures the resolution mode. If the device fails
 * to be configured, this will try and reset the device to configure again.
 * This however still might not work and calling the function `bh1750_reset_on_error` 
 * until the bh1750_data_t's `state` flag is `0x00` is recommended.
 *
 * @param[in,out] sensor_data Pointer to the `bh1750_data_t` structure that 
 *   will hold the I2C bus number. The `i2c_bus` member will be set during 
 *   initialization.
 *
 * @param[in] first_time Boolean to let the function know if you have already
 *   called this before, its not recommended to run it as 'first_time' multiple
 *   times since memory allocation for a Semaphore occurs and can run into 
 *   memory issues if ran too much. Run it as 'first_time' once then set it to
 *   false when re-running the function.
 *
 * @return
 *   - ESP_OK on success.
 *   - An error code from the `esp_err_t` enumeration on failure.
 *
 * @note 
 *   - This should only be called once.
 *   - Delays are introduced after power on, reset, and resolution
 *     mode settings to ensure proper sensor initialization. 
 */
esp_err_t bh1750_init(bh1750_data_t *sensor_data, bool first_time);

/**
 * @brief Reads light intensity data from the BH1750 sensor.
 *
 * This function reads 2 bytes of data from the BH1750 sensor and converts the
 * raw data into lux. The conversion factor is 1.2 to convert the raw sensor 
 * value into a meaningful lux value.
 *
 * @param[in,out] sensor_data Pointer to a `bh1750_data_t` struct that contains:
 *   - `i2c_bus`: The I2C bus number to use for communication (input).
 *   - `lux`: Will be updated with the light intensity in lux (output).
 *            If the reading fails, `lux` will be set to -1.0.
 *
 * @note Ensure the bh1750 is initialized before calling this function.
 */
void bh1750_read(bh1750_data_t *sensor_data);

/**
 * @brief Checks if the BH1750 sensor encountered an error and attempts to reset it.
 *
 * This function checks the `state` flag of the `bh1750_data_t` structure to 
 * determine if an error occurred. If an error is detected, it runs the BH1750
 * initialization function to reset the sensor. If the reset is successful, the 
 * `state` flag will be updated to `k_bh1750_ready`.
 *
 * @param[in,out] sensor_data Pointer to a `bh1750_data_t` struct that contains:
 *   - `state`: The current state of the sensor (input/output). 
 *              A non-zero value indicates an error. After a successful reset, 
 *              this flag is reset.
 */
void bh1750_reset_on_error(bh1750_data_t *sensor_data);

/**
 * @brief Executes periodic tasks for the BH1750 sensor.
 * 
 * This function reads light intensity data from the BH1750 sensor and
 * checks for any errors in the sensor state. If an error is detected, the sensor 
 * is reset to attempt recovery. The function also introduces a delay based 
 * on the polling rate to ensure continuous operation at the desired intervals.
 * 
 * @param[in,out] sensor_data Pointer to the `bh1750_data_t` structure that
 *                            contains sensor data used for reading and error checking.
 * 
 * @note The delay is calculated from the `bh1750_polling_rate_ticks` 
 *       global variable, which defines the polling rate in system ticks.
 */
void bh1750_tasks(void *sensor_data);

#endif /* TOPOROBO_BH1750_HAL_H */
