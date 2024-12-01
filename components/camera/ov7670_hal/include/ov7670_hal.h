/* components/camera/ov7670_hal/include/ov7670_hal.h */

/* OV7670 HAL (Hardware Abstraction Layer) Header File
 *
 * This file provides the interface for interacting with the OV7670 camera module.
 * The OV7670 is a CMOS image sensor capable of capturing still images and video. I
 * outputs data over an 8-bit parallel interface with synchronization signals, which
 * are managed externally by an FPGA or microcontroller.
 *
 *******************************************************************************
 *
 *    +-----------------------+
 *    |        OV7670         |
 *    |-----------------------|
 *    | 3V3  | 3.3V Power     |----------> 3.3V
 *    | GND  | Ground         |----------> GND
 *    | SCL  | I2C Clock      |----------> GPIO_NUM_22 (I2C Configuration)
 *    | SDA  | I2C Data       |----------> GPIO_NUM_21 (I2C Configuration)
 *    | VS   | Vertical Sync  |----------> DE10-Lite or External Controller
 *    | HS   | Horizontal Sync|----------> DE10-Lite or External Controller
 *    | PCLK | Pixel Clock    |----------> DE10-Lite or External Controller
 *    | XLK  | External Clock |----------> DE10-Lite or External Clock Source
 *    | D[7] | Parallel Data  |----------> DE10-Lite or External Controller
 *    | D[6] | Parallel Data  |----------> DE10-Lite or External Controller
 *    | D[5] | Parallel Data  |----------> DE10-Lite or External Controller
 *    | D[4] | Parallel Data  |----------> DE10-Lite or External Controller
 *    | D[3] | Parallel Data  |----------> DE10-Lite or External Controller
 *    | D[2] | Parallel Data  |----------> DE10-Lite or External Controller
 *    | D[1] | Parallel Data  |----------> DE10-Lite or External Controller
 *    | D[0] | Parallel Data  |----------> DE10-Lite or External Controller
 *    | RET  | Reset          |----------> DE10-Lite or External Controller
 *    | PWDN | Power Down     |----------> DE10-Lite or External Controller
 *    +-----------------------+
 *
 *    Block Diagram for Wiring
 *
 *    +----------------------------------------------------+
 *    |                     OV7670 Camera                  |
 *    |                                                    |
 *    |   +-------------------+                            |
 *    |   | Image Sensor      |                            |
 *    |   | (Lens + CMOS)     |                            |
 *    |   +-------------------+                            |
 *    |                                                    |
 *    |   +-------------------+     +------------------+   |
 *    |   | Signal Processing  |--->| Parallel Data    |   |
 *    |   | Unit               |    | Output (D[0:7])  |   |
 *    |   +--------------------+    +------------------+   |
 *    |                                                    |
 *    |   +---------------------+                          |
 *    |   | Sync Signals        |--------------------------|
 *    |   | (VS, HS, PCLK)      |                          |
 *    |   +---------------------+                          |
 *    |                                                    |
 *    |   +---------------------+                          |
 *    |   | I2C Interface       |<-------------------------|
 *    |   | (SCL, SDA)          |                          |
 *    |   +---------------------+                          |
 *    |                                                    |
 *    |   +---------------------+                          |
 *    |   | Power Supply Unit   |                          |
 *    |   | (PSU)               |                          |
 *    |   +---------------------+                          |
 *    +----------------------------------------------------+
 *
 ******************************************************************************/

#ifndef TOPOROBO_OV7670_HAL_H
#define TOPOROBO_OV7670_HAL_H

#include <stdint.h>
#include "esp_err.h"

/* Constants ******************************************************************/

/**
 * @brief GPIO pin for I2C Clock (SCL).
 */
extern const uint8_t ov7670_scl_io;

/**
 * @brief GPIO pin for I2C Data (SDA).
 */
extern const uint8_t ov7670_sda_io;

/**
 * @brief Polling rate for checking camera configuration status in ticks.
 */
extern const uint32_t ov7670_polling_rate_ticks;

/**
 * @brief OV7670 I2C address.
 */
extern const uint8_t ov7670_i2c_address;

/* Enums **********************************************************************/

/**
 * @enum ov7670_states_
 * @brief Represents the operational states of the OV7670 camera module.
 */
typedef enum {
    k_ov7670_uninitialized = 0x00, /**< Camera is not initialized */
    k_ov7670_ready         = 0x01, /**< Camera is ready for use */
    k_ov7670_config_error  = 0xF0  /**< Configuration error occurred */
} ov7670_states_t;

/* Structs ********************************************************************/

/**
 * @struct ov7670_data_
 * @brief Structure for storing OV7670 camera module data.
 */
typedef struct {
    uint8_t             state; /**< Current state of the camera module */
    uint8_t             retries; /**< Retry count for configuration errors */
    uint32_t            retry_interval; /**< Retry interval in ticks */
    uint32_t            last_attempt_ticks; /**< Tick count of last retry */
} ov7670_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the OV7670 camera module.
 *
 * Configures the I2C interface and applies default settings to the camera module.
 *
 * @param[in,out] sensor_data Pointer to the `ov7670_data_t` structure that will
 *                            hold the camera's operational state.
 *
 * @return
 * - `ESP_OK` on successful initialization.
 * - Error codes from `esp_err_t` on failure.
 */
esp_err_t ov7670_init(ov7670_data_t *sensor_data);

/**
 * @brief Handles errors and retries configuration of the OV7670 module.
 *
 * Attempts to reconfigure the OV7670 module when errors occur, using an
 * exponential backoff strategy.
 *
 * @param[in,out] sensor_data Pointer to the `ov7670_data_t` structure managing
 *                            retry attempts and intervals.
 */
void ov7670_reset_on_error(ov7670_data_t *sensor_data);

/**
 * @brief Executes periodic tasks for the OV7670 module.
 *
 * This function should be called periodically to monitor the camera's status
 * and handle reconfiguration if needed.
 *
 * @param[in,out] sensor_data Pointer to the `ov7670_data_t` structure.
 */
void ov7670_tasks(void *sensor_data);

#endif /* TOPOROBO_OV7670_HAL_H */

