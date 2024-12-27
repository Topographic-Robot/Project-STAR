/* components/camera/ov7670_hal/include/ov7670_hal.h */

/* OV7670 HAL (Hardware Abstraction Layer) Header File
 *
 * This file provides the interface for interacting with the OV7670 camera module.
 * The OV7670 is a CMOS image sensor capable of capturing still images and video. It
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

extern const uint8_t  ov7670_scl_io;             /**< GPIO pin for I2C Clock (SCL) */
extern const uint8_t  ov7670_sda_io;             /**< GPIO pin for I2C Data (SDA) */
extern const uint32_t ov7670_polling_rate_ticks; /**< Polling rate for checking camera configuration status in ticks */
extern const uint8_t  ov7670_i2c_address;        /**< OV7670 I2C address */
extern const uint8_t  ov7670_i2c_bus;            /**< OV7670 I2C bus */
extern const uint32_t ov7670_i2c_freq_hz;        /**< OV7670 I2C Freq in Hz (100k) */

/* Enums **********************************************************************/

/**
 * @enum ov7670_states_t
 * @brief Represents the operational states of the OV7670 camera module.
 */
typedef enum : uint8_t {
  k_ov7670_uninitialized = 0x00, /**< Camera is not initialized */
  k_ov7670_ready         = 0x01, /**< Camera is ready for use */
  k_ov7670_config_error  = 0xF0, /**< Configuration error occurred */
} ov7670_states_t;

/**
 * @enum ov7670_resolution_t
 * @brief Supported resolutions for the OV7670 camera.
 */
typedef enum : uint8_t {
  k_ov7670_res_vga  = 0x00, /**< VGA (640x480), default */
  k_ov7670_res_qvga = 0x10, /**< QVGA (320x240) */
} ov7670_resolution_t;

/**
 * @enum ov7670_output_format_t
 * @brief Supported output formats for the OV7670 camera.
 */
typedef enum : uint8_t {
  k_ov7670_output_yuv   = 0x00, /**< YUV422 format, default */
  k_ov7670_output_rgb   = 0x04, /**< RGB565 format */
  k_ov7670_output_bayer = 0x01, /**< Bayer RAW format */
} ov7670_output_format_t;

/**
 * @enum ov7670_clock_divider_t
 * @brief Supported clock dividers for the OV7670 camera.
 */
typedef enum : uint8_t {
  k_ov7670_clk_div_1 = 0x00, /**< Clock divider 1 (no division) */
  k_ov7670_clk_div_2 = 0x01, /**< Clock divider 2 */
  k_ov7670_clk_div_4 = 0x02, /**< Clock divider 4 */
  k_ov7670_clk_div_8 = 0x03, /**< Clock divider 8 */
} ov7670_clock_divider_t;

/**
 * @enum ov7670_register_t
 * @brief OV7670 register addresses.
 */
typedef enum : uint8_t {
  k_ov7670_reg_com7   = 0x12, /**< Common Control 7 */
  k_ov7670_reg_clkrc  = 0x11, /**< Clock Control */
  k_ov7670_reg_rgb444 = 0x8C, /**< RGB444 Control */
  k_ov7670_reg_com15  = 0x40, /**< Common Control 15 */
  k_ov7670_reg_tslb   = 0x3A, /**< Line Buffer Control */
  k_ov7670_reg_com3   = 0x0C, /**< Common Control 3 */
  k_ov7670_reg_com14  = 0x3E, /**< Common Control 14 */
} ov7670_register_t;

/* Structs ********************************************************************/

/**
 * @struct ov7670_config_t
 * @brief Configuration settings for the OV7670 camera module.
 */
typedef struct {
  ov7670_resolution_t    resolution;    /**< Desired resolution (QVGA, VGA, etc.) */
  ov7670_output_format_t output_format; /**< Desired output format (RGB, YUV, etc.) */
  ov7670_clock_divider_t clock_divider; /**< Clock divider for internal pixel clock */
} ov7670_config_t;

/**
 * @struct ov7670_data_t
 * @brief Structure for storing OV7670 camera module data.
 */
typedef struct {
  uint8_t         state;              /**< Current state of the camera module */
  uint8_t         retries;            /**< Retry count for configuration errors */
  uint32_t        retry_interval;     /**< Retry interval in ticks */
  uint32_t        last_attempt_ticks; /**< Tick count of last retry */
  ov7670_config_t config;             /**< Current configuration settings */
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

