/* components/controllers/pca9685_hal/include/pca9685_hal.h */

#ifndef TOPOROBO_PCA9685_HAL_H
#define TOPOROBO_PCA9685_HAL_H

/* PCA9685 16-Channel 12-Bit PWM Driver IC */
/* Communicates over I2C protocol with configurable address range from 0x40 to 0x7F */

/*******************************************************************************
 *
 *     +-----------------------+
 *     |       PCA9685         |
 *     |-----------------------|
 *     | VCC    | 3.3V to 5V   |----------> VCC
 *     | GND    | Ground       |----------> GND
 *     | SCL    | I2C Clock    |----------> GPIO_NUM_22 (100,000Hz)
 *     | SDA    | I2C Data     |----------> GPIO_NUM_21 (100,000Hz)
 *     | OE     | Output Enable|----------> GND (optional, enables PWM output)
 *     | A0-A5  | Address Pins |----------> Floating or GND/VCC to set address
 *     | V+     | Servo Power  |----------> External Power (e.g., 5-6V for servos)
 *     +-----------------------+
 *
 *     Block Diagram for Wiring
 *
 *     +----------------------------------------------------+
 *     |                      PCA9685                       |
 *     |                                                    |
 *     |   +----------------+     +---------------------+   |
 *     |   | PWM Channels   |---->| PWM Control Logic   |   |
 *     |   | (16 Channels)  |     |                     |   |
 *     |   +----------------+     +---------------------+   |
 *     |                                                    |
 *     |   +------------------+    +--------------------+   |
 *     |   | Oscillator       |--->| Frequency Control  |   |
 *     |   | Circuit          |    | Unit               |   |
 *     |   +------------------+    +--------------------+   |
 *     |                                                    |
 *     |   +---------------------+                          |
 *     |   | I2C Interface       |<-------------------------|
 *     |   | (SDA, SCL, Address) |                          |
 *     |   +---------------------+                          |
 *     |                                                    |
 *     |   +---------------------+                          |
 *     |   | Power Supply Unit   |                          |
 *     |   | (PSU)               |                          |
 *     |   +---------------------+                          |
 *     +----------------------------------------------------+
 *
 *     Internal Structure
 *
 ******************************************************************************/

#include <stdint.h>
#include "esp_err.h"

/* Constants ******************************************************************/

extern const uint8_t  pca9685_scl_io;           /**< GPIO pin for I2C Serial Clock Line */
extern const uint8_t  pca9685_sda_io;           /**< GPIO pin for I2C Serial Data Line */
extern const uint32_t pca9685_i2c_freq_hz;      /**< I2C Bus Frequency in Hz */
extern const uint8_t  pca9685_i2c_address;      /**< Base I2C address for PCA9685 */
extern const uint8_t  pca9685_i2c_bus;          /**< I2C bus for PCA9685 */
extern const uint32_t pca9685_osc_freq;         /**< Internal Oscillator Frequency (25 MHz) */
extern const uint16_t pca9685_pwm_resolution;   /**< 12-bit PWM resolution (4096 steps) */
extern const uint16_t pca9685_default_pwm_freq; /**< Default PWM frequency (50 Hz for servos) */
extern const uint16_t pca9685_max_pwm_value;    /**< Maximum value for PWM duty cycle (4095) */
extern const uint16_t pca9685_pwm_period_us;    /**< Total PWM period for 50Hz (20000 µs) */
extern const char    *pca9685_tag;              /**< Tag for logs */

/* Enums **********************************************************************/

/**
 * @enum pca9685_states_
 * @brief Enum to represent the state of the PCA9685.
 *
 * This enum defines the possible states of a PCA9685 board, indicating whether
 * the board is ready, uninitialized, or in an error state.
 */
typedef enum : uint8_t {
  k_pca9685_ready               = 0x00, /**< The PCA9685 board is ready and operational */
  k_pca9685_uninitialized       = 0x01, /**< The PCA9685 board is not yet initialized */
  k_pca9685_error               = 0xF0, /**< A general catch-all error */
  k_pca9685_power_on_error      = 0xA1, /**< An error occurred during power-on of the board */
  k_pca9685_reset_error         = 0xA2, /**< An error occurred during reset of the board */
  k_pca9685_communication_error = 0xA3  /**< An error occurred during I2C communication */
} pca9685_states_t;

/**
 * @enum pca9685_commands_
 * @brief Enum to represent the I2C commands for the PCA9685.
 *
 * This enum defines the I2C register addresses and relevant commands
 * for configuring and controlling the PCA9685 PWM driver.
 *
 * @note Only the channel 0 register addresses are explicitly defined.
 * For other channels (channel 1 to channel 15), the register addresses
 * can be calculated by adding 4 to the base address of channel 0.
 * For example:
 * - Channel 1 on low byte address = `k_pca9685_channel0_on_l_cmd + 4`
 * - Channel 2 on low byte address = `k_pca9685_channel0_on_l_cmd + 8`
 * - And so on, for subsequent channels.
 */
typedef enum : uint8_t {
  k_pca9685_mode1_cmd              = 0x00, /**< MODE1 register (mode configuration) */
  k_pca9685_mode2_cmd              = 0x01, /**< MODE2 register (output configuration) */
  k_pca9685_prescale_cmd           = 0xFE, /**< Prescaler for PWM frequency */
  k_pca9685_channel0_on_l_cmd      = 0x06, /**< Channel 0 output on time (low byte) */
  k_pca9685_channel0_on_h_cmd      = 0x07, /**< Channel 0 output on time (high byte) */
  k_pca9685_channel0_off_l_cmd     = 0x08, /**< Channel 0 output off time (low byte) */
  k_pca9685_channel0_off_h_cmd     = 0x09, /**< Channel 0 output off time (high byte) */
  k_pca9685_auto_increment_cmd     = 0x20, /**< Auto-increment bit for MODE1 register */
  k_pca9685_restart_cmd            = 0x80, /**< Restart bit to enable PWM after setting frequency */
  k_pca9685_sleep_cmd              = 0x10, /**< Sleep bit to put PCA9685 into low-power mode */
  k_pca9685_allcall_cmd            = 0x01, /**< ALLCALL bit for addressing all PCA9685 devices simultaneously */
  k_pca9685_output_change_ack_cmd  = 0x01, /**< Output change on ACK bit in MODE2 register */
  k_pca9685_output_change_stop_cmd = 0x00, /**< Output change on STOP condition in MODE2 register */
} pca9685_commands_t;

/* Structs ********************************************************************/

/**
 * @struct pca9685_board_
 * @brief Structure representing each PCA9685 board in the system.
 *
 * This structure holds information about each PCA9685 board, including the
 * I2C bus number, current state, the board ID, and the number of boards. I
 * also contains a pointer to the next board in a singly linked list.
 */
typedef struct pca9685_board_t {
  uint8_t                 i2c_address; /**< Base I2C address */
  uint8_t                 i2c_bus;     /**< I2C bus number used for communication */
  uint8_t                 state;       /**< Current state of the PCA9685, using the pca9685_states_t enum */
  uint8_t                 board_id;    /**< The board's ID; used to distinguish boards in multi-board setups */
  uint8_t                 num_boards;  /**< Number of initialized PCA9685 boards */
  struct pca9685_board_t *next;        /**< Pointer to the next board in the PCA9685 singly linked list */
} pca9685_board_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the PCA9685 PWM driver over I2C for multiple boards.
 *
 * This function initializes the I2C driver for the specified number of PCA9685
 * boards and sets up the PWM frequency for controlling servos or other PWM-controlled devices.
 * It ensures that each board is initialized only once.
 *
 * @param[in,out] controller_data Pointer to the head of the linked list containing PCA9685 boards.
 * @param[in] num_boards Number of boards to initialize.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t pca9685_init(pca9685_board_t **controller_data, uint8_t num_boards);

/**
 * @brief Sets the angle for one or more servo motors on a specific board.
 *
 * This function takes a motor mask, an angle, and a board ID to set the position of one
 * or more servo motors on a specific PCA9685 board. The function ensures the board is initialized
 * and ready before attempting to set the angle.
 *
 * @param[in] controller_data Pointer to the linked list of PCA9685 boards.
 * @param[in] motor_mask Mask indicating which motors to control (bitmask).
 * @param[in] board_id The ID of the board to control.
 * @param[in] angle The desired servo angle (0-180 degrees).
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t pca9685_set_angle(pca9685_board_t *controller_data, uint16_t motor_mask,
                            uint8_t board_id, float angle);

#endif /* TOPOROBO_PCA9685_HAL_H */

