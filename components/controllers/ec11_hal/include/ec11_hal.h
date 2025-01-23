#ifndef TOPOROBO_EC11_HAL_H
#define TOPOROBO_EC11_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Constants ******************************************************************/

extern const char    *ec11_tag;      /**< Tag for logging messages related to the EC11 encoder. */
extern const uint32_t poll_delay_ms; /**< Delay between encoder state polls in milliseconds. */

/* Enums **********************************************************************/

/**
 * @brief Enumeration of EC11 encoder states.
 *
 * Represents the possible states of the EC11 rotary encoder based on its output pins.
 */
typedef enum : uint8_t {
  k_ec11_state_00 = 0b00, /**< Both outputs low */
  k_ec11_state_01 = 0b01, /**< Output A low, B high */
  k_ec11_state_11 = 0b11, /**< Both outputs high */
  k_ec11_state_10 = 0b10, /**< Output A high, B low */
} ec11_states_t;

/**
 * @brief Enumeration of EC11 encoder operational states.
 *
 * Defines the possible operational states of the EC11 encoder,
 * including initialization status and error conditions.
 */
typedef enum : uint8_t {
  k_ec11_ready         = 0x00, /**< Encoder is initialized and ready */
  k_ec11_uninitialized = 0x10, /**< Encoder is not initialized */
  k_ec11_error         = 0xF0, /**< General error state */
} ec11_op_states_t;

/* Structs ********************************************************************/

/**
 * @brief Structure to store EC11 encoder data and configuration.
 *
 * Contains GPIO pin assignments, current state, position tracking,
 * and button state for a single EC11 encoder instance.
 */
typedef struct {
  uint8_t  pin_a;           /**< GPIO pin for encoder output A */
  uint8_t  pin_b;           /**< GPIO pin for encoder output B */
  uint8_t  pin_btn;         /**< GPIO pin for encoder push button */
  int32_t  position;        /**< Current encoder position (increments/decrements) */
  uint8_t  state;           /**< Current operational state (see ec11_op_states_t) */
  bool     button_pressed;  /**< Current state of the push button */
  uint8_t  prev_state;      /**< Previous encoder state for rotation detection */
} ec11_data_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initializes an EC11 rotary encoder instance.
 *
 * Configures the GPIO pins for the encoder's outputs and button,
 * and sets up the necessary pull-up resistors.
 *
 * @param[in] encoder Pointer to the encoder data structure
 * @return
 * - ESP_OK on success
 * - ESP_FAIL or other error codes on failure
 */
esp_err_t ec11_init(ec11_data_t *encoder);

/**
 * @brief Task function for polling the EC11 encoder state.
 *
 * Continuously monitors the encoder's outputs to detect rotation direction
 * and button state. Designed to run as a FreeRTOS task.
 *
 * @param[in] arg Pointer to the encoder data structure
 */
void ec11_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_EC11_HAL_H */
