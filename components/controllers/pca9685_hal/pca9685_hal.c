/* components/controllers/pca9685_hal/pca9685_hal.c */

#include "pca9685_hal.h"
#include "common/i2c.h"
#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "log_handler.h"
#include "error_handler.h"
#include "common/common_cleanup.h"
#include "common/common_setup.h"

/* Constants ******************************************************************/

const uint8_t     pca9685_scl_io           = GPIO_NUM_22;
const uint8_t     pca9685_sda_io           = GPIO_NUM_21;
const uint32_t    pca9685_i2c_freq_hz      = 100000;
const uint8_t     pca9685_i2c_address      = 0x40;
const i2c_port_t  pca9685_i2c_bus          = I2C_NUM_0;
const uint32_t    pca9685_osc_freq         = 25000000; /**< 25MHz internal osc */
const uint16_t    pca9685_pwm_resolution   = 4096;     /**< 12-bit resolution */
const uint16_t    pca9685_default_pwm_freq = 54;       /**< Measured servo freq */
const uint16_t    pca9685_max_pwm_value    = 4095;     /**< 0 to 4095 */
const uint16_t    pca9685_pwm_period_us    = 18519;    /**< 54Hz ≈ 18.519ms */
const char* const pca9685_tag              = "PCA9685";
const uint8_t     pca9685_step_size_deg    = 5;
const uint32_t    pca9685_step_delay_ms    = 20;
const float       pca9685_default_angle    = 90.0f;
const uint8_t     pca9685_max_retries      = 3;
const uint32_t    pca9685_retry_interval   = pdMS_TO_TICKS(100);
const uint32_t    pca9685_max_backoff      = pdMS_TO_TICKS(5 * 60 * 1000); /* 5 minutes */

/* Private Function Prototypes ************************************************/

static esp_err_t priv_pca9685_reset(void* const context);

/* Private Function Implementations *******************************************/

static esp_err_t pca9685_write_register(uint8_t i2c_addr, 
                                        uint8_t reg, 
                                        uint8_t value) 
{
  uint8_t   write_buf[2] = {reg, value};
  esp_err_t ret = i2c_master_write_to_device(pca9685_i2c_bus, 
                                             i2c_addr, 
                                             write_buf, 
                                             sizeof(write_buf), 
                                             pdMS_TO_TICKS(100));
  if (ret != ESP_OK) {
    log_error(pca9685_tag, 
              "Write Error", 
              "Failed to write register 0x%02X with value 0x%02X", 
              reg, 
              value);
  }
  return ret;
}

static esp_err_t pca9685_set_pwm_freq(uint8_t i2c_addr, uint16_t freq) 
{
  /* Calculate prescale value based on the formula from the datasheet */
  float   prescale_float = ((float)pca9685_osc_freq / (freq * pca9685_pwm_resolution)) - 1;
  uint8_t prescale       = (uint8_t)(prescale_float + 0.5f);

  /* Read current mode1 register */
  uint8_t   mode1;
  esp_err_t ret = pca9685_read_register(i2c_addr, k_pca9685_mode1_cmd, &mode1);
  if (ret != ESP_OK) {
    log_error(pca9685_tag, "Read Error", "Failed to read mode1 register");
    return ret;
  }

  /* Put the device to sleep (required to change prescale) */
  ret = pca9685_write_register(i2c_addr, 
                               k_pca9685_mode1_cmd, 
                               mode1 | k_pca9685_sleep_cmd);
  if (ret != ESP_OK) {
    log_error(pca9685_tag, "Write Error", "Failed to write sleep command");
    return ret;
  }

  /* Write prescale value */
  ret = pca9685_write_register(i2c_addr, k_pca9685_prescale_cmd, prescale);
  if (ret != ESP_OK) {
    log_error(pca9685_tag, "Write Error", "Failed to write prescale value");
    return ret;
  }

  /* Restore original mode1 value without sleep bit */
  ret = pca9685_write_register(i2c_addr, k_pca9685_mode1_cmd, mode1);
  if (ret != ESP_OK) {
    log_error(pca9685_tag, "Write Error", "Failed to restore mode1 value");
    return ret;
  }

  /* Wait for oscillator */
  vTaskDelay(pdMS_TO_TICKS(1));

  /* Set restart bit */
  ret = pca9685_write_register(i2c_addr, 
                               k_pca9685_mode1_cmd, mode1 | k_pca9685_restart_cmd);
  if (ret != ESP_OK) {
    log_error(pca9685_tag, "Write Error", "Failed to set restart bit");
    return ret;
  }

  return ESP_OK;
}

static esp_err_t pca9685_set_pwm(uint8_t  i2c_addr, 
                                 uint8_t  channel, 
                                 uint16_t on, 
                                 uint16_t off) 
{
  uint8_t   reg = k_pca9685_channel0_on_l_cmd + (channel * 4);
  esp_err_t ret;

  log_info(pca9685_tag, 
           "PWM Update", 
           "Setting channel %u: ON=%u, OFF=%u", 
           channel, 
           on, 
           off);

  /* Write ON time */
  ret = pca9685_write_register(i2c_addr, reg, on & 0xFF);
  if (ret != ESP_OK) {
    log_error(pca9685_tag, 
              "Write Error", 
              "Failed to write ON low byte for channel %u", 
              channel);
    return ret;
  }
  ret = pca9685_write_register(i2c_addr, reg + 1, (on >> 8) & 0xFF);
  if (ret != ESP_OK) {
    log_error(pca9685_tag, 
              "Write Error", 
              "Failed to write ON high byte for channel %u", 
              channel);
    return ret;
  }

  /* Write OFF time */
  ret = pca9685_write_register(i2c_addr, reg + 2, off & 0xFF);
  if (ret != ESP_OK) {
    log_error(pca9685_tag, 
              "Write Error", 
              "Failed to write OFF low byte for channel %u", 
              channel);
    return ret;
  }
  ret = pca9685_write_register(i2c_addr, reg + 3, (off >> 8) & 0xFF);
  if (ret != ESP_OK) {
    log_error(pca9685_tag, 
              "Write Error", 
              "Failed to write OFF high byte for channel %u", 
              channel);
    return ret;
  }

  return ESP_OK;
}

static uint16_t angle_to_pwm(float angle) 
{
  /* At 54Hz:
   * 1ms pulse = (4096 * 1ms) / 18.519ms ≈ 221 counts
   * 2ms pulse = (4096 * 2ms) / 18.519ms ≈ 442 counts */
  const uint16_t min_pulse = 221; /* 1ms at 54Hz */
  const uint16_t max_pulse = 442; /* 2ms at 54Hz */
  
  /* Constrain angle to 0-180 degrees */
  if (angle < 0.0f) {
    angle = 0.0f;
  }
  if (angle > 180.0f) {
    angle = 180.0f;
  }
  
  /* Linear interpolation */
  uint16_t pwm = min_pulse + (uint16_t)((max_pulse - min_pulse) * angle / 180.0f);
  
  /* Calculate actual pulse width in microseconds for debugging */
  float pulse_us = (float)pwm * (18519.0f / pca9685_pwm_resolution);
  
  log_info(pca9685_tag, 
           "Angle Convert", 
           "Angle %.1f° converted to PWM %u (%.2f ms pulse)", 
           angle, 
           pwm, 
           pulse_us / 1000.0f); /* Convert to milliseconds */
           
  return pwm;
}

/**
 * @brief Reset function for the PCA9685 controller
 * 
 * This function is called by the error handler when a reset is needed.
 * It performs a full reinitialization of the controller.
 * 
 * @param context Pointer to the pca9685_board_t structure
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t priv_pca9685_reset(void* const context)
{
  pca9685_board_t* board = (pca9685_board_t*)context;
  esp_err_t ret;

  if (!board) {
    log_error(pca9685_tag, "Reset Error", "Invalid board context");
    return ESP_ERR_INVALID_ARG;
  }

  /* Reset the PCA9685 by writing to the MODE1 register */
  ret = pca9685_write_register(board->i2c_address, 
                              k_pca9685_mode1_cmd, 
                              k_pca9685_restart_cmd);
  if (ret != ESP_OK) {
    board->state = k_pca9685_reset_error;
    log_error(pca9685_tag, "Reset Error", "Failed to reset PCA9685 controller");
    return ret;
  }
  vTaskDelay(pdMS_TO_TICKS(10));

  /* Set the PWM frequency */
  ret = pca9685_set_pwm_freq(board->i2c_address, pca9685_default_pwm_freq);
  if (ret != ESP_OK) {
    board->state = k_pca9685_communication_error;
    log_error(pca9685_tag, "Frequency Error", "Failed to set PWM frequency");
    return ret;
  }

  board->state = k_pca9685_ready;
  return ESP_OK;
}

/* Public Function Implementations *******************************************/

esp_err_t pca9685_init(pca9685_board_t** const controller_data, uint8_t num_boards) 
{
  if (controller_data == NULL || num_boards == 0) {
    log_error(pca9685_tag, 
              "Init Error", 
              "Invalid arguments: controller=%p, num_boards=%u", 
              (void*)controller_data, 
              num_boards);
    return ESP_ERR_INVALID_ARG;
  }

  /* Initialize I2C using common setup function */
  esp_err_t ret = common_setup_i2c(pca9685_i2c_bus,
                                  pca9685_scl_io,
                                  pca9685_sda_io,
                                  pca9685_i2c_freq_hz,
                                  pca9685_tag);
  if (ret != ESP_OK) {
    log_error(pca9685_tag, "I2C Error", "Failed to set up I2C bus: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Create and initialize the board list */
  pca9685_board_t* head    = NULL;
  pca9685_board_t* current = NULL;

  for (uint8_t i = 0; i < num_boards; i++) {
    pca9685_board_t *board = calloc(1, sizeof(pca9685_board_t));
    if (board == NULL) {
      log_error(pca9685_tag, 
                "Memory Error", 
                "Failed to allocate memory for board %u", 
                i);
      /* Clean up on allocation failure */
      while (head != NULL) {
        current = head;
        head    = head->next;
        free(current);
      }
      return ESP_ERR_NO_MEM;
    }

    /* Initialize board structure */
    board->i2c_address = pca9685_i2c_address + i;
    board->i2c_bus     = pca9685_i2c_bus;
    board->board_id    = i;
    board->num_boards  = num_boards;
    board->state       = k_pca9685_uninitialized;
    board->next        = NULL;

    /* Initialize error handler */
    char tag_with_id[32];
    snprintf(tag_with_id, sizeof(tag_with_id), "%s Board %u", pca9685_tag, i);
    error_handler_init(&(board->error_handler),
                      tag_with_id,
                      pca9685_max_retries,
                      pca9685_retry_interval,
                      pca9685_max_backoff,
                      priv_pca9685_reset,
                      board,
                      pca9685_retry_interval,
                      pca9685_max_backoff);

    /* Initialize motors array */
    for (int j = 0; j < PCA9685_MOTORS_PER_BOARD; j++) {
      board->motors[j].pos_deg  = pca9685_default_angle;
      board->motors[j].board_id = i;
      board->motors[j].motor_id = j;
    }

    /* Add to linked list */
    if (head == NULL) {
      head    = board;
      current = board;
    } else {
      current->next = board;
      current       = board;
    }

    /* Initialize PCA9685 hardware */
    /* Reset the device */
    ret = pca9685_write_register(board->i2c_address, 
                                 k_pca9685_mode1_cmd, 
                                 k_pca9685_restart_cmd);
    if (ret != ESP_OK) {
      log_error(pca9685_tag, "Reset Error", "Failed to reset board %u", i);
      board->state = k_pca9685_reset_error;
      error_handler_record_status(&(board->error_handler), ret);
      continue;
    }

    /* Set frequency */
    ret = pca9685_set_pwm_freq(board->i2c_address, pca9685_default_pwm_freq);
    if (ret != ESP_OK) {
      log_error(pca9685_tag, 
                "Freq Error", 
                "Failed to set PWM frequency for board %u", 
                i);
      board->state = k_pca9685_communication_error;
      error_handler_record_status(&(board->error_handler), ret);
      continue;
    }

    /* Configure for normal operation */
    ret = pca9685_write_register(board->i2c_address, 
                                 k_pca9685_mode2_cmd, 
                                 k_pca9685_output_logic_mode | k_pca9685_output_change_stop_cmd);
    if (ret != ESP_OK) {
      log_error(pca9685_tag, 
                "Config Error", 
                "Failed to configure output mode for board %u", 
                i);
      board->state = k_pca9685_communication_error;
      error_handler_record_status(&(board->error_handler), ret);
      continue;
    }

    board->state = k_pca9685_ready;
    log_info(pca9685_tag, 
             "Board Init", 
             "Initialized board %u at address 0x%02X", 
             i, 
             board->i2c_address);

    /* Set all motors to their default angle */
    uint16_t default_pwm = angle_to_pwm(pca9685_default_angle);
    for (int j = 0; j < PCA9685_MOTORS_PER_BOARD; j++) {
      ret = pca9685_set_pwm(board->i2c_address, j, 0, default_pwm);
      if (ret != ESP_OK) {
        log_warn(pca9685_tag, 
                 "Motor Init", 
                 "Failed to set default angle for motor %u on board %u", 
                 j, 
                 i);
        continue;
      }
    }
  }

  *controller_data = head;
  return ESP_OK;
}

esp_err_t pca9685_set_angle(const pca9685_board_t* const controller_data, 
                            uint16_t                     motor_mask,
                            uint8_t                      board_id, 
                            float                        target_angle) 
{
  if (controller_data == NULL || target_angle < 0.0f || target_angle > 180.0f) {
    log_error(pca9685_tag, 
              "Param Error", 
              "Invalid arguments: controller=%p, angle=%.2f", 
              (void*)controller_data, 
              target_angle);
    return ESP_ERR_INVALID_ARG;
  }

  /* Find the target board */
  pca9685_board_t* board = (pca9685_board_t*)controller_data;
  while (board != NULL && board->board_id != board_id) {
    board = board->next;
  }

  if (board == NULL || board->state != k_pca9685_ready) {
    log_error(pca9685_tag, 
              "Board Error", 
              "Board %u not found or not ready (state=%u)", 
              board_id, 
              board ? board->state : -1);
    return ESP_FAIL;
  }

  /* Convert angle to PWM value */
  uint16_t target_pwm = angle_to_pwm(target_angle);
  esp_err_t ret = ESP_OK;

  /* Set the angle for each motor in the mask */
  for (int i = 0; i < PCA9685_MOTORS_PER_BOARD; i++) {
    if (motor_mask & (1 << i)) {
      float current_angle = board->motors[i].pos_deg;
      
      /* If the angle change is large, move gradually to avoid jerky motion */
      if (fabs(target_angle - current_angle) > pca9685_step_size_deg) {
        ret = pca9685_move_gradually(board, i, current_angle, target_angle);
      } else {
        ret = pca9685_set_pwm(board->i2c_address, i, 0, target_pwm);
      }
      
      if (ret != ESP_OK) {
        log_error(pca9685_tag, 
                  "Motor Error", 
                  "Failed to set angle for motor %d on board %u", 
                  i, 
                  board_id);
        board->state = k_pca9685_communication_error;
        error_handler_record_status(&(board->error_handler), ret);
        return ret;
      }
      
      /* Update the stored position */
      board->motors[i].pos_deg = target_angle;
    }
  }

  return ESP_OK;
}

esp_err_t pca9685_cleanup(pca9685_board_t** const controller_data)
{
  if (controller_data == NULL || *controller_data == NULL) {
    log_error(pca9685_tag, 
              "Cleanup Error", 
              "Invalid controller data pointer");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(pca9685_tag, "Cleanup Start", "Beginning PCA9685 controller cleanup");
  esp_err_t ret = ESP_OK;

  /* Put all boards in sleep mode and reset outputs */
  pca9685_board_t* current = *controller_data;
  while (current != NULL) {
    /* Put board in sleep mode */
    esp_err_t temp_ret = pca9685_write_register(current->i2c_address, 
                                                k_pca9685_mode1_cmd, 
                                                k_pca9685_sleep_cmd);
    if (temp_ret != ESP_OK) {
      log_warn(pca9685_tag, 
               "Sleep Warning", 
               "Failed to put board %u in sleep mode: %s", 
               current->board_id, 
               esp_err_to_name(temp_ret));
      ret = temp_ret;
    }

    /* Reset all outputs to 0 */
    for (int i = 0; i < PCA9685_MOTORS_PER_BOARD; i++) {
      temp_ret = pca9685_set_pwm(current->i2c_address, i, 0, 0);
      if (temp_ret != ESP_OK) {
        log_warn(pca9685_tag, 
                 "Reset Warning", 
                 "Failed to reset output %d on board %u: %s", 
                 i, 
                 current->board_id, 
                 esp_err_to_name(temp_ret));
        ret = temp_ret;
      }
    }

    /* Clean up error handler */
    temp_ret = error_handler_cleanup(&(current->error_handler));
    if (temp_ret != ESP_OK) {
      log_warn(pca9685_tag,
               "Error Handler Warning",
               "Failed to clean up error handler for board %u: %s",
               current->board_id,
               esp_err_to_name(temp_ret));
      ret = temp_ret;
    }

    current = current->next;
  }

  /* Clean up GPIO pins using common cleanup */
  uint64_t pin_mask = (1ULL << pca9685_scl_io) | (1ULL << pca9685_sda_io);
  esp_err_t temp_ret = common_cleanup_gpio(pin_mask, pca9685_tag);
  if (temp_ret != ESP_OK) {
    log_warn(pca9685_tag, 
             "GPIO Warning", 
             "Failed to clean up GPIO pins: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }

  /* Clean up I2C resources using common cleanup */
  temp_ret = common_cleanup_i2c(pca9685_i2c_bus, pca9685_scl_io, pca9685_sda_io, pca9685_tag);
  if (temp_ret != ESP_OK) {
    log_warn(pca9685_tag, 
             "I2C Warning", 
             "Failed to clean up I2C resources: %s", 
             esp_err_to_name(temp_ret));
    ret = temp_ret;
  }

  /* Free allocated memory and reset pointer */
  current = *controller_data;
  while (current != NULL) {
    pca9685_board_t* next = current->next;
    free(current);
    current = next;
  }
  *controller_data = NULL;

  if (ret == ESP_OK) {
    log_info(pca9685_tag, 
             "Cleanup Complete", 
             "PCA9685 controller resources released successfully");
  } else {
    log_warn(pca9685_tag, 
             "Cleanup Warning", 
             "PCA9685 cleanup completed with some warnings");
  }

  return ret;
}
