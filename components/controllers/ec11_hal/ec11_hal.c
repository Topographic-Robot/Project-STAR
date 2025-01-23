
#include "ec11_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Constants ******************************************************************/

const char    *ec11_tag      = "EC11";
const uint32_t poll_delay_ms = 10;

/* Public Functions ***********************************************************/

esp_err_t ec11_init(ec11_data_t *encoder)
{
  if (encoder == NULL) {
    ESP_LOGE(ec11_tag, "Invalid encoder pointer");
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(ec11_tag, "Initializing EC11 encoder");

  /* Configure GPIO pins */
  gpio_config_t io_conf = {
    .pin_bit_mask = ((uint64_t)1 << encoder->pin_a) | 
                    ((uint64_t)1 << encoder->pin_b) | 
                    ((uint64_t)1 << encoder->pin_btn),
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE
  };

  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(ec11_tag, "Failed to configure GPIO pins (err=0x%x)", ret);
    encoder->state = k_ec11_error;
    return ret;
  }

  /* Initialize encoder state */
  encoder->position       = 0;
  encoder->button_pressed = false;
  encoder->prev_state     = (gpio_get_level(encoder->pin_a) << 1) | 
                            gpio_get_level(encoder->pin_b);
  encoder->state          = k_ec11_ready;

  ESP_LOGI(ec11_tag, "EC11 encoder initialized successfully");
  return ESP_OK;
}

void ec11_register_callback(ec11_data_t *encoder, 
                            ec11_callback_t callback, 
                            void *board_ptr, 
                            uint16_t motor_mask)
{
  if (encoder == NULL || callback == NULL || board_ptr == NULL) {
    ESP_LOGE(ec11_tag, "Invalid encoder or callback pointer");
    return;
  }

  encoder->callback   = callback;
  encoder->board_ptr  = board_ptr;
  encoder->motor_mask = motor_mask;
}

void ec11_task(void *arg)
{
  ec11_data_t *encoder = (ec11_data_t *)arg;
  
  if (encoder == NULL || encoder->state == k_ec11_uninitialized) {
    ESP_LOGE(ec11_tag, "Invalid encoder data");
    return;
  }

  while (1) {
    /* Read the current state of the encoder */
    int current_state = (gpio_get_level(encoder->pin_a) << 1) | 
                        gpio_get_level(encoder->pin_b);

    /* Only process if the state has changed */
    if (current_state != encoder->prev_state) {
      /* Determine the direction of rotation */
      if ((encoder->prev_state == k_ec11_state_00 && current_state == k_ec11_state_01) ||
          (encoder->prev_state == k_ec11_state_01 && current_state == k_ec11_state_11) ||
          (encoder->prev_state == k_ec11_state_11 && current_state == k_ec11_state_10) ||
          (encoder->prev_state == k_ec11_state_10 && current_state == k_ec11_state_00)) {
        encoder->position++;
        ESP_LOGI(ec11_tag, "Clockwise (pos: %ld)", encoder->position);
        if (encoder->callback) {
          encoder->callback(k_ec11_cw, encoder->board_ptr, encoder->motor_mask);
        }
      } else if ((encoder->prev_state == k_ec11_state_00 && current_state == k_ec11_state_10) ||
                 (encoder->prev_state == k_ec11_state_10 && current_state == k_ec11_state_11) ||
                 (encoder->prev_state == k_ec11_state_11 && current_state == k_ec11_state_01) ||
                 (encoder->prev_state == k_ec11_state_01 && current_state == k_ec11_state_00)) {
        encoder->position--;
        ESP_LOGI(ec11_tag, "Counterclockwise (pos: %ld)", encoder->position);
        if (encoder->callback) {
          encoder->callback(k_ec11_ccw, encoder->board_ptr, encoder->motor_mask);
        }
      }
      /* Update the previous state */
      encoder->prev_state = current_state;
    }

    /* Read and update button state */
    bool current_button = gpio_get_level(encoder->pin_btn) == 0; /* Active low */
    if (current_button != encoder->button_pressed) {
      encoder->button_pressed = current_button;
      ESP_LOGI(ec11_tag, "Button %s", current_button ? "pressed" : "released");
      if (encoder->callback) {
        encoder->callback(current_button ? k_ec11_btn_press : k_ec11_btn_release, 
                          encoder->board_ptr, 
                          encoder->motor_mask);
      }
    }

    /* Delay to avoid busy-waiting */
    vTaskDelay(pdMS_TO_TICKS(poll_delay_ms));
  }
}
