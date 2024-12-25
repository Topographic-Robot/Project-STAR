/* main/include/tasks/motor_tasks.c */

#include "motor_tasks.h"
#include "pca9685_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Constants ******************************************************************/

const uint8_t num_pca9685_boards = 1;
const char   *motor_tag          = "Motor Tasks";

/* Public Functions ***********************************************************/

esp_err_t motors_init(pca9685_board_t **pwm_controller)
{
  esp_err_t ret = pca9685_init(pwm_controller, num_pca9685_boards);
  if (ret != ESP_OK) {
    ESP_LOGE(motor_tag, "Failed to initialize all PWM boards.");
    return ret;
  }
  ESP_LOGI(motor_tag, "Initialized all PWM boards with default values.");

  /* TODO: Map the motors with their identifications e.g hip/femur/tibia */

  return ESP_OK;
}

esp_err_t motor_tasks_start(pca9685_board_t *pwm_controller)
{
  ESP_LOGI(motor_tag, "Started priv_testing task.");

  /* Example Usage -- we still need to implement this so it can be used
   * either by like a controller, webui/app (webserver), etc
   *
   * xTaskCreate(...)
   * func(...):
   *  while (1) {
   *    func *gait_type = select_gait_func(some_input_val);
   *    gait_type();
   *    xTaskDelay(...);
   *  }
   */

  return ESP_OK;
}
