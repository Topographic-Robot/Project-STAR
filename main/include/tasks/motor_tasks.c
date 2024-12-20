/* main/include/tasks/motor_tasks.c */

#include "motor_tasks.h"
#include "pca9685_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Constants ******************************************************************/

const uint8_t num_pca9685_boards = 1;
const char   *motor_tag          = "Motor Tasks";

/* Private Functions **********************************************************/

static void priv_testing(void *arg)
{
  ESP_LOGI(motor_tag, "priv_testing task running...");

  pca9685_board_t *pwm_controller_linked_list = (pca9685_board_t *)arg;

  /* Simulate task operation */
  while (1) {
    for (float f = 0.0f; f <= 180.0; f += 10) {
      pca9685_set_angle(pwm_controller_linked_list, 0xFFFF, 0, f);
      ESP_LOGI(motor_tag, "Setting motors to %f", f);
      vTaskDelay(pdMS_TO_TICKS(1000)); /* Delay for 1 second */
    }    
  }
}

/* Public Functions ***********************************************************/

esp_err_t motors_init(pca9685_board_t **pwm_controller_linked_list)
{
  esp_err_t ret = pca9685_init(pwm_controller_linked_list, num_pca9685_boards);
  if (ret != ESP_OK) {
    ESP_LOGE(motor_tag, "Failed to initialize all PWM boards.");
    return ret;
  }
  ESP_LOGI(motor_tag, "Initialized all PWM boards.");
  return ESP_OK;
}

esp_err_t motor_tasks_start(pca9685_board_t *pwm_controller_linked_list)
{
  BaseType_t task_created;

  /* Start priv_testing task */
  task_created = xTaskCreate(
    priv_testing,
    "TestingTask",
    2048,
    (void *)pwm_controller_linked_list,
    5,
    NULL
  );

  if (task_created != pdPASS) {
    ESP_LOGE(motor_tag, "Failed to create priv_testing task.");
    return ESP_FAIL;
  }

  ESP_LOGI(motor_tag, "Started priv_testing task.");
  return ESP_OK;
}
