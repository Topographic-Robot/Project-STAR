/* main/include/tasks/gait_tasks.c */

#include "gait_tasks.h"
#include "pca9685_hal.h"
#include "esp_log.h"

/* Constants ******************************************************************/

const char   *gait_tag          = "Gait Tasks";
const uint8_t max_active_servos = 3;

/* Public Functions ***********************************************************/

esp_err_t tripod_gait(pca9685_board_t *pwm_controller, float heading, 
                      float distance)
{
  ESP_LOGI(gait_tag, "Starting tripod gait: heading=%.2f, distance=%.2f", 
           heading, distance);

  return ESP_OK;
}

esp_err_t wave_gait(pca9685_board_t *pwm_controller, float heading, 
                    float distance)
{
  ESP_LOGI(gait_tag, "Starting wave gait: heading=%.2f, distance=%.2f", 
           heading, distance);

  return ESP_OK;
}

esp_err_t ripple_gait(pca9685_board_t *pwm_controller, float heading, 
                      float distance)
{
  ESP_LOGI(gait_tag, "Starting ripple gait: heading=%.2f, distance=%.2f", 
           heading, distance);

  return ESP_OK;
}

esp_err_t quadruped_gait(pca9685_board_t *pwm_controller, float heading, 
                         float distance)
{
  ESP_LOGI(gait_tag, "Starting quadruped gait: heading=%.2f, distance=%.2f", 
           heading, distance);

  return ESP_OK;
}
