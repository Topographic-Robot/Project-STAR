/* main/include/tasks/gait_tasks.c */

#include "gait_tasks.h"
#include <math.h>
#include "pca9685_hal.h"
#include "esp_log.h"

/* Constants ******************************************************************/

const char *gait_tag = "Gait Tasks";

/* Private Functions (Static) *************************************************/

/**
 * @brief Set a relative angle for a motor based on its joint type.
 *
 * Converts a relative angle (e.g., -30°, +30°) to an absolute angle (0°–180°) 
 * for the PCA9685 servo and validates the angle based on joint constraints.
 *
 * @param[in] pwm_controller Pointer to the PCA9685 board controller.
 * @param[in] motor_mask Mask indicating which motor(s) to control (bitmask).
 * @param[in] board_id The ID of the PCA9685 board controlling the motor(s).
 * @param[in] joint_type The type of joint (hip, knee, or tibia).
 * @param[in] relative_angle The desired angle relative to 90°.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t priv_set_relative_angle(pca9685_board_t *pwm_controller, uint16_t motor_mask, 
                                  uint8_t board_id, joint_type_t joint_type, 
                                  float relative_angle) 
{
  float absolute_angle = 90.0f; /* Default neutral position */
  float angle_min      = 0.0f, angle_max = 180.0f;

  /* Determine angle constraints based on joint type */
  switch (joint_type) {
    case k_hip:
      angle_min = 90.0f + hip_angle_from_90_min;
      angle_max = 90.0f + hip_angle_from_90_max;
      break;
    case k_knee:
      angle_min = 90.0f + knee_angle_from_90_min;
      angle_max = 90.0f + knee_angle_from_90_max;
      break;
    case k_tibia:
      angle_min = 90.0f + tibia_angle_from_90_min;
      angle_max = 90.0f + tibia_angle_from_90_max;
      break;
    default:
      ESP_LOGE("SetAngle", "Invalid joint type: %d", joint_type);
      return ESP_ERR_INVALID_ARG;
  }

  /* Calculate the absolute angle */
  absolute_angle = 90.0f + relative_angle;

  /* Clamp the angle to the joint's range */
  if (absolute_angle < angle_min) {
    ESP_LOGW("SetAngle", "Clamping angle to min %.2f° for joint type %d", angle_min, joint_type);
    absolute_angle = angle_min;
  } else if (absolute_angle > angle_max) {
    ESP_LOGW("SetAngle", "Clamping angle to max %.2f° for joint type %d", angle_max, joint_type);
    absolute_angle = angle_max;
  }

  /* Set the angle on the PCA9685 */
  return pca9685_set_angle(pwm_controller, motor_mask, board_id, absolute_angle);
}

/**
 * @brief Calculate the step distance based on the angles of the hip, knee, and tibia joints.
 *
 * This function computes the horizontal stride distance of a leg based on the input angles
 * for the hip, knee, and tibia joints. It also calculates the vertical clearance for diagnostic
 * purposes. Angles must be within their respective valid ranges (ALL RELATIVE ANGLES).
 *
 * @param[in] hip_angle_deg   Angle of the hip joint in degrees. Must be within
 *                            the range [hip_angle_from_90_min, hip_angle_from_90_max].
 * @param[in] knee_angle_deg  Angle of the knee joint in degrees. Must be within 
 *                            the range [knee_angle_from_90_min, knee_angle_from_90_max].
 * @param[in] tibia_angle_deg Angle of the tibia joint in degrees. Must be within 
 *                            the range [tibia_angle_from_90_min, tibia_angle_from_90_max].
 *
 * @return Horizontal stride distance in centimeters if all angles are valid; -1.0f otherwise.
 *
 * @note The femur_length_cm and tibia_length_cm constants must be defined globally.
 *       The gait_tag must be used for logging purposes.
 */
float priv_calculate_step_distance(float hip_angle_deg, float knee_angle_deg, 
                                   float tibia_angle_deg)
{
  /* TODO: Verify this for accuracy */

  /* Validate input parameters */
  if (hip_angle_deg < hip_angle_from_90_min || hip_angle_deg > hip_angle_from_90_max) {
    ESP_LOGE(gait_tag, "Invalid hip angle: %.2f°. Must be in range [%.2f°, %.2f°].",
        hip_angle_deg, hip_angle_from_90_min, hip_angle_from_90_max);
    return -1.0f;
  }

  if (knee_angle_deg < knee_angle_from_90_min || knee_angle_deg > knee_angle_from_90_max) {
    ESP_LOGE(gait_tag, "Invalid knee angle: %.2f°. Must be in range [%.2f°, %.2f°].",
        knee_angle_deg, knee_angle_from_90_min, knee_angle_from_90_max);
    return -1.0f;
  }

  if (tibia_angle_deg < tibia_angle_from_90_min || tibia_angle_deg > tibia_angle_from_90_max) {
    ESP_LOGE(gait_tag, "Invalid tibia angle: %.2f°. Must be in range [%.2f°, %.2f°].",
        tibia_angle_deg, tibia_angle_from_90_min, tibia_angle_from_90_max);
    return -1.0f;
  }

  /* Convert angles to radians */
  float hip_angle_rad   = hip_angle_deg * (M_PI / 180.0f);
  float knee_angle_rad  = knee_angle_deg * (M_PI / 180.0f);
  float tibia_angle_rad = tibia_angle_deg * (M_PI / 180.0f);

  /* Calculate horizontal stride */
  float horizontal_stride = 2.0f * femur_length_cm * sinf(hip_angle_rad / 2.0f) +
                            tibia_length_cm * sinf(tibia_angle_rad);

  /* Calculate vertical clearance (for diagnostics) */
  float vertical_clearance = femur_length_cm * cosf(knee_angle_rad) +
                             tibia_length_cm * cosf(tibia_angle_rad);

  ESP_LOGI(gait_tag, "Stride: %.2f cm, Clearance: %.2f cm", 
      horizontal_stride, vertical_clearance);

  return horizontal_stride;
}

/**
 * @brief Process a motor mask in chunks, ensuring the maximum active servos are not exceeded.
 *
 * This function divides a 16-bit motor mask into smaller chunks, each containing a number
 * of active motors less than or equal to the specified max_active_servos. It then applies
 * the specified target angle to the motors of the selected joint type for each chunk, with
 * a delay between processing to minimize current draw.
 *
 * @param[in] pwm_controller    Pointer to the PCA9685 board controller object.
 * @param[in] mask              16-bit mask representing the motors to be processed.
 * @param[in] max_active_servos Maximum number of active servos to process at one time.
 * @param[in] relative_angle    Relative angle in degrees to set for the specified joint type.
 *
 * @return ESP_OK if all chunks are processed successfully; an appropriate esp_err_t code otherwise.
 *
 * @note The joint_type parameter must correspond to an enumerated value defined in joint_type_t.
 *       The function delays execution using vTaskDelay between chunk processing to reduce current draw.
 */
esp_err_t priv_process_mask_in_chunks(pca9685_board_t *pwm_controller, uint16_t mask,
                                      float relative_angle)
{
  /* TODO: Implement this (this needs to make sure to handle the hip motors first 
   * since they provide clearance for the other motors */
  return ESP_OK;
}

/* Public Functions ***********************************************************/

esp_err_t tripod_gait(pca9685_board_t *pwm_controller, float heading, 
    uint16_t distance)
{
  ESP_LOGI(gait_tag, "Starting tripod gait: heading=%.2f, distance=%ucm", 
      heading, distance);

  /* TODO: Implement this */
  return ESP_OK;
}

esp_err_t wave_gait(pca9685_board_t *pwm_controller, float heading, 
    uint16_t distance)
{
  ESP_LOGI(gait_tag, "Starting wave gait: heading=%.2f, distance=%ucm", 
      heading, distance);

  /* TODO: Implement this */
  return ESP_OK;
}

esp_err_t ripple_gait(pca9685_board_t *pwm_controller, float heading, 
    uint16_t distance)
{
  ESP_LOGI(gait_tag, "Starting ripple gait: heading=%.2f, distance=%ucm", 
      heading, distance);

  /* TODO: Implement this */
  return ESP_OK;
}

esp_err_t quadruped_gait(pca9685_board_t *pwm_controller, float heading, 
    uint16_t distance)
{
  ESP_LOGI(gait_tag, "Starting quadruped gait: heading=%.2f, distance=%ucm", 
      heading, distance);

  /* TODO: Implement this */
  return ESP_OK;
}
