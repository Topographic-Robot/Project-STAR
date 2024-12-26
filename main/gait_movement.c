/* main/gait_movement.c */

#include "gait_movement.h"
#include <math.h>
#include "hexapod_geometry.h"
#include "pca9685_hal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Globals (Static) ***********************************************************/

static leg_t s_legs[6] = {};

/* Constants ******************************************************************/

const char *gait_tag = "Gait Movement";

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
      ESP_LOGE(gait_tag, "Invalid joint type: %d", joint_type);
      return ESP_ERR_INVALID_ARG;
  }

  /* Calculate the absolute angle */
  absolute_angle = 90.0f + relative_angle;

  /* Clamp the angle to the joint's range */
  if (absolute_angle < angle_min) {
    ESP_LOGW(gait_tag, "Clamping angle to min %.2f° for joint type %d", angle_min, joint_type);
    absolute_angle = angle_min;
  } else if (absolute_angle > angle_max) {
    ESP_LOGW(gait_tag, "Clamping angle to max %.2f° for joint type %d", angle_max, joint_type);
    absolute_angle = angle_max;
  }

  /* Set the angle on the PCA9685 */
  return pca9685_set_angle(pwm_controller, motor_mask, board_id, absolute_angle);
}

/**
 * @brief Assigns motor details to a given motor struct.
 *
 * This helper function assigns the joint type, board ID, motor ID, and initializes
 * the motor's position to 0.0 degrees. It also ensures that motor indices wrap
 * correctly across multiple PCA9685 boards.
 *
 * @param[in,out] motor         Pointer to the motor_t structure to be configured.
 * @param[in]     joint_type    The type of joint (hip, knee, tibia).
 * @param[in]     board         Pointer to the current PCA9685 board.
 * @param[in,out] motor_index   Pointer to the motor index on the current board.
 * @param[in,out] board_id      Pointer to the ID of the current PCA9685 board.
 *
 * @note This function modifies motor_index and board_id if motor indices exceed
 *       the board's capacity (16 motors).
 *
 * @return ESP_OK on success, or ESP_FAIL if a new board is required but not available.
 */
static esp_err_t priv_assign_motor(motor_t *motor, joint_type_t joint_type, 
                                   pca9685_board_t **board, uint8_t *motor_index, 
                                   uint8_t *board_id)
{
  if (*motor_index >= 16) {
    if ((*board)->next == NULL) {
      ESP_LOGE(gait_tag, "Insufficient PCA9685 boards for initializing motors.");
      return ESP_FAIL;
    }
    *board       = (*board)->next;
    *motor_index = 0;
    (*board_id)++;
  }

  motor->joint_type = joint_type;
  motor->pos_deg    = 0.0f;
  motor->board_id   = *board_id;
  motor->motor_id   = *motor_index;

  (*motor_index)++;
  return ESP_OK;
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
 * @brief Helper function to extract a chunk of active motors from a mask.
 *
 * This function extracts the next chunk of motors from the mask, ensuring that
 * the number of active motors in the chunk does not exceed the specified limit.
 *
 * @param[in]  mask              The full 16-bit mask of motors.
 * @param[in]  max_active_servos Maximum number of active motors per chunk.
 * @param[out] chunk             The resulting chunk of motors.
 *
 * @return The updated mask after removing the processed chunk.
 */
static uint16_t priv_extract_chunk(uint16_t mask, uint8_t max_active_servos, 
                                   uint16_t *chunk) 
{
  uint8_t active_count = 0;
  *chunk               = 0;

  for (uint8_t i = 0; i < 16; ++i) {
    if ((mask & (1 << i)) && active_count < max_active_servos) {
      *chunk |= (1 << i);
      mask   &= ~(1 << i); /* Clear the bit from the original mask */
      active_count++;
    }
    if (active_count == max_active_servos) {
      break;
    }
  }
  return mask;
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
  if (pwm_controller == NULL) {
    ESP_LOGE(gait_tag, "PCA9685 controller pointer is NULL.");
    return ESP_ERR_INVALID_ARG;
  }

  uint16_t  chunk          = 0;
  uint16_t  remaining_mask = mask;
  esp_err_t ret            = ESP_OK;

  /* Process hip motors first for clearance */
  uint16_t hip_mask = mask;
  for (uint8_t i = 0; i < 16; ++i) {
    if (pwm_controller->motors[i].joint_type != k_hip) {
      hip_mask &= ~(1 << i); /* Remove non-hip motors from the hip mask */
    }
  }

  while (hip_mask) {
    hip_mask = priv_extract_chunk(hip_mask, max_active_servos, &chunk);

    /* Apply relative angle to hip motors in the current chunk */
    ret = pca9685_set_angle(pwm_controller, chunk, pwm_controller->board_id, relative_angle);
    if (ret != ESP_OK) {
      ESP_LOGE(gait_tag, "Failed to set angles for hip motors. Error: %d", ret);
      return ret;
    }

    /* Update motor positions */
    for (uint8_t i = 0; i < 16; ++i) {
      if (chunk & (1 << i)) {
        pwm_controller->motors[i].pos_deg += relative_angle;
      }
    }

    /* Delay to reduce current draw */
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  /* Remove processed hip motors from the remaining mask */
  remaining_mask &= ~hip_mask;

  /* Process remaining motors after hip motors */
  while (remaining_mask) {
    remaining_mask = priv_extract_chunk(remaining_mask, max_active_servos, &chunk);

    /* Apply relative angle to motors in the current chunk */
    ret = pca9685_set_angle(pwm_controller, chunk, pwm_controller->board_id, relative_angle);
    if (ret != ESP_OK) {
      ESP_LOGE(gait_tag, "Failed to set angles for motors. Error: %d", ret);
      return ret;
    }

    /* Update motor positions */
    for (uint8_t i = 0; i < 16; ++i) {
      if (chunk & (1 << i)) {
        pwm_controller->motors[i].pos_deg += relative_angle;
      }
    }

    /* Delay to reduce current draw */
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  ESP_LOGI(gait_tag, "All motors processed successfully.");
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

esp_err_t gait_init(pca9685_board_t *pwm_controller)
{
  if (pwm_controller == NULL) {
    ESP_LOGE(gait_tag, "PCA9685 controller pointer is NULL.");
    return ESP_ERR_INVALID_ARG;
  }

  /* Initialize leg configurations */
  uint8_t          motor_index = 0;
  uint8_t          board_id    = 0;
  pca9685_board_t *board       = pwm_controller;

  /* Map motors to s_legs and joints */
  for (uint8_t leg_id = 0; leg_id < 6; ++leg_id) {
    s_legs[leg_id].id = leg_id;

    /* Assign motors to each joint */
    esp_err_t ret1, ret2, ret3;
    s_legs[leg_id].hip_motor   = &(board->motors[motor_index]);
    s_legs[leg_id].knee_motor  = &(board->motors[motor_index]);
    s_legs[leg_id].tibia_motor = &(board->motors[motor_index]);

    ret1 = priv_assign_motor(s_legs[leg_id].hip_motor, k_hip, &board, &motor_index,
                             &board_id);
    ret2 = priv_assign_motor(s_legs[leg_id].knee_motor, k_knee, &board, &motor_index,
                             &board_id);
    ret3 = priv_assign_motor(s_legs[leg_id].tibia_motor, k_tibia, &board, &motor_index,
                             &board_id);

    if (ret1 != ESP_OK || ret2 != ESP_OK || ret3 != ESP_OK) {
      ESP_LOGE(gait_tag, "Failed to initialize motors for leg %u.", leg_id);
      return ESP_FAIL;
    }
  }

  ESP_LOGI(gait_tag, "Legs initialized successfully.");
  return ESP_OK;
}
