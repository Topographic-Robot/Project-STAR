/* main/include/hexapod_geometry.h */

#ifndef TOPOROBO_HEXAPOD_GEOMETRY
#define TOPOROBO_HEXAPOD_GEOMETRY

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ec11_hal.h"

/* Constants ******************************************************************/

extern const float   hip_angle_from_90_min;   /**< Minimum allowable angle from 90degs for the hip joint in degrees. */
extern const float   hip_angle_from_90_max;   /**< Maximum allowable angle from 90degs for the hip joint in degrees. */
extern const float   knee_angle_from_90_min;  /**< Minimum allowable angle from 90degs for the knee joint in degrees. */
extern const float   knee_angle_from_90_max;  /**< Maximum allowable angle from 90degs for the knee joint in degrees. */
extern const float   tibia_angle_from_90_min; /**< Minimum allowable angle from 90degs for the tibia joint in degrees. */
extern const float   tibia_angle_from_90_max; /**< Maximum allowable angle from 90degs for the tibia joint in degrees. */
extern const float   hip_length_cm;           /**< Length of the hip segment in centimeters (from the base to the femur). */
extern const float   femur_length_cm;         /**< Length of the femur segment in centimeters (from the femur to the tibia). */
extern const float   tibia_length_cm;         /**< Length of the tibia segment in centimeters (from the tibia to the ground). */
extern const uint8_t max_active_servos;       /**< Limit on simultaneously active servos to manage power draw. */

/* Enums **********************************************************************/

/**
 * @brief Defines the joint types in the hexapod robot.
 *
 * Used to specify the joint controlled by a motor.
 */
typedef enum : uint8_t {
  k_hip,   /**< The hip joint. */
  k_knee,  /**< The knee joint. */
  k_tibia, /**< The tibia joint. */
} joint_type_t;

/* Structs ********************************************************************/

/**
 * @brief Represents a motor associated with a specific joint.
 *
 * Contains information about a motor's type, current position, and identification
 * on the PCA9685 servo controller boards.
 */
typedef struct {
  joint_type_t joint_type; /**< Type of joint this motor controls (hip, knee, or tibia). */
  float        pos_deg;    /**< Current position of the motor in degrees (range: 0 to 180). */
  uint8_t      board_id;   /**< ID of the PCA9685 board (e.g., 0 or 1). */
  uint8_t      motor_id;   /**< ID of the motor on the board (range: 0 to 15). */
  ec11_data_t  ec11_data; /**< Data for the EC11 encoder (if applicable). */
} motor_t;

/**
 * @brief Represents the configuration of a single leg in the hexapod robot.
 *
 * Contains the leg ID and pointers to the motor configurations for the hip,
 * knee, and tibia joints.
 */
typedef struct {
  uint8_t  id;          /**< The unique ID of the leg (range: 0 to 5). */
  motor_t *hip_motor;   /**< Pointer to the hip motor configuration. */
  motor_t *knee_motor;  /**< Pointer to the knee motor configuration. */
  motor_t *tibia_motor; /**< Pointer to the tibia motor configuration. */
} leg_t;

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_HEXAPOD_GEOMETRY */

