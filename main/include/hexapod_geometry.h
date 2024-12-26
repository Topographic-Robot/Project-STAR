/* main/include/hexapod_geometry.h */

#ifndef TOPOROBO_HEXAPOD_GEOMETRY
#define TOPOROBO_HEXAPOD_GEOMETRY

#include <stdint.h>

/* Constants ******************************************************************/

/** 
 * @brief Minimum and maximum angle from 90degs for the hip joint in degrees. 
 */
extern const float hip_angle_from_90_min; /**< Minimum allowable angle from 90degs for the hip joint in degrees. */
extern const float hip_angle_from_90_max; /**< Maximum allowable angle from 90degs for the hip joint in degrees. */

/** 
 * @brief Minimum and maximum angle from 90degs for the knee joint in degrees. 
 */
extern const float knee_angle_from_90_min;  /**< Minimum allowable angle from 90degs for the knee joint in degrees. */
extern const float knee_angle_from_90_max;  /**< Maximum allowable angle from 90degs for the knee joint in degrees. */

/** 
 * @brief Minimum and maximum angle from 90degs for the tibia joint in degrees. 
 */
extern const float tibia_angle_from_90_min; /**< Minimum allowable angle from 90degs for the tibia joint in degrees. */
extern const float tibia_angle_from_90_max; /**< Maximum allowable angle from 90degs for the tibia joint in degrees. */

/** 
 * @brief Lengths of the leg segments in centimeters. 
 */
extern const float hip_length_cm;   /**< Length of the hip segment in centimeters (from the base to the femur). */
extern const float femur_length_cm; /**< Length of the femur segment in centimeters (from the femur to the tibia). */
extern const float tibia_length_cm; /**< Length of the tibia segment in centimeters (from the tibia to the ground). */

/** 
 * @brief Maximum number of servos that can be active at the same time. 
 */
extern const uint8_t max_active_servos; /**< Limit on simultaneously active servos to manage power draw. */

/* Enums **********************************************************************/

/**
 * @enum joint_type_t
 * @brief Represents the type of joint in a hexapod leg.
 * 
 * This enum defines the three main joint types for a hexapod:
 * - k_hip: The joint connecting the body to the first segment (hip).
 * - k_knee: The joint connecting the femur to the tibia.
 * - k_tibia: The joint connecting the tibia to the ground contact point.
 */
typedef enum : uint8_t {
  k_hip,   /**< Represents the hip joint. */
  k_knee,  /**< Represents the knee joint. */
  k_tibia, /**< Represents the tibia joint. */
} joint_type_t;

/* Structs ********************************************************************/

/**
 * @struct motor_t
 * @brief Represents a motor associated with a specific joint.
 * 
 * This struct encapsulates details about a motor controlling a joint,
 * including its type (hip, knee, or tibia) and its current position in degrees.
 */
typedef struct motor_t {
  joint_type_t joint_type; /**< Type of joint this motor controls. */
  float        pos_deg;    /**< Current position of the motor in degrees (from 0 to 180deg). */
  uint8_t      board_id;   /**< The ID of the PCA9685 board (0 or 1) */
  uint8_t      motor_id;   /**< The ID of the motor on the board (0 to 15) */
} motor_t;

/**
 * @brief Represents the configuration of a single leg in the hexapod robot.
 *
 * This structure holds the data necessary to control and identify a specific leg
 * in the hexapod. Each leg has an associated ID and pointers to the motor configurations
 * for its hip, knee, and tibia joints, enabling precise movement and control.
 *
 * The `hip_motor`, `knee_motor`, and `tibia_motor` pointers link to the respective
 * motor configurations stored in the PCA9685 board's motor map. These pointers allow
 * the gait logic to access motor settings and commands seamlessly.
 *
 * @note This structure is designed to be used with the hexapod's gait algorithms, 
 * ensuring proper initialization and management of each leg's motors.
 */
typedef struct leg_t {
  uint8_t  id;          /**< The ID for the leg (0 to 5). */
  motor_t *hip_motor;   /**< A pointer to the hip motor struct stored in the PCA9685 controller. */
  motor_t *knee_motor;  /**< A pointer to the knee motor struct stored in the PCA9685 controller. */
  motor_t *tibia_motor; /**< A pointer to the tibia motor struct stored in the PCA9685 controller. */
} leg_t;

#endif /* TOPOROBO_HEXAPOD_GEOMETRY */

