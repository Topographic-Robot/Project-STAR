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
  float        pos_deg;    /**< Current position of the motor in degrees. */
} motor_t;

#endif /* TOPOROBO_HEXAPOD_GEOMETRY */

