/* main/hexapod_geometry.c */

#include "hexapod_geometry.h"

/* Constants ******************************************************************/

const float hip_angle_from_90_min = 0.0f;  /* This means the min is 90deg */
const float hip_angle_from_90_max = 60.0f; /* This means the max is 90+60=150deg */

const float knee_angle_from_90_min = 0.0f;  /* This means the min is 90deg */
const float knee_angle_from_90_max = 90.0f; /* This means the max is 90+90=180deg */

const float tibia_angle_from_90_min = -45.0f; /* This means the min is 90-45=45deg */
const float tibia_angle_from_90_max = 45.0f;  /* This means the max is 90+45=135deg */

/* TODO: Replace this with the actual values once Matt measures them */
const float hip_length_cm   = 5.0f;  /* Distance from the body center to the hip joint */
const float femur_length_cm = 10.0f; /* Length of the femur (thigh segment) */
const float tibia_length_cm = 12.0f; /* Length of the tibia (shin segment) */

const uint8_t max_active_servos = 3; /* Maximum number of servos allowed to move simultaneously */
