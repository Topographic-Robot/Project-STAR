/* pstar_examples/include/mpu6050_example.h */

#ifndef MPU6050_EXAMPLE_H
#define MPU6050_EXAMPLE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Entry point for the MPU6050 example.
 * Initializes resources, reads the sensor periodically, and typically does not return.
 */
void example_mpu6050_app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_EXAMPLE_H */