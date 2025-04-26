/* pstar_examples/include/qmc5883_example.h */

#ifndef QMC5883_EXAMPLE_H
#define QMC5883_EXAMPLE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Entry point for the QMC5883 example.
 * Initializes resources, reads the sensor periodically, and typically does not return.
 */
void example_qmc5883_app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* QMC5883_EXAMPLE_H */
