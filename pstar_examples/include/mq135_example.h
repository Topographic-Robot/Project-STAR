/* pstar_examples/include/mq135_example.h */

#ifndef MQ135_EXAMPLE_H
#define MQ135_EXAMPLE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Entry point for the MQ135 example.
 * Initializes resources, reads the sensor periodically, and typically does not return.
 */
void example_mq135_app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* MQ135_EXAMPLE_H */
