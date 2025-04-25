/* pstar_examples/include/dht22_example.h */

#ifndef DHT22_EXAMPLE_H
#define DHT22_EXAMPLE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Entry point for the DHT22 example.
 * Initializes resources, reads the sensor periodically, and typically does not return.
 */
void example_dht22_app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* DHT22_EXAMPLE_H */
