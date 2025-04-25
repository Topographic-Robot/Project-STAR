/* pstar_examples/include/hd44780_example.h */

#ifndef HD44780_EXAMPLE_H
#define HD44780_EXAMPLE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Entry point for the HD44780 example.
 * Initializes resources based on Kconfig (Parallel or I2C), runs the example
 * loop demonstrating LCD functions, and typically does not return.
 */
void example_hd44780_app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* HD44780_EXAMPLE_H */