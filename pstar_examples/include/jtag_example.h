/* pstar_examples/include/jtag_example.h */

#ifndef JTAG_EXAMPLE_H
#define JTAG_EXAMPLE_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Entry point for the JTAG Configuration example.
 * Retrieves and prints the configured JTAG pins based on Kconfig settings.
 */
void example_jtag_app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* JTAG_EXAMPLE_H */
