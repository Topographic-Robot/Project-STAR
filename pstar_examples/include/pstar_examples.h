/* pstar_examples/include/pstar_examples.h */

#ifndef PSTAR_EXAMPLES_H
#define PSTAR_EXAMPLES_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Checks Kconfig and runs the selected example's main function.
 * This function typically does not return, as the example function
 * it calls will likely contain an infinite loop.
 */
void run_selected_example(void);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_EXAMPLES_H */
