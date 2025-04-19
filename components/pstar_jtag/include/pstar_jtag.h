/* components/pstar_jtag/include/pstar_jtag.h */

#ifndef PSTAR_COMPONENT_JTAG_H
#define PSTAR_COMPONENT_JTAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "soc/gpio_num.h"

/* --- Structs --- */

typedef struct pstar_jtag {
  gpio_num_t tck, tms, tdi, tdo;
} pstar_jtag_t;

/* --- Functions --- */

/**
 * @brief get the jtag pins via Kconfig options
 */
esp_err_t pstar_get_jtag_pins(pstar_jtag_t* tag);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_JTAG_H */
