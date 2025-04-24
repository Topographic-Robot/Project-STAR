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
 * @brief Get the JTAG pins via Kconfig options
 *
 * @param[out] tag Pointer to store JTAG pin configuration
 * @return esp_err_t ESP_OK on success, or error code
 */
esp_err_t pstar_get_jtag_pins(pstar_jtag_t* tag);

/**
 * @brief Registers the pins used by JTAG with the pin validator using KConfig values.
 *
 * This function should be called after the pin validator is ready but before
 * pstar_validate_pins() is called. It only performs registration if both the
 * pin validator component and JTAG are enabled via Kconfig.
 *
 * @return esp_err_t ESP_OK on success (or if validator is disabled), or an error code on failure.
 */
esp_err_t pstar_jtag_register_kconfig_pins(void);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_JTAG_H */