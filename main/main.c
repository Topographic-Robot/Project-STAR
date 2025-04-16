/* main/main.c */

#include "pstar_jtag.h"
#include "pstar_pin_validator.h"

void app_main(void)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
#if CONFIG_PSTAR_KCONFIG_JTAG_ENABLED
  pstar_jtag_t jtag_pins;
  pstar_get_jtag_pins(&jtag_pins);

  ESP_ERROR_CHECK(pstar_register_pin(jtag_pins.tdi, "JTAG TDI", false));
  ESP_ERROR_CHECK(pstar_register_pin(jtag_pins.tck, "JTAG TCK", false));
  ESP_ERROR_CHECK(pstar_register_pin(jtag_pins.tms, "JTAG TMS", false));
  ESP_ERROR_CHECK(pstar_register_pin(jtag_pins.tdo, "JTAG TDO", false));
#endif

  pstar_validate_pins();
#endif
}
