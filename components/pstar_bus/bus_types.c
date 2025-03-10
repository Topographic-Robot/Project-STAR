/* components/pstar_bus/bus_types.c */

#include "bus_types.h"

/* Public Functions ***********************************************************/

const char* pstar_bus_type_to_string(pstar_bus_type_t type)
{
  static const char* const pstar_bus_type_strings[] = {
    "NONE", /* k_pstar_bus_type_none */
    "I2C",  /* k_pstar_bus_type_i2c */
    "SPI",  /* k_pstar_bus_type_spi */
    "UART", /* k_pstar_bus_type_uart */
    "GPIO", /* k_pstar_bus_type_gpio */
    "SDIO", /* k_pstar_bus_type_sdio */
  };

  /* Validate the input type to prevent out-of-bounds array access */
  if (type >= k_pstar_bus_type_count) {
    return "UNKNOWN";
  }

  return pstar_bus_type_strings[type];
}

const char* pstar_common_mode_to_string(pstar_common_mode_t mode)
{
  static const char* const pstar_mode_strings[] = {
    "NONE",      /* k_pstar_mode_none */
    "POLLING",   /* k_pstar_mode_polling */
    "INTERRUPT", /* k_pstar_mode_interrupt */
    "DMA",       /* k_pstar_mode_dma */
    "COMBINED",  /* k_pstar_mode_combined */
  };

  /* Validate the input mode to prevent out-of-bounds array access */
  if (mode >= k_pstar_mode_count) {
    return "UNKNOWN";
  }

  return pstar_mode_strings[mode];
}
