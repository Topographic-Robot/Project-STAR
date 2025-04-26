/* components/pstar_bus/pstar_bus_types.c */

#include "pstar_bus_types.h"

/* --- Public Functions --- */

const char* pstar_bus_type_to_string(pstar_bus_type_t type)
{
  /* Ensure the array size matches the enum count (excluding _count) */
  static const char* const pstar_bus_type_strings[k_pstar_bus_type_count] = {
    [k_pstar_bus_type_none] = "NONE",
    [k_pstar_bus_type_i2c]  = "I2C",
    [k_pstar_bus_type_spi]  = "SPI",
  };

  /* Use >= count for check as enum starts from 0 */
  if (type >= k_pstar_bus_type_count || type < k_pstar_bus_type_none) {
    return "UNKNOWN";
  }

  return pstar_bus_type_strings[type];
}
