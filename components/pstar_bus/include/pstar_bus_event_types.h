/* components/pstar_bus/include/pstar_bus_event_types.h */

#ifndef PSTAR_COMPONENT_BUS_EVENT_TYPES_H
#define PSTAR_COMPONENT_BUS_EVENT_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_common_types.h"

#include "driver/i2c.h"
#include "driver/spi_master.h" /* Added for SPI */

#include <stdbool.h>
#include <stdint.h>

/* --- Structs --- */

/**
 * @brief I2C bus event data structure.
 */
typedef struct pstar_i2c_event_data {
  i2c_port_t port;     /**< I2C port number */
  uint8_t    address;  /**< I2C device address associated with the config */
  bool       is_write; /**< True if this is a write event */
  size_t     len;      /**< Length of data transferred (can be 0 for command-only writes/errors) */
} pstar_i2c_event_data_t;

/**
 * @brief SPI bus event data structure.
 */
typedef struct pstar_spi_event_data {
  spi_host_device_t host;     /**< SPI host number */
  int               cs_pin;   /**< Chip select pin used for the transaction */
  bool              is_write; /**< True if it was primarily a write/transmit operation */
  size_t            tx_len;   /**< Length of data transmitted */
  size_t            rx_len;   /**< Length of data received */
} pstar_spi_event_data_t;

/**
 * @brief Bus event structure passed to callback functions.
 */
typedef struct pstar_bus_event {
  pstar_bus_type_t bus_type; /**< Type of bus that generated the event */
  const char*      bus_name; /**< Name of the bus configuration that generated the event */

  /* Union to hold specific event data based on bus type */
  union {
    pstar_i2c_event_data_t i2c; /**< I2C specific event data */
    pstar_spi_event_data_t spi; /**< SPI specific event data */
    /* Add other bus types here */
  };

} pstar_bus_event_t;

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_BUS_EVENT_TYPES_H */
