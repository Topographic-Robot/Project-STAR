/* components/pstar_bus/include/pstar_bus_event_types.h */

#ifndef PSTAR_BUS_EVENT_TYPES_H
#define PSTAR_BUS_EVENT_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_common_types.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/uart.h"

#include <stdbool.h>
#include <stdint.h>

/* Structs ********************************************************************/

/**
 * @brief I2C bus event data structure.
 */
typedef struct pstar_i2c_event_data {
  i2c_port_t port;     /**< I2C port number */
  uint8_t    address;  /**< I2C device address */
  bool       is_write; /**< True if this is a write event */
  size_t     len;      /**< Length of data */
} pstar_i2c_event_data_t;

/**
 * @brief SPI bus event data structure.
 */
typedef struct pstar_spi_event_data {
  spi_host_device_t host;     /**< SPI host device */
  bool              is_write; /**< True if this is a write event */
  size_t            len;      /**< Length of data */
} pstar_spi_event_data_t;

/**
 * @brief UART bus event data structure.
 */
typedef struct pstar_uart_event_data {
  uart_port_t port;     /**< UART port number */
  bool        is_write; /**< True if this is a write event */
  size_t      len;      /**< Length of data */
} pstar_uart_event_data_t;

/**
 * @brief GPIO bus event data structure.
 */
typedef struct pstar_gpio_event_data {
  gpio_num_t pin_num; /**< GPIO pin number */
  uint32_t   level;   /**< GPIO level */
} pstar_gpio_event_data_t;

/**
 * @brief SDIO bus event data structure.
 */
typedef struct pstar_sdio_event_data {
  bool   is_write;     /**< True if this is a write event */
  size_t len;          /**< Length of data */
  size_t offset;       /**< Offset of operation */
  bool   card_present; /**< True if card is present */
} pstar_sdio_event_data_t;

/**
 * @brief Bus event structure for callback information.
 */
typedef struct pstar_bus_event {
  pstar_bus_type_t bus_type; /**< Type of bus that generated the event */
  const char*      bus_name; /**< Name of the bus that generated the event */

  union {
    pstar_i2c_event_data_t  i2c;  /**< I2C specific event data */
    pstar_spi_event_data_t  spi;  /**< SPI specific event data */
    pstar_uart_event_data_t uart; /**< UART specific event data */
    pstar_gpio_event_data_t gpio; /**< GPIO specific event data */
    pstar_sdio_event_data_t sdio; /**< SDIO specific event data */
  } data;                         /**< Bus-specific event data */
} pstar_bus_event_t;

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_EVENT_TYPES_H */
