/* components/pstar_bus/include/pstar_bus_manager_types.h */

#ifndef PSTAR_BUS_MANAGER_TYPES_H
#define PSTAR_BUS_MANAGER_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_common_types.h"
#include "pstar_bus_protocol_types.h"

/* Structs ********************************************************************/

/**
 * @brief Union of configuration types for different bus interfaces.
 */
typedef union pstar_bus_config_union {
  pstar_i2c_bus_config_t  i2c;  /**< I2C specific configuration */
  pstar_spi_bus_config_t  spi;  /**< SPI specific configuration */
  pstar_uart_bus_config_t uart; /**< UART specific configuration */
  pstar_gpio_bus_config_t gpio; /**< GPIO specific configuration */
  pstar_sdio_bus_config_t sdio; /**< SDIO specific configuration */
} pstar_bus_config_union_t;

/**
 * @brief Bus configuration structure.
 *
 * This structure contains all the configuration data for a specific bus
 * instance. It serves as a node in a linked list, with each node
 * representing a different bus managed by the bus_manager.
 */
typedef struct pstar_bus_config {
  const char*         name;        /**< Unique name for this bus instance */
  pstar_bus_type_t    type;        /**< Type of bus */
  pstar_common_mode_t mode;        /**< Operation mode */
  bool                initialized; /**< Whether this bus has been initialized */
  void*               handle;      /**< Device handle (if applicable) */
  void*               user_ctx;    /**< User context pointer */

  pstar_bus_config_union_t config; /**< Bus-specific configuration */

  struct pstar_bus_config* next; /**< Pointer to the next bus in the list */
} pstar_bus_config_t;

/**
 * @brief Bus manager structure that maintains a list of bus configurations.
 *
 * @note Thread Safety: The functions managing bus configurations are not
 *       inherently thread-safe. Users should ensure external locking if
 *       multiple tasks may access the manager concurrently.
 */
typedef struct pstar_bus_manager {
  pstar_bus_config_t* buses; /**< Linked list of bus configurations */
  SemaphoreHandle_t   mutex; /**< Mutex for thread safety */
  const char*         tag;   /**< Tag for logging */
} pstar_bus_manager_t;

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_MANAGER_TYPES_H */
