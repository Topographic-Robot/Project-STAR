/* components/pstar_bus/include/pstar_bus_protocol_types.h */

#ifndef PSTAR_BUS_PROTOCOL_TYPES_H
#define PSTAR_BUS_PROTOCOL_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_common_types.h"
#include "pstar_bus_event_types.h"
#include "pstar_bus_function_types.h"

#include "driver/i2c.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* --- Protocol Configuration Structures --- */

/**
 * @brief I2C callback functions structure.
 */
typedef struct pstar_i2c_callbacks {
  pstar_i2c_transfer_complete_cb_t
                       on_transfer_complete; /**< Optional: Called after successful read/write */
  pstar_i2c_error_cb_t on_error;             /**< Optional: Called on I2C transaction error */
} pstar_i2c_callbacks_t;

/**
 * @brief I2C operation functions structure.
 */
typedef struct pstar_i2c_ops {
  pstar_i2c_write_fn_t         write;         /**< Write function pointer (reg_addr + data) */
  pstar_i2c_read_fn_t          read;          /**< Read function pointer (reg_addr + data) */
  pstar_i2c_write_command_fn_t write_command; /**< Write function pointer (command byte only) */
  pstar_i2c_read_raw_fn_t      read_raw;      /**< Read function pointer (raw bytes, no reg_addr) */
} pstar_i2c_ops_t;

/**
 * @brief I2C bus configuration structure (part of pstar_bus_config_t).
 */
typedef struct pstar_i2c_bus_config {
  i2c_port_t            port;      /**< I2C port number */
  i2c_config_t          config;    /**< Underlying ESP-IDF I2C configuration */
  uint8_t               address;   /**< 7-bit I2C device address for this config */
  pstar_i2c_callbacks_t callbacks; /**< User-provided callback functions */
  pstar_i2c_ops_t       ops;       /**< Operation function pointers (defaults provided) */
} pstar_i2c_bus_config_t;

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_PROTOCOL_TYPES_H */
