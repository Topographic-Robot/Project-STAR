/* components/pstar_bus/include/pstar_bus_protocol_types.h */

#ifndef PSTAR_COMPONENT_BUS_PROTOCOL_TYPES_H
#define PSTAR_COMPONENT_BUS_PROTOCOL_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_common_types.h"
#include "pstar_bus_event_types.h"
#include "pstar_bus_function_types.h"

#include "driver/i2c.h"
#include "driver/spi_master.h" /* Added for SPI */

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

/**
 * @brief SPI callback functions structure.
 */
typedef struct pstar_spi_callbacks {
  pstar_spi_transfer_complete_cb_t
                       on_transfer_complete; /**< Optional: Called after successful transfer */
  pstar_spi_error_cb_t on_error;             /**< Optional: Called on SPI transaction error */
} pstar_spi_callbacks_t;

/**
 * @brief SPI operation functions structure.
 */
typedef struct pstar_spi_ops {
  pstar_spi_transmit_fn_t transmit; /**< Transmit function pointer */
  pstar_spi_receive_fn_t  receive;  /**< Receive function pointer */
  pstar_spi_transfer_fn_t transfer; /**< Full-duplex transfer function pointer */
} pstar_spi_ops_t;

/**
 * @brief SPI bus configuration structure (part of pstar_bus_config_t).
 */
typedef struct pstar_spi_bus_config {
  spi_host_device_t host;    /**< SPI host number */
  spi_bus_config_t  bus_cfg; /**< Underlying ESP-IDF SPI bus configuration */
                             /* Note: bus_cfg members like mosi_io_num are renamed below */
  spi_device_interface_config_t dev_cfg;   /**< Underlying ESP-IDF SPI device configuration */
  pstar_spi_callbacks_t         callbacks; /**< User-provided callback functions */
  pstar_spi_ops_t               ops;       /**< Operation function pointers (defaults provided) */

  /* --- Renamed members within spi_bus_config_t for internal use --- */
  /* These correspond to the fields in the ESP-IDF spi_bus_config_t */
  int posi_io_num;     /**< GPIO pin for POSI (Primary Out Secondary In) */
  int piso_io_num;     /**< GPIO pin for PISO (Primary In Secondary Out) */
  int sclk_io_num;     /**< GPIO pin for SCLK */
  int quadwp_io_num;   /**< GPIO pin for Quad SPI WP */
  int quadhd_io_num;   /**< GPIO pin for Quad SPI HD */
  int max_transfer_sz; /**< Maximum transfer size, in bytes */
  /* flags and intr_flags are also part of spi_bus_config_t if needed */

} pstar_spi_bus_config_t;

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_BUS_PROTOCOL_TYPES_H */