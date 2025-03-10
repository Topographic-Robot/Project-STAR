/* components/pstar_bus/include/bus_protocol_types.h */

#ifndef PSTAR_BUS_PROTOCOL_TYPES_H
#define PSTAR_BUS_PROTOCOL_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "bus_common_types.h"
#include "bus_event_types.h"
#include "bus_function_types.h"

/* Protocol Configuration Structures *******************************************/

/**
 * @brief I2C callback functions structure.
 */
typedef struct pstar_i2c_callbacks {
  pstar_i2c_transfer_complete_cb_t on_transfer_complete; /**< Called when transfer completes */
  pstar_i2c_error_cb_t             on_error;             /**< Called on error */
} pstar_i2c_callbacks_t;

/**
 * @brief I2C operation functions structure.
 */
typedef struct pstar_i2c_ops {
  pstar_i2c_write_fn_t write; /**< Write function */
  pstar_i2c_read_fn_t  read;  /**< Read function */
} pstar_i2c_ops_t;

/**
 * @brief I2C bus configuration structure.
 */
typedef struct pstar_i2c_bus_config {
  i2c_port_t            port;     /**< I2C port number */
  i2c_config_t          config;   /**< I2C configuration */
  uint8_t               address;  /**< 7-bit I2C device address */
  pstar_i2c_callbacks_t callbacks; /**< I2C callback functions */
  pstar_i2c_ops_t       ops;      /**< I2C operation function pointers */
} pstar_i2c_bus_config_t;

/**
 * @brief SPI callback functions structure.
 */
typedef struct pstar_spi_callbacks {
  pstar_spi_transfer_complete_cb_t on_transfer_complete; /**< Called when transfer completes */
  pstar_spi_error_cb_t             on_error;             /**< Called on error */
} pstar_spi_callbacks_t;

/**
 * @brief SPI operation functions structure.
 */
typedef struct pstar_spi_ops {
  pstar_spi_write_fn_t write; /**< Write function */
  pstar_spi_read_fn_t  read;  /**< Read function */
} pstar_spi_ops_t;

/**
 * @brief SPI bus configuration structure.
 * 
 * Note: Uses ESP-IDF's spi_bus_config_t internally
 */
typedef struct pstar_spi_bus_config {
  spi_host_device_t             host;       /**< SPI host device */
  spi_bus_config_t              bus_config; /**< SPI bus configuration (ESP-IDF type) */
  spi_device_interface_config_t dev_config; /**< SPI device configuration */
  pstar_spi_callbacks_t         callbacks;  /**< SPI callback functions */
  pstar_spi_ops_t               ops;        /**< SPI operation function pointers */
} pstar_spi_bus_config_t;

/**
 * @brief UART callback functions structure.
 */
typedef struct pstar_uart_callbacks {
  pstar_uart_data_received_cb_t on_data_received; /**< Called when data is received */
  pstar_uart_data_sent_cb_t     on_data_sent;     /**< Called when data is sent */
  pstar_uart_error_cb_t         on_error;         /**< Called on error */
} pstar_uart_callbacks_t;

/**
 * @brief UART operation functions structure.
 */
typedef struct pstar_uart_ops {
  pstar_uart_write_fn_t write; /**< Write function */
  pstar_uart_read_fn_t  read;  /**< Read function */
} pstar_uart_ops_t;

/**
 * @brief UART bus configuration structure.
 */
typedef struct pstar_uart_bus_config {
  uart_port_t            port;           /**< UART port number to use */
  uart_config_t          config;         /**< ESP-IDF UART configuration */
  size_t                 rx_buffer_size; /**< Size of UART RX buffer */
  size_t                 tx_buffer_size; /**< Size of UART TX buffer */
  
  /* UART runtime data */
  QueueHandle_t          rx_queue;       /**< UART RX queue handle */
  QueueHandle_t          tx_queue;       /**< UART TX queue handle */
  
  pstar_uart_callbacks_t callbacks;      /**< UART callback functions */
  pstar_uart_ops_t       ops;            /**< UART operation function pointers */
} pstar_uart_bus_config_t;

/**
 * @brief GPIO callback functions structure.
 */
typedef struct pstar_gpio_callbacks {
  pstar_gpio_interrupt_cb_t    on_interrupt;    /**< Called on interrupt */
  pstar_gpio_level_change_cb_t on_level_change; /**< Called on level change */
} pstar_gpio_callbacks_t;

/**
 * @brief GPIO operation functions structure.
 */
typedef struct pstar_gpio_ops {
  pstar_gpio_set_level_fn_t  set_level;  /**< Set GPIO level function */
  pstar_gpio_get_level_fn_t  get_level;  /**< Get GPIO level function */
  pstar_gpio_isr_add_fn_t    isr_add;    /**< Add ISR handler function */
  pstar_gpio_isr_remove_fn_t isr_remove; /**< Remove ISR handler function */
} pstar_gpio_ops_t;

/**
 * @brief GPIO bus configuration structure.
 */
typedef struct pstar_gpio_bus_config {
  gpio_config_t          config;    /**< GPIO configuration */
  pstar_gpio_callbacks_t callbacks; /**< GPIO callback functions */
  pstar_gpio_ops_t       ops;       /**< GPIO operation function pointers */
} pstar_gpio_bus_config_t;

/**
 * @brief SDIO callback functions structure.
 */
typedef struct pstar_sdio_callbacks {
  pstar_sdio_transfer_complete_cb_t on_transfer_complete; /**< Called when transfer completes */
  pstar_sdio_error_cb_t             on_error;             /**< Called on error */
  pstar_sdio_card_detect_cb_t       on_card_detect;       /**< Called on card insertion/removal */
} pstar_sdio_callbacks_t;

/**
 * @brief SDIO operation functions structure.
 */
typedef struct pstar_sdio_ops {
  pstar_sdio_write_fn_t write; /**< Write function */
  pstar_sdio_read_fn_t  read;  /**< Read function */
  pstar_sdio_ioctl_fn_t ioctl; /**< IOCTL function */
} pstar_sdio_ops_t;

/**
 * @brief SDIO bus configuration structure.
 */
typedef struct pstar_sdio_bus_config {
  sdmmc_host_t           host;             /**< SDIO host configuration */
  sdmmc_slot_config_t    slot_config;      /**< SDIO slot configuration */
  sdmmc_card_t*          card;             /**< SDIO card handle */
  
  /* Callbacks and operations */
  pstar_sdio_callbacks_t callbacks;        /**< SDIO callback functions */
  pstar_sdio_ops_t       ops;              /**< SDIO operation function pointers */
  
  /* Internal state */
  bool                   card_inserted;    /**< Whether card is currently inserted */
  bool                   detect_enabled;   /**< Whether card detect is enabled */
  SemaphoreHandle_t      mutex;            /**< Mutex for thread safety */
  gpio_num_t             card_detect_pin;  /**< Card detect GPIO pin */
  
  /* SPI mode configuration */
  sdspi_device_config_t  sdspi_dev_config; /**< SD SPI device configuration for SPI mode */
} pstar_sdio_bus_config_t;

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_PROTOCOL_TYPES_H */
