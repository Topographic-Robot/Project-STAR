/* components/pstar_bus/include/bus_types.h */

#ifndef PSTAR_BUS_TYPES_H
#define PSTAR_BUS_TYPES_H

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

/* Forward Declarations *******************************************************/

/* Forward declare pstar_bus_config_t and pstar_bus_event_t */
typedef struct pstar_bus_config pstar_bus_config_t;
typedef struct pstar_bus_event pstar_bus_event_t;

/* Forward declare pstar_bus_manager_t for use in function pointer types */
typedef struct pstar_bus_manager pstar_bus_manager_t;

/* Enums **********************************************************************/

/**
 * @brief Types of supported bus interfaces.
 */
typedef enum : uint8_t {
  k_pstar_bus_type_none,  /**< No bus type specified */
  k_pstar_bus_type_i2c,   /**< I2C bus */
  k_pstar_bus_type_spi,   /**< SPI bus */
  k_pstar_bus_type_uart,  /**< UART bus */
  k_pstar_bus_type_gpio,  /**< GPIO bus */
  k_pstar_bus_type_sdio,  /**< SDIO bus */
  k_pstar_bus_type_count, /**< Number of bus types */
} pstar_bus_type_t;

/**
 * @brief Common operation modes for buses.
 */
typedef enum : uint8_t {
  k_pstar_mode_none,      /**< No mode specified */
  k_pstar_mode_polling,   /**< Polling mode (synchronous) */
  k_pstar_mode_interrupt, /**< Interrupt mode (asynchronous) */
  k_pstar_mode_dma,       /**< DMA mode (high-performance asynchronous) */
  k_pstar_mode_combined,  /**< Combined mode (interrupt+DMA) */
  k_pstar_mode_count,     /**< Number of modes */
} pstar_common_mode_t;

/**
 * @brief IOCTL commands for SDIO bus.
 */
typedef enum : uint32_t {
  k_pstar_sdio_ioctl_get_card_info,      /**< Get card info (arg: sdmmc_card_t**) */
  k_pstar_sdio_ioctl_get_status,         /**< Get card status (arg: uint8_t*) */
  k_pstar_sdio_ioctl_get_csd,            /**< Get CSD register (arg: sdmmc_csd_t*) */
  k_pstar_sdio_ioctl_get_cid,            /**< Get CID register (arg: sdmmc_cid_t*) */
  k_pstar_sdio_ioctl_get_scr,            /**< Get SCR register (arg: sdmmc_scr_t*) */
  k_pstar_sdio_ioctl_get_ocr,            /**< Get OCR register (arg: uint32_t*) */
  k_pstar_sdio_ioctl_get_rca,            /**< Get RCA register (arg: uint16_t*) */
  k_pstar_sdio_ioctl_get_bus_width,      /**< Get bus width (arg: uint8_t*) */
  k_pstar_sdio_ioctl_set_bus_width,      /**< Set bus width (arg: uint8_t*) */
  k_pstar_sdio_ioctl_get_bus_freq,       /**< Get bus frequency (arg: uint32_t*) */
  k_pstar_sdio_ioctl_set_bus_freq,       /**< Set bus frequency (arg: uint32_t*) */
  k_pstar_sdio_ioctl_enable_card_detect, /**< Enable card detect (arg: bool*) */
  k_pstar_sdio_ioctl_custom,             /**< Custom command, device-specific */
} pstar_sdio_ioctl_cmd_t;

/* Callback Function Types ****************************************************/

/**
 * @brief Common callback types for I2C and SPI
 */
/* I2C Callback Function Types */
typedef void (*pstar_i2c_transfer_complete_cb_t)(pstar_bus_event_t* event, void* user_ctx);
typedef void (*pstar_i2c_error_cb_t)(pstar_bus_event_t* event, esp_err_t error, void* user_ctx);

/* SPI Callback Function Types */
typedef void (*pstar_spi_transfer_complete_cb_t)(pstar_bus_event_t* event, void* user_ctx);
typedef void (*pstar_spi_error_cb_t)(pstar_bus_event_t* event, esp_err_t error, void* user_ctx);

/* UART Callback Function Types */
typedef void (*pstar_uart_data_received_cb_t)(pstar_bus_event_t* event, const uint8_t* data, size_t len, void* user_ctx);
typedef void (*pstar_uart_data_sent_cb_t)(pstar_bus_event_t* event, size_t len, void* user_ctx);
typedef void (*pstar_uart_error_cb_t)(pstar_bus_event_t* event, esp_err_t error, void* user_ctx);

/* GPIO Callback Function Types */
typedef void (*pstar_gpio_interrupt_cb_t)(pstar_bus_event_t* event, void* user_ctx);
typedef void (*pstar_gpio_level_change_cb_t)(pstar_bus_event_t* event, uint32_t level, void* user_ctx);

/* SDIO Callback Function Types */
typedef void (*pstar_sdio_transfer_complete_cb_t)(pstar_bus_event_t* event, void* user_ctx);
typedef void (*pstar_sdio_error_cb_t)(pstar_bus_event_t* event, esp_err_t error, void* user_ctx);
typedef void (*pstar_sdio_card_detect_cb_t)(pstar_bus_event_t* event, bool inserted, void* user_ctx);

/* Operation Function Types ***************************************************/

/**
 * @brief I2C operation function types
 */
typedef esp_err_t (*pstar_i2c_write_fn_t)(const pstar_bus_manager_t* manager,
                                         const char*                name, 
                                         const uint8_t*             data, 
                                         size_t                     len, 
                                         uint8_t                    reg_addr, 
                                         size_t*                    bytes_written);

typedef esp_err_t (*pstar_i2c_read_fn_t)(const pstar_bus_manager_t* manager,
                                        const char*                name, 
                                        uint8_t*                   data, 
                                        size_t                     len, 
                                        uint8_t                    reg_addr, 
                                        size_t*                    bytes_read);

/**
 * @brief SPI operation function types
 */
typedef esp_err_t (*pstar_spi_write_fn_t)(const pstar_bus_manager_t* manager,
                                         const char*                name, 
                                         const uint8_t*             data, 
                                         size_t                     len, 
                                         size_t*                    bytes_written);

typedef esp_err_t (*pstar_spi_read_fn_t)(const pstar_bus_manager_t* manager,
                                        const char*                name, 
                                        uint8_t*                   data, 
                                        size_t                     len, 
                                        size_t*                    bytes_read);

/**
 * @brief UART operation function types
 */
typedef esp_err_t (*pstar_uart_write_fn_t)(const pstar_bus_manager_t* manager,
                                          const char*                name, 
                                          const uint8_t*             data, 
                                          size_t                     len, 
                                          size_t*                    bytes_written);

typedef esp_err_t (*pstar_uart_read_fn_t)(const pstar_bus_manager_t* manager,
                                         const char*                name, 
                                         uint8_t*                   data, 
                                         size_t                     len, 
                                         size_t*                    bytes_read);

/**
 * @brief GPIO operation function types
 */
typedef esp_err_t (*pstar_gpio_set_level_fn_t)(const pstar_bus_manager_t* manager,
                                              const char*                name, 
                                              gpio_num_t                 pin_num, 
                                              uint32_t                   level);

typedef esp_err_t (*pstar_gpio_get_level_fn_t)(const pstar_bus_manager_t* manager,
                                              const char*                name, 
                                              gpio_num_t                 pin_num, 
                                              uint32_t*                  level);

typedef esp_err_t (*pstar_gpio_isr_add_fn_t)(const pstar_bus_manager_t* manager,
                                            const char*                name, 
                                            gpio_num_t                 pin_num, 
                                            gpio_isr_t                 isr_handler, 
                                            void*                      args);

typedef esp_err_t (*pstar_gpio_isr_remove_fn_t)(const pstar_bus_manager_t* manager,
                                               const char*                name, 
                                               gpio_num_t                 pin_num);

/**
 * @brief SDIO operation function types
 */
typedef esp_err_t (*pstar_sdio_write_fn_t)(const pstar_bus_manager_t* manager,
                                          const char*                name, 
                                          const uint8_t*             data, 
                                          size_t                     len, 
                                          size_t                     offset,
                                          size_t*                    bytes_written);

typedef esp_err_t (*pstar_sdio_read_fn_t)(const pstar_bus_manager_t* manager,
                                         const char*                name, 
                                         uint8_t*                   data, 
                                         size_t                     len, 
                                         size_t                     offset,
                                         size_t*                    bytes_read);

typedef esp_err_t (*pstar_sdio_ioctl_fn_t)(const pstar_bus_manager_t* manager,
                                          const char*                name,
                                          int                        cmd,
                                          void*                      arg);

/* Structs ********************************************************************/

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
  i2c_port_t             port;     /**< I2C port number */
  i2c_config_t           config;   /**< I2C configuration */
  uint8_t                address;  /**< 7-bit I2C device address */
  pstar_i2c_callbacks_t  callbacks; /**< I2C callback functions */
  pstar_i2c_ops_t        ops;      /**< I2C operation function pointers */
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
  uart_port_t             port;           /**< UART port number to use */
  uart_config_t           config;         /**< ESP-IDF UART configuration */
  size_t                  rx_buffer_size; /**< Size of UART RX buffer */
  size_t                  tx_buffer_size; /**< Size of UART TX buffer */
  uint32_t                rx_timeout_ms;  /**< Read timeout in milliseconds (default: 1000) */
  
  /* UART runtime data */
  QueueHandle_t           rx_queue;       /**< UART RX queue handle */
  QueueHandle_t           tx_queue;       /**< UART TX queue handle */
  
  pstar_uart_callbacks_t  callbacks;      /**< UART callback functions */
  pstar_uart_ops_t        ops;            /**< UART operation function pointers */
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
  gpio_config_t           config;    /**< GPIO configuration */
  pstar_gpio_callbacks_t  callbacks; /**< GPIO callback functions */
  pstar_gpio_ops_t        ops;       /**< GPIO operation function pointers */
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
  sdmmc_host_t            host;            /**< SDIO host configuration */
  sdmmc_slot_config_t     slot_config;     /**< SDIO slot configuration */
  sdmmc_card_t*           card;            /**< SDIO card handle */
  
  /* Callbacks and operations */
  pstar_sdio_callbacks_t  callbacks;       /**< SDIO callback functions */
  pstar_sdio_ops_t        ops;             /**< SDIO operation function pointers */
  
  /* Internal state */
  bool                    card_inserted;   /**< Whether card is currently inserted */
  bool                    detect_enabled;  /**< Whether card detect is enabled */
  SemaphoreHandle_t       mutex;           /**< Mutex for thread safety */
  gpio_num_t              card_detect_pin; /**< Card detect GPIO pin */
  
  /* SPI mode configuration */
  sdspi_device_config_t   sdspi_dev_config; /**< SD SPI device configuration for SPI mode */
} pstar_sdio_bus_config_t;

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
  const char*                name;        /**< Unique name for this bus instance */
  pstar_bus_type_t           type;        /**< Type of bus */
  pstar_common_mode_t        mode;        /**< Operation mode */
  bool                       initialized; /**< Whether this bus has been initialized */
  void*                      handle;      /**< Device handle (if applicable) */
  void*                      user_ctx;    /**< User context pointer */
  
  pstar_bus_config_union_t   config;      /**< Bus-specific configuration */
  
  struct pstar_bus_config*   next;        /**< Pointer to the next bus in the list */
} pstar_bus_config_t;

/* Public Functions ***********************************************************/

/**
 * @brief Convert bus type to string.
 * 
 * @param[in] type Bus type.
 * @return const char* String representation of the bus type.
 */
const char* pstar_bus_type_to_string(pstar_bus_type_t type);

/**
 * @brief Convert mode to string.
 * 
 * @param[in] mode Operation mode.
 * @return const char* String representation of the mode.
 */
const char* pstar_common_mode_to_string(pstar_common_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_TYPES_H */