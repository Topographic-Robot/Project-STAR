/* components/pstar_bus/include/pstar_bus_function_types.h */

#ifndef PSTAR_BUS_FUNCTION_TYPES_H
#define PSTAR_BUS_FUNCTION_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_common_types.h"
#include "pstar_bus_event_types.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/uart.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* Forward Declarations */
typedef struct pstar_bus_manager pstar_bus_manager_t;

/* Callback Function Types ****************************************************/

/* I2C Callback Function Types */
typedef void (*pstar_i2c_transfer_complete_cb_t)(pstar_bus_event_t* event, void* user_ctx);

typedef void (*pstar_i2c_error_cb_t)(pstar_bus_event_t* event, esp_err_t error, void* user_ctx);

/* SPI Callback Function Types */
typedef void (*pstar_spi_transfer_complete_cb_t)(pstar_bus_event_t* event, void* user_ctx);

typedef void (*pstar_spi_error_cb_t)(pstar_bus_event_t* event, esp_err_t error, void* user_ctx);

/* UART Callback Function Types */
typedef void (*pstar_uart_data_received_cb_t)(pstar_bus_event_t* event,
                                              const uint8_t*     data,
                                              size_t             len,
                                              void*              user_ctx);

typedef void (*pstar_uart_data_sent_cb_t)(pstar_bus_event_t* event, size_t len, void* user_ctx);

typedef void (*pstar_uart_error_cb_t)(pstar_bus_event_t* event, esp_err_t error, void* user_ctx);

/* GPIO Callback Function Types */
typedef void (*pstar_gpio_interrupt_cb_t)(pstar_bus_event_t* event, void* user_ctx);

typedef void (*pstar_gpio_level_change_cb_t)(pstar_bus_event_t* event,
                                             uint32_t           level,
                                             void*              user_ctx);

/* SDIO Callback Function Types */
typedef void (*pstar_sdio_transfer_complete_cb_t)(pstar_bus_event_t* event, void* user_ctx);

typedef void (*pstar_sdio_error_cb_t)(pstar_bus_event_t* event, esp_err_t error, void* user_ctx);

typedef void (*pstar_sdio_card_detect_cb_t)(pstar_bus_event_t* event,
                                            bool               inserted,
                                            void*              user_ctx);

/* Operation Function Types ***************************************************/

/* I2C Operation Function Types */
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

/* SPI Operation Function Types */
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

/* UART Operation Function Types */
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

/* GPIO Operation Function Types */
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

/* SDIO Operation Function Types */
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

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_FUNCTION_TYPES_H */
