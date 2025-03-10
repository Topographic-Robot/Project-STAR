/* components/common/uart.c */

#include "common/uart.h"
#include "common/bus_manager.h"
#include "driver/uart.h"
#include "log_handler.h"

/* Constants ******************************************************************/

const uint32_t uart_timeout_ticks = pdMS_TO_TICKS(1000); /* Timeout for UART operations in ticks */

/* Private Functions **********************************************************/

esp_err_t priv_uart_init(uint8_t           tx_io, 
                         uint8_t           rx_io, 
                         uint32_t          baud_rate,
                         uart_port_t       uart_num,
                         size_t            rx_buffer_size,
                         size_t            tx_buffer_size,
                         const char* const tag)
{
  /* Use the bus manager to initialize the UART port */
  esp_err_t ret = bus_manager_uart_init(tx_io, 
                                        rx_io, 
                                        baud_rate, 
                                        uart_num, 
                                        rx_buffer_size, 
                                        tx_buffer_size);
  
  if (ret != ESP_OK) {
    log_error(tag, 
              "Init Error", 
              "Failed to initialize UART port %d: %s", 
              uart_num,
              esp_err_to_name(ret));
  } else {
    log_info(tag,
             "Init Success",
             "UART port %d initialized successfully",
             uart_num);
  }
  
  return ret;
}

/**
 * @brief Deinitializes the UART interface.
 *
 * Uses the bus manager to deinitialize the specified UART port.
 *
 * @param[in] uart_num UART port number to deinitialize.
 * @param[in] tag      Logging tag for error messages.
 *
 * @return 
 * - `ESP_OK` on successful deinitialization.
 * - Error codes from `esp_err_t` on failure.
 */
esp_err_t priv_uart_deinit(uart_port_t uart_num, const char* const tag)
{
  esp_err_t ret = bus_manager_uart_deinit(uart_num);
  
  if (ret != ESP_OK) {
    log_error(tag, 
              "Deinit Error", 
              "Failed to deinitialize UART port %d: %s", 
              uart_num,
              esp_err_to_name(ret));
  } else {
    log_info(tag,
             "Deinit Success",
             "UART port %d deinitialized successfully",
             uart_num);
  }
  
  return ret;
}

esp_err_t priv_uart_read(uint8_t*          data, 
                         size_t            len, 
                         int32_t*          out_length,
                         uart_port_t       uart_num, 
                         const char* const tag)
{
  /* Read UART data from the specified UART port */
  int32_t length = uart_read_bytes(uart_num, data, len, uart_timeout_ticks);

  if (length > 0) {
    *out_length = length; /* Store the length of data read */
    log_info(tag, "Data Received", "Read %lu bytes from UART", length);
    return ESP_OK;
  } else {
    log_error(tag, 
              "Read Error", 
              "Failed to read data from UART (timeout or error)");
    *out_length = 0; /* No data read */
    return ESP_FAIL;
  }
}

esp_err_t priv_uart_write(const uint8_t* const data,
                          size_t               len,
                          int32_t*             bytes_written,
                          uart_port_t          uart_num,
                          const char* const    tag)
{
  /* Write data to the UART port */
  int32_t written = uart_write_bytes(uart_num, (const char*)data, len);
  
  if (written > 0) {
    *bytes_written = written; /* Store the number of bytes written */
    log_info(tag, "Data Sent", "Wrote %lu bytes to UART", written);
    return ESP_OK;
  } else {
    log_error(tag, "Write Error", "Failed to write data to UART");
    *bytes_written = 0; /* No data written */
    return ESP_FAIL;
  }
}

