/* components/common/uart.c */

#include "common/uart.h"
#include "driver/uart.h"
#include "log_handler.h"

/* Constants ******************************************************************/

const uint32_t uart_timeout_ticks = pdMS_TO_TICKS(1000); /* Timeout for UART operations in ticks */

/* Private Functions **********************************************************/

esp_err_t priv_uart_init(uint8_t     tx_io, 
                         uint8_t     rx_io, 
                         uint32_t    baud_rate,
                         uart_port_t uart_num,
                         size_t      rx_buffer_size,
                         size_t      tx_buffer_size,
                         const char *tag)
{
  uart_config_t uart_config = {
    .baud_rate = baud_rate,                /* Set the baud rate (communication speed) */
    .data_bits = UART_DATA_8_BITS,         /* Set data bits (8 bits per word) */
    .parity    = UART_PARITY_DISABLE,      /* No parity check */
    .stop_bits = UART_STOP_BITS_1,         /* Set 1 stop bit */
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, /* Disable hardware flow control */
  };

  /* Configure the UART driver with the specified settings */
  esp_err_t ret = uart_param_config(uart_num, &uart_config);
  if (ret != ESP_OK) {
    log_error(tag, 
              "Config Error", 
              "Failed to configure UART parameters: %s", 
              esp_err_to_name(ret));
    return ret;
  }

  /* Set the TX and RX pin numbers for the UART interface */
  ret = uart_set_pin(uart_num, tx_io, rx_io, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    log_error(tag, 
              "Pin Error", 
              "Failed to configure UART pins TX:%u RX:%u: %s", 
              tx_io, 
              rx_io, 
              esp_err_to_name(ret));
    return ret;
  }

  /* Install the UART driver with RX and TX buffers */
  ret = uart_driver_install(uart_num, rx_buffer_size, tx_buffer_size, 0, NULL, 0);
  if (ret != ESP_OK) {
    log_error(tag, 
              "Driver Error", 
              "Failed to install UART driver: %s", 
              esp_err_to_name(ret));
  }

  return ret; /* Return the error status or ESP_OK */
}

esp_err_t priv_uart_read(uint8_t    *data, 
                         size_t      len, 
                         int32_t    *out_length,
                         uart_port_t uart_num, 
                         const char *tag)
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

esp_err_t priv_uart_write(const uint8_t *data,
                          size_t         len,
                          int32_t       *bytes_written,
                          uart_port_t    uart_num,
                          const char    *tag)
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

