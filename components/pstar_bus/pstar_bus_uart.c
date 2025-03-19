/* components/pstar_bus/bus_uart.c */

#include "pstar_bus_uart.h"
#include "pstar_bus_manager.h"
#include "pstar_log_handler.h"
#include "pstar_bus_types.h"
#include "pstar_bus_event.h"
#include <string.h>

/* Constants ******************************************************************/

#define BUS_UART_TAG ("Bus UART")

/* Private Function Prototypes ************************************************/

static esp_err_t priv_pstar_bus_uart_write(const pstar_bus_manager_t* manager,
                                           const char*                name,
                                           const uint8_t*             data, 
                                           size_t                     len, 
                                           size_t*                    bytes_written);

static esp_err_t priv_pstar_bus_uart_read(const pstar_bus_manager_t* manager,
                                          const char*                name,
                                          uint8_t*                   data, 
                                          size_t                     len, 
                                          size_t*                    bytes_read);

/* Public Functions ***********************************************************/

void pstar_bus_uart_init_default_ops(pstar_uart_ops_t* ops)
{
  if (ops == NULL) {
    log_error(BUS_UART_TAG, 
              "Init Error", 
              "UART operations pointer is NULL");
    return;
  }
  
  ops->write = priv_pstar_bus_uart_write;
  ops->read  = priv_pstar_bus_uart_read;
  
  log_info(BUS_UART_TAG, 
           "Default Ops", 
           "Initialized default UART operations");
}

esp_err_t pstar_bus_uart_write(const pstar_bus_manager_t* manager,
                               const char*                name,
                               const uint8_t*             data, 
                               size_t                     len, 
                               size_t*                    bytes_written)
{
  /* Validate input */
  if (!manager || !name) {
    log_error(BUS_UART_TAG, 
              "Write Error", 
              "Invalid parameters: manager or name is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_UART_TAG, 
              "Write Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is a UART bus */
  if (bus_config->type != k_pstar_bus_type_uart) {
    log_error(BUS_UART_TAG, 
              "Write Error", 
              "Bus '%s' is not a UART bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Call the write operation function */
  if (bus_config->config.uart.ops.write) {
    return bus_config->config.uart.ops.write(manager, 
                                             name, 
                                             data, 
                                             len, 
                                             bytes_written);
  } else {
    log_error(BUS_UART_TAG, 
              "Write Error", 
              "No write operation defined for UART bus '%s'", 
              name);
    return ESP_ERR_NOT_SUPPORTED;
  }
}

esp_err_t pstar_bus_uart_read(const pstar_bus_manager_t* manager,
                        const char*          name,
                        uint8_t*             data, 
                        size_t               len, 
                        size_t*              bytes_read)
{
  /* Validate input */
  if (!manager || !name) {
    log_error(BUS_UART_TAG, 
              "Read Error", 
              "Invalid parameters: manager or name is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_UART_TAG, 
              "Read Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is a UART bus */
  if (bus_config->type != k_pstar_bus_type_uart) {
    log_error(BUS_UART_TAG, 
              "Read Error", 
              "Bus '%s' is not a UART bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Call the read operation function */
  if (bus_config->config.uart.ops.read) {
    return bus_config->config.uart.ops.read(manager, name, data, len, bytes_read);
  } else {
    log_error(BUS_UART_TAG, 
              "Read Error", 
              "No read operation defined for UART bus '%s'", 
              name);
    return ESP_ERR_NOT_SUPPORTED;
  }
}

/* Private Functions **********************************************************/

/**
 * @brief Write data to a UART device using the default implementation.
 * 
 * @param[in]  manager       Pointer to the bus manager.
 * @param[in]  name          Name of the UART bus.
 * @param[in]  data          Data to write.
 * @param[in]  len           Length of data to write.
 * @param[out] bytes_written Pointer to store the number of bytes written (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
static esp_err_t priv_pstar_bus_uart_write(const pstar_bus_manager_t* manager,
                                           const char*                name,
                                           const uint8_t*             data, 
                                           size_t                     len, 
                                           size_t*                    bytes_written)
{
  esp_err_t result = ESP_OK;
  
  /* Validate input */
  if (!manager || !name || !data || len == 0) {
    log_error(BUS_UART_TAG, 
              "Write Error", 
              "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_UART_TAG, 
              "Write Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is a UART bus */
  if (bus_config->type != k_pstar_bus_type_uart) {
    log_error(BUS_UART_TAG, 
              "Write Error", 
              "Bus '%s' is not a UART bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(BUS_UART_TAG, 
              "Write Error", 
              "Bus '%s' is not initialized", 
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Get UART-specific configuration */
  uart_port_t port = bus_config->config.uart.port;
  
  /* Write data to UART */
  int written = uart_write_bytes(port, (const char*)data, len);
  if (written < 0) {
    log_error(BUS_UART_TAG, 
              "Write Error", 
              "Failed to write data to UART port %d", 
              port);
    return ESP_FAIL;
  }
  
  /* Ensure all data is transmitted before returning */
  result = uart_wait_tx_done(port, pdMS_TO_TICKS(1000));
  if (result != ESP_OK) {
    log_error(BUS_UART_TAG, 
              "Write Error", 
              "Failed to wait for UART transmission completion: %s", 
              esp_err_to_name(result));
    return result;
  }
  
  log_debug(BUS_UART_TAG, 
            "Write Success", 
            "Successfully wrote %d bytes to UART bus '%s' (port %d)",
            written, 
            name, 
            port);
  
  /* Set bytes written if provided */
  if (bytes_written) {
    *bytes_written = written;
  }
  
  /* Call callback if available */
  if (bus_config->config.uart.callbacks.on_data_sent) {
    /* Create event structure */
    pstar_bus_event_t event = {
      .bus_type  = k_pstar_bus_type_uart,
      .bus_name  = name,
      .data.uart = {
        .port     = port,
        .is_write = true,
        .len      = written
      }
    };
    
    /* Call the callback */
    bus_config->config.uart.callbacks.on_data_sent(&event, 
                                                   written, 
                                                   bus_config->user_ctx);
  }
  
  return ESP_OK;
}

/**
 * @brief Read data from a UART device using the default implementation.
 * 
 * @param[in]  manager    Pointer to the bus manager.
 * @param[in]  name       Name of the UART bus.
 * @param[out] data       Buffer to read data into.
 * @param[in]  len        Length of data to read.
 * @param[out] bytes_read Pointer to store the number of bytes read (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
static esp_err_t priv_pstar_bus_uart_read(const pstar_bus_manager_t* manager,
                                          const char*                name,
                                          uint8_t*                   data, 
                                          size_t                     len, 
                                          size_t*                    bytes_read)
{
  /* Validate input */
  if (!manager || !name || !data || len == 0) {
    log_error(BUS_UART_TAG, 
              "Read Error", 
              "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_UART_TAG, 
              "Read Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is a UART bus */
  if (bus_config->type != k_pstar_bus_type_uart) {
    log_error(BUS_UART_TAG, 
              "Read Error", 
              "Bus '%s' is not a UART bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(BUS_UART_TAG, 
              "Read Error", 
              "Bus '%s' is not initialized", 
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Get UART-specific configuration */
  uart_port_t port = bus_config->config.uart.port;
  
  /* Read data from UART with timeout */
  size_t available_bytes;
  esp_err_t result = uart_get_buffered_data_len(port, &available_bytes);
  if (result != ESP_OK) {
    log_error(BUS_UART_TAG, 
              "Read Error", 
              "Failed to get available data length: %s", 
              esp_err_to_name(result));
    return result;
  }
  
  /* If no data is available, wait for some data to arrive */
  if (available_bytes == 0) {
    log_debug(BUS_UART_TAG, 
              "Read Wait", 
              "No data available, waiting for data on UART port %d", 
              port);
    
    /* Get configurable timeout value (default to 1000ms if not specified) */
    const uint32_t timeout_ms = 1000;
    
    /* Wait for data with configurable timeout */
    int read_bytes = uart_read_bytes(port, data, len, pdMS_TO_TICKS(timeout_ms));
    if (read_bytes < 0) {
      log_error(BUS_UART_TAG, 
                "Read Error", 
                "Failed to read data from UART port %d", 
                port);
      return ESP_FAIL;
    }
    
    if (read_bytes == 0) {
      log_warn(BUS_UART_TAG, 
               "Read Timeout", 
               "Timeout (%lu ms) while waiting for data on UART port %d", 
               timeout_ms, 
               port);
      
      /* Set bytes read to 0 if provided */
      if (bytes_read) {
        *bytes_read = 0;
      }
      
      return ESP_ERR_TIMEOUT;
    }
    
    log_debug(BUS_UART_TAG, 
              "Read Success", 
              "Successfully read %d bytes from UART bus '%s' (port %d)",
              read_bytes, 
              name, 
              port);
    
    /* Set bytes read if provided */
    if (bytes_read) {
      *bytes_read = read_bytes;
    }
    
    /* Call callback if available */
    if (bus_config->config.uart.callbacks.on_data_received) {
      /* Create event structure */
      pstar_bus_event_t event = {
        .bus_type  = k_pstar_bus_type_uart,
        .bus_name  = name,
        .data.uart = {
          .port     = port,
          .is_write = false,
          .len      = read_bytes
        }
      };
      
      /* Call the callback */
      bus_config->config.uart.callbacks.on_data_received(&event, 
                                                         data, 
                                                         read_bytes, 
                                                         bus_config->user_ctx);
    }
    
    return ESP_OK;
  }
  
  /* Data is available, read what we can */
  size_t to_read = (available_bytes < len) ? available_bytes : len;
  int read_bytes = uart_read_bytes(port, data, to_read, 0);
  if (read_bytes < 0) {
    log_error(BUS_UART_TAG, 
              "Read Error", 
              "Failed to read data from UART port %d", 
              port);
    return ESP_FAIL;
  }
  
  log_debug(BUS_UART_TAG, 
            "Read Success", 
            "Successfully read %d bytes from UART bus '%s' (port %d)",
            read_bytes, 
            name, 
            port);
  
  /* Set bytes read if provided */
  if (bytes_read) {
    *bytes_read = read_bytes;
  }
  
  /* Call callback if available */
  if (bus_config->config.uart.callbacks.on_data_received) {
    /* Create event structure */
    pstar_bus_event_t event = {
      .bus_type  = k_pstar_bus_type_uart,
      .bus_name  = name,
      .data.uart = {
        .port     = port,
        .is_write = false,
        .len      = read_bytes
      }
    };
    
    /* Call the callback */
    bus_config->config.uart.callbacks.on_data_received(&event, 
                                                       data, 
                                                       read_bytes, 
                                                       bus_config->user_ctx);
  }
  
  return ESP_OK;
}
