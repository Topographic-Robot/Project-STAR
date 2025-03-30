/* components/pstar_bus/pstar_bus_uart.c */

#include "pstar_bus_uart.h"

#include "pstar_bus_event_types.h"
#include "pstar_bus_manager.h"
#include "pstar_bus_types.h"
#include "pstar_log_handler.h"

#include <string.h>

/* Constants ******************************************************************/

static const char* TAG = "Bus UART";

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
    log_error(TAG, "Init Error", "UART operations pointer is NULL");
    return;
  }

  ops->write = priv_pstar_bus_uart_write;
  ops->read  = priv_pstar_bus_uart_read;

  log_info(TAG, "Default Ops", "Initialized default UART operations");
}

esp_err_t pstar_bus_uart_write(const pstar_bus_manager_t* manager,
                               const char*                name,
                               const uint8_t*             data,
                               size_t                     len,
                               size_t*                    bytes_written)
{
  /* Validate input */
  if (!manager || !name || !data || len == 0) {
    log_error(TAG, "Write Error", "Invalid parameters: manager, name, data is NULL or len is 0");
    return ESP_ERR_INVALID_ARG;
  }

  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(TAG, "Write Error", "Bus '%s' not found", name);
    return ESP_ERR_NOT_FOUND;
  }

  /* Check if this is a UART bus */
  if (bus_config->type != k_pstar_bus_type_uart) {
    log_error(TAG,
              "Write Error",
              "Bus '%s' is not a UART bus (type: %s)",
              name,
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if ops function pointer is valid */
  if (!bus_config->config.uart.ops.write) {
    log_error(TAG, "Write Error", "No write operation defined for UART bus '%s'", name);
    return ESP_ERR_NOT_SUPPORTED;
  }

  /* Call the write operation function */
  return bus_config->config.uart.ops.write(manager, name, data, len, bytes_written);
}

esp_err_t pstar_bus_uart_read(const pstar_bus_manager_t* manager,
                              const char*                name,
                              uint8_t*                   data,
                              size_t                     len,
                              size_t*                    bytes_read)
{
  /* Validate input */
  if (!manager || !name || !data || len == 0) {
    log_error(TAG, "Read Error", "Invalid parameters: manager, name, data is NULL or len is 0");
    return ESP_ERR_INVALID_ARG;
  }

  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(TAG, "Read Error", "Bus '%s' not found", name);
    return ESP_ERR_NOT_FOUND;
  }

  /* Check if this is a UART bus */
  if (bus_config->type != k_pstar_bus_type_uart) {
    log_error(TAG,
              "Read Error",
              "Bus '%s' is not a UART bus (type: %s)",
              name,
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if ops function pointer is valid */
  if (!bus_config->config.uart.ops.read) {
    log_error(TAG, "Read Error", "No read operation defined for UART bus '%s'", name);
    return ESP_ERR_NOT_SUPPORTED;
  }

  /* Call the read operation function */
  return bus_config->config.uart.ops.read(manager, name, data, len, bytes_read);
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

  /* TODO: Validate input - Already done in public function */
  /* if (!manager || !name || !data || len == 0) { ... } */

  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    /* Logged in public function */
    return ESP_ERR_NOT_FOUND;
  }

  /* Check if this is a UART bus - Already done in public function */
  /* TODO: if (bus_config->type != k_pstar_bus_type_uart) { ... } */

  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(TAG, "Write Error", "Bus '%s' is not initialized", name);
    return ESP_ERR_INVALID_STATE;
  }

  /* Get UART-specific configuration */
  uart_port_t port = bus_config->config.uart.port;

  /* Write data to UART */
  int written = uart_write_bytes(port, (const char*)data, len);
  if (written < 0) {
    log_error(TAG, "Write Error", "Failed to write data to UART port %d", port);
    /* Call error callback if available */
    if (bus_config->config.uart.callbacks.on_error) {
      pstar_bus_event_t event = {.bus_type  = k_pstar_bus_type_uart,
                                 .bus_name  = name,
                                 .data.uart = {.port = port, .is_write = true, .len = 0}};
      bus_config->config.uart.callbacks.on_error(&event, ESP_FAIL, bus_config->user_ctx);
    }
    return ESP_FAIL;
  }
  /* Check if all bytes were written (uart_write_bytes might return < len) */
  if ((size_t)written != len) {
    log_warn(TAG,
             "Write Incomplete",
             "uart_write_bytes wrote %d of %zu bytes to port %d",
             written,
             len,
             port);
  }

  /* Ensure all data is transmitted before returning */
  result = uart_wait_tx_done(port, pdMS_TO_TICKS(1000));
  if (result != ESP_OK) {
    log_error(TAG,
              "Write Error",
              "Failed to wait for UART transmission completion on port %d: %s",
              port,
              esp_err_to_name(result));
    /* Call error callback if available */
    if (bus_config->config.uart.callbacks.on_error) {
      pstar_bus_event_t event = {
        .bus_type  = k_pstar_bus_type_uart,
        .bus_name  = name,
        .data.uart = {.port = port, .is_write = true, .len = (size_t)written}};
      bus_config->config.uart.callbacks.on_error(&event, result, bus_config->user_ctx);
    }
    return result;
  }

  log_debug(TAG,
            "Write Success",
            "Successfully wrote %d bytes to UART bus '%s' (port %d)",
            written,
            name,
            port);

  /* Set bytes written if provided */
  if (bytes_written) {
    *bytes_written = (size_t)written;
  }

  /* Call callback if available */
  if (bus_config->config.uart.callbacks.on_data_sent) {
    /* Create event structure */
    pstar_bus_event_t event = {
      .bus_type  = k_pstar_bus_type_uart,
      .bus_name  = name,
      .data.uart = {.port = port, .is_write = true, .len = (size_t)written}};

    /* Call the callback */
    bus_config->config.uart.callbacks.on_data_sent(&event, (size_t)written, bus_config->user_ctx);
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
  /* TODO: Validate input - Already done in public function */
  /* if (!manager || !name || !data || len == 0) { ... } */

  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    return ESP_ERR_NOT_FOUND;
  }

  /* Check if this is a UART bus - Already done in public function */
  /* TODO: if (bus_config->type != k_pstar_bus_type_uart) { ... } */

  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(TAG, "Read Error", "Bus '%s' is not initialized", name);
    return ESP_ERR_INVALID_STATE;
  }

  /* Get UART-specific configuration */
  uart_port_t port = bus_config->config.uart.port;

  /* Read data from UART with timeout */
  size_t    available_bytes = 0;
  esp_err_t result          = uart_get_buffered_data_len(port, &available_bytes);
  if (result != ESP_OK) {
    log_error(TAG,
              "Read Error",
              "Failed to get available data length on port %d: %s",
              port,
              esp_err_to_name(result));
    /* Call error callback if available */
    if (bus_config->config.uart.callbacks.on_error) {
      pstar_bus_event_t event = {.bus_type  = k_pstar_bus_type_uart,
                                 .bus_name  = name,
                                 .data.uart = {.port = port, .is_write = false, .len = 0}};
      bus_config->config.uart.callbacks.on_error(&event, result, bus_config->user_ctx);
    }
    return result;
  }

  /* Determine how many bytes to actually read */
  size_t to_read = (available_bytes < len) ? available_bytes : len;

  int read_bytes = 0;
  if (to_read > 0) {
    /* Read available data without blocking */
    read_bytes = uart_read_bytes(port, data, to_read, 0); /* Use 0 timeout */
  } else {
    /* No data available, wait for data with timeout */
    const uint32_t timeout_ms = 1000;
    log_debug(TAG,
              "Read Wait",
              "No data available, waiting up to %lu ms on UART port %d",
              timeout_ms,
              port);
    read_bytes = uart_read_bytes(port, data, len, pdMS_TO_TICKS(timeout_ms));

    if (read_bytes == 0) {
      log_debug(TAG,
                "Read Timeout",
                "Timeout (%lu ms) while waiting for data on UART port %d",
                timeout_ms,
                port);
      /* Call error callback if available, with ESP_ERR_TIMEOUT */
      if (bus_config->config.uart.callbacks.on_error) {
        pstar_bus_event_t event = {.bus_type  = k_pstar_bus_type_uart,
                                   .bus_name  = name,
                                   .data.uart = {.port = port, .is_write = false, .len = 0}};
        bus_config->config.uart.callbacks.on_error(&event, ESP_ERR_TIMEOUT, bus_config->user_ctx);
      }
      /* Set bytes read to 0 if provided */
      if (bytes_read) {
        *bytes_read = 0;
      }
      return ESP_ERR_TIMEOUT;
    }
  }

  if (read_bytes < 0) {
    log_error(TAG, "Read Error", "uart_read_bytes failed for UART port %d", port);
    /* Call error callback if available */
    if (bus_config->config.uart.callbacks.on_error) {
      pstar_bus_event_t event = {.bus_type  = k_pstar_bus_type_uart,
                                 .bus_name  = name,
                                 .data.uart = {.port = port, .is_write = false, .len = 0}};
      bus_config->config.uart.callbacks.on_error(&event, ESP_FAIL, bus_config->user_ctx);
    }
    return ESP_FAIL;
  }

  log_debug(TAG,
            "Read Success",
            "Successfully read %d bytes from UART bus '%s' (port %d)",
            read_bytes,
            name,
            port);

  /* Set bytes read if provided */
  if (bytes_read) {
    *bytes_read = (size_t)read_bytes;
  }

  /* Call callback if available and bytes were read */
  if (read_bytes > 0 && bus_config->config.uart.callbacks.on_data_received) {
    /* Create event structure */
    pstar_bus_event_t event = {
      .bus_type  = k_pstar_bus_type_uart,
      .bus_name  = name,
      .data.uart = {.port = port, .is_write = false, .len = (size_t)read_bytes}};

    /* Call the callback */
    bus_config->config.uart.callbacks.on_data_received(&event,
                                                       data,
                                                       (size_t)read_bytes,
                                                       bus_config->user_ctx);
  }

  return ESP_OK;
}
