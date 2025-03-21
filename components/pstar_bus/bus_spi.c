/* components/pstar_bus/bus_spi.c */

#include "bus_spi.h"
#include "bus_manager.h"
#include "log_handler.h"
#include "bus_types.h"
#include "bus_event.h"
#include <string.h>

/* Constants ******************************************************************/

#define BUS_SPI_TAG ("Bus SPI")

/* Private Functions **********************************************************/

static esp_err_t priv_pstar_bus_spi_write(const pstar_bus_manager_t* manager,
                                          const char*                name,
                                          const uint8_t*             data, 
                                          size_t                     len, 
                                          size_t*                    bytes_written);

static esp_err_t priv_pstar_bus_spi_read(const pstar_bus_manager_t* manager,
                                         const char*                name,
                                         uint8_t*                   data, 
                                         size_t                     len, 
                                         size_t*                    bytes_read);

/* Public Functions ***********************************************************/

void pstar_bus_spi_init_default_ops(pstar_spi_ops_t* ops)
{
  if (ops == NULL) {
    log_error(BUS_SPI_TAG, 
              "Init Error", 
              "SPI operations pointer is NULL");
    return;
  }
  
  ops->write = priv_pstar_bus_spi_write;
  ops->read  = priv_pstar_bus_spi_read;
  
  log_info(BUS_SPI_TAG, 
           "Default Ops", 
           "Initialized default SPI operations");
}

esp_err_t pstar_bus_spi_write(const pstar_bus_manager_t* manager,
                              const char*                name,
                              const uint8_t*             data, 
                              size_t                     len, 
                              size_t*                    bytes_written)
{
  /* Validate input */
  if (!manager || !name) {
    log_error(BUS_SPI_TAG, 
              "Write Error", 
              "Invalid parameters: manager or name is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_SPI_TAG, 
              "Write Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is an SPI bus */
  if (bus_config->type != k_pstar_bus_type_spi) {
    log_error(BUS_SPI_TAG, 
              "Write Error", 
              "Bus '%s' is not an SPI bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Call the write operation function */
  if (bus_config->config.spi.ops.write) {
    return bus_config->config.spi.ops.write(manager, 
                                            name, 
                                            data, 
                                            len, 
                                            bytes_written);
  } else {
    log_error(BUS_SPI_TAG, 
              "Write Error", 
              "No write operation defined for SPI bus '%s'", 
              name);
    return ESP_ERR_NOT_SUPPORTED;
  }
}

esp_err_t pstar_bus_spi_read(const pstar_bus_manager_t* manager,
                             const char*                name,
                             uint8_t*                   data, 
                             size_t                     len, 
                             size_t*                    bytes_read)
{
  /* Validate input */
  if (!manager || !name) {
    log_error(BUS_SPI_TAG, 
              "Read Error", 
              "Invalid parameters: manager or name is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_SPI_TAG, 
              "Read Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is an SPI bus */
  if (bus_config->type != k_pstar_bus_type_spi) {
    log_error(BUS_SPI_TAG, 
              "Read Error", 
              "Bus '%s' is not an SPI bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Call the read operation function */
  if (bus_config->config.spi.ops.read) {
    return bus_config->config.spi.ops.read(manager, name, data, len, bytes_read);
  } else {
    log_error(BUS_SPI_TAG, 
              "Read Error", 
              "No read operation defined for SPI bus '%s'", 
              name);
    return ESP_ERR_NOT_SUPPORTED;
  }
}

/* Private Functions **********************************************************/

/**
 * @brief Write data to an SPI device using the default implementation.
 * 
 * @param[in]  manager       Pointer to the bus manager.
 * @param[in]  name          Name of the SPI bus.
 * @param[in]  data          Data to write.
 * @param[in]  len           Length of data to write.
 * @param[out] bytes_written Pointer to store the number of bytes written (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
static esp_err_t priv_pstar_bus_spi_write(const pstar_bus_manager_t* manager,
                                          const char*                name,
                                          const uint8_t*             data, 
                                          size_t                     len, 
                                          size_t*                    bytes_written)
{
  esp_err_t           result        = ESP_OK;
  bool                device_added  = false;
  spi_device_handle_t device_handle = NULL;
  
  /* Validate input */
  if (!manager || !name || !data || len == 0) {
    log_error(BUS_SPI_TAG, 
              "Write Error", 
              "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_SPI_TAG, 
              "Write Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is an SPI bus */
  if (bus_config->type != k_pstar_bus_type_spi) {
    log_error(BUS_SPI_TAG, 
              "Write Error", 
              "Bus '%s' is not an SPI bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(BUS_SPI_TAG, 
              "Write Error", 
              "Bus '%s' is not initialized", 
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Get SPI-specific configuration */
  spi_host_device_t host = bus_config->config.spi.host;
  
  /* Set up transaction */
  spi_transaction_t transaction = {
    .length    = len * 8,    /* Length in bits */
    .tx_buffer = data,       /* Data to send */
    .rx_buffer = NULL,       /* No receive buffer for write operation */
    .user      = bus_config, /* Store bus_config for callback context */
  };
  
  /* Execute transaction */
  if (bus_config->handle) {
    device_handle = (spi_device_handle_t)bus_config->handle;
    result        = spi_device_transmit(device_handle, &transaction);
  } else {
    /* Add device to SPI bus */
    result = spi_bus_add_device(host, 
                                &bus_config->config.spi.dev_config, 
                                &device_handle);
    
    if (result != ESP_OK) {
      log_error(BUS_SPI_TAG, 
                "Device Error", 
                "Failed to add SPI device to bus '%s': %s", 
                name, 
                esp_err_to_name(result));
      return result;
    }
    
    /* Mark device as added for cleanup in case of error */
    device_added = true;
    
    /* Store the handle for future use */
    bus_config->handle = device_handle;
    
    log_debug(BUS_SPI_TAG, 
              "Device Added", 
              "Added SPI device to bus '%s'", 
              name);
              
    /* Execute transaction */
    result = spi_device_transmit(device_handle, &transaction);
  }
  
  if (result != ESP_OK) {
    log_error(BUS_SPI_TAG, 
              "Write Error", 
              "Failed to execute SPI transaction on bus '%s' (host %d): %s", 
              name, 
              host, 
              esp_err_to_name(result));
    
    /* If we just added the device and the transaction failed, clean up */
    if (device_added) {
      esp_err_t remove_err = spi_bus_remove_device(device_handle);
      if (remove_err != ESP_OK) {
        log_error(BUS_SPI_TAG,
                  "Device Error",
                  "Failed to remove SPI device after failed transaction: %s",
                  esp_err_to_name(remove_err));
      } else {
        log_debug(BUS_SPI_TAG,
                  "Device Removed",
                  "Successfully removed SPI device after failed transaction");
      }
      
      /* Also free the bus if we were responsible for adding the device */
      esp_err_t free_err = spi_bus_free(host);
      if (free_err != ESP_OK) {
        log_error(BUS_SPI_TAG,
                 "Bus Error",
                 "Failed to free SPI bus after failed transaction: %s",
                 esp_err_to_name(free_err));
      } else {
        log_debug(BUS_SPI_TAG,
                 "Bus Freed",
                 "Successfully freed SPI bus after failed transaction");
      }
      
      /* Always clear the handle pointer, even if removal fails */
      bus_config->handle = NULL;
    }
    
    return result;
  } else {
    log_debug(BUS_SPI_TAG, 
              "Write Success", 
              "Successfully wrote %zu bytes to SPI bus '%s' (host %d)",
              len, 
              name, 
              host);
    
    /* Set bytes written if provided */
    if (bytes_written) {
      *bytes_written = len;
    }
    
    /* Call callback if available */
    if (bus_config->config.spi.callbacks.on_transfer_complete) {
      /* Create event structure */
      pstar_bus_event_t event = {
        .bus_type = k_pstar_bus_type_spi,
        .bus_name = name,
        .data.spi = {
          .host     = host,
          .is_write = true,
          .len      = len
        }
      };
      
      /* Call the callback */
      bus_config->config.spi.callbacks.on_transfer_complete(&event, 
                                                            bus_config->user_ctx);
    }
  }
  
  return result;
}

/**
 * @brief Read data from an SPI device using the default implementation.
 * 
 * @param[in]  manager    Pointer to the bus manager.
 * @param[in]  name       Name of the SPI bus.
 * @param[out] data       Buffer to read data into.
 * @param[in]  len        Length of data to read.
 * @param[out] bytes_read Pointer to store the number of bytes read (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
static esp_err_t priv_pstar_bus_spi_read(const pstar_bus_manager_t* manager,
                                         const char*                name,
                                         uint8_t*                   data, 
                                         size_t                     len, 
                                         size_t*                    bytes_read)
{
  esp_err_t           result       = ESP_OK;
  bool                device_added = false;
  spi_device_handle_t device_handle = NULL;
  
  /* Validate input */
  if (!manager || !name || !data || len == 0) {
    log_error(BUS_SPI_TAG, 
              "Read Error", 
              "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_SPI_TAG, 
              "Read Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is an SPI bus */
  if (bus_config->type != k_pstar_bus_type_spi) {
    log_error(BUS_SPI_TAG, 
              "Read Error", 
              "Bus '%s' is not an SPI bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(BUS_SPI_TAG, 
              "Read Error", 
              "Bus '%s' is not initialized", 
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Get SPI-specific configuration */
  spi_host_device_t host = bus_config->config.spi.host;
  
  /* Set up transaction */
  spi_transaction_t transaction = {
    .length    = len * 8,    /* Length in bits */
    .tx_buffer = NULL,       /* No transmit buffer for read operation */
    .rx_buffer = data,       /* Data buffer to read into */
    .user      = bus_config, /* Store bus_config for callback context */
  };
  
  /* Execute transaction */
  if (bus_config->handle) {
    device_handle = (spi_device_handle_t)bus_config->handle;
    result        = spi_device_transmit(device_handle, &transaction);
  } else {
    /* Add device to SPI bus */
    result = spi_bus_add_device(host, 
                                &bus_config->config.spi.dev_config, 
                                &device_handle);
    
    if (result != ESP_OK) {
      log_error(BUS_SPI_TAG, 
                "Device Error", 
                "Failed to add SPI device to bus '%s': %s", 
                name, 
                esp_err_to_name(result));
      return result;
    }
    
    /* Mark device as added for cleanup in case of error */
    device_added = true;
    
    /* Store the handle for future use */
    bus_config->handle = device_handle;
    
    log_debug(BUS_SPI_TAG, 
              "Device Added", 
              "Added SPI device to bus '%s'", 
              name);
              
    /* Execute transaction */
    result = spi_device_transmit(device_handle, &transaction);
  }
  
  if (result != ESP_OK) {
    log_error(BUS_SPI_TAG, 
              "Read Error", 
              "Failed to execute SPI transaction on bus '%s' (host %d): %s", 
              name, 
              host, 
              esp_err_to_name(result));
    
    /* If we just added the device and the transaction failed, clean up */
    if (device_added) {
      esp_err_t remove_err = spi_bus_remove_device(device_handle);
      if (remove_err != ESP_OK) {
        log_error(BUS_SPI_TAG,
                  "Device Error",
                  "Failed to remove SPI device after failed transaction: %s",
                  esp_err_to_name(remove_err));
      } else {
        log_debug(BUS_SPI_TAG,
                  "Device Removed",
                  "Successfully removed SPI device after failed transaction");
      }
      
      /* Also free the bus if we were responsible for adding the device */
      esp_err_t free_err = spi_bus_free(host);
      if (free_err != ESP_OK) {
        log_error(BUS_SPI_TAG,
                 "Bus Error",
                 "Failed to free SPI bus after failed transaction: %s",
                 esp_err_to_name(free_err));
      } else {
        log_debug(BUS_SPI_TAG,
                 "Bus Freed",
                 "Successfully freed SPI bus after failed transaction");
      }
      
      /* Always clear the handle pointer, even if removal fails */
      bus_config->handle = NULL;
    }
    
    return result;
  } else {
    log_debug(BUS_SPI_TAG, 
              "Read Success", 
              "Successfully read %zu bytes from SPI bus '%s' (host %d)",
              len, 
              name, 
              host);
    
    /* Set bytes read if provided */
    if (bytes_read) {
      *bytes_read = len;
    }
    
    /* Call callback if available */
    if (bus_config->config.spi.callbacks.on_transfer_complete) {
      /* Create event structure */
      pstar_bus_event_t event = {
        .bus_type = k_pstar_bus_type_spi,
        .bus_name = name,
        .data.spi = {
          .host     = host,
          .is_write = false,
          .len      = len
        }
      };
      
      /* Call the callback */
      bus_config->config.spi.callbacks.on_transfer_complete(&event, 
                                                            bus_config->user_ctx);
    }
  }
  
  return result;
}
