/* components/pstar_bus/bus_i2c.c */

#include "bus_i2c.h"
#include "bus_manager.h"
#include "log_handler.h"
#include "bus_types.h"
#include "bus_event.h"
#include <string.h>

/* Constants ******************************************************************/

static const char* const bus_i2c_tag = "Bus I2C";

/* Private Function Prototypes ************************************************/

static esp_err_t priv_pstar_bus_i2c_write(const pstar_bus_manager_t* manager,
                                          const char*                name,
                                          const uint8_t*             data, 
                                          size_t                     len, 
                                          uint8_t                    reg_addr, 
                                          size_t*                    bytes_written);

static esp_err_t priv_pstar_bus_i2c_read(const pstar_bus_manager_t* manager,
                                         const char*                name,
                                         uint8_t*                   data, 
                                         size_t                     len, 
                                         uint8_t                    reg_addr, 
                                         size_t*                    bytes_read);

/* Public Functions ***********************************************************/

void pstar_bus_i2c_init_default_ops(pstar_i2c_ops_t* ops)
{
  if (ops == NULL) {
    log_error(bus_i2c_tag, 
              "Init Error", 
              "I2C operations pointer is NULL");
    return;
  }
  
  ops->write = priv_pstar_bus_i2c_write;
  ops->read  = priv_pstar_bus_i2c_read;
  
  log_info(bus_i2c_tag, 
           "Default Ops", 
           "Initialized default I2C operations");
}

esp_err_t pstar_bus_i2c_write(const pstar_bus_manager_t* manager,
                              const char*                name,
                              const uint8_t*             data, 
                              size_t                     len, 
                              uint8_t                    reg_addr, 
                              size_t*                    bytes_written)
{
  /* Validate input */
  if (!manager || !name) {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "Invalid parameters: manager or name is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is an I2C bus */
  if (bus_config->type != k_pstar_bus_type_i2c) {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "Bus '%s' is not an I2C bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Call the write operation function */
  if (bus_config->config.i2c.ops.write) {
    return bus_config->config.i2c.ops.write(manager, 
                                            name, 
                                            data, 
                                            len, 
                                            reg_addr, 
                                            bytes_written);
  } else {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "No write operation defined for I2C bus '%s'", 
              name);
    return ESP_ERR_NOT_SUPPORTED;
  }
}

esp_err_t pstar_bus_i2c_read(const pstar_bus_manager_t* manager,
                             const char*                name,
                             uint8_t*                   data, 
                             size_t                     len, 
                             uint8_t                    reg_addr, 
                             size_t*                    bytes_read)
{
  /* Validate input */
  if (!manager || !name) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Invalid parameters: manager or name is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is an I2C bus */
  if (bus_config->type != k_pstar_bus_type_i2c) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Bus '%s' is not an I2C bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Call the read operation function */
  if (bus_config->config.i2c.ops.read) {
    return bus_config->config.i2c.ops.read(manager, 
                                           name, 
                                           data, 
                                           len, 
                                           reg_addr, 
                                           bytes_read);
  } else {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "No read operation defined for I2C bus '%s'", 
              name);
    return ESP_ERR_NOT_SUPPORTED;
  }
}


/* Private Functions **********************************************************/

/**
 * @brief Write data to an I2C device using the default implementation.
 * 
 * @param[in]  manager       Pointer to the bus manager.
 * @param[in]  name          Name of the I2C bus.
 * @param[in]  data          Data to write.
 * @param[in]  len           Length of data to write.
 * @param[in]  reg_addr      Register address to write to.
 * @param[out] bytes_written Pointer to store the number of bytes written (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
static esp_err_t priv_pstar_bus_i2c_write(const pstar_bus_manager_t* manager,
                                          const char*                name,
                                          const uint8_t*             data, 
                                          size_t                     len, 
                                          uint8_t                    reg_addr, 
                                          size_t*                    bytes_written)
{
  esp_err_t result = ESP_OK;
  
  /* Validate input */
  if (!manager || !name || !data || len == 0) {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is an I2C bus */
  if (bus_config->type != k_pstar_bus_type_i2c) {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "Bus '%s' is not an I2C bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "Bus '%s' is not initialized", 
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Get I2C-specific configuration */
  i2c_port_t port    = bus_config->config.i2c.port;
  uint8_t    address = bus_config->config.i2c.address;
  
  /* Initialize command link */
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if (cmd == NULL) {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "Failed to create I2C command link");
    return ESP_ERR_NO_MEM;
  }
  
  /* Start condition */
  if ((result = i2c_master_start(cmd)) != ESP_OK) {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "Failed to add START condition: %s", 
              esp_err_to_name(result));
    i2c_cmd_link_delete(cmd);
    return result;
  }
  
  /* Send device address (write mode) */
  if ((result = i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true)) != ESP_OK) {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "Failed to write device address: %s", 
              esp_err_to_name(result));
    i2c_cmd_link_delete(cmd);
    return result;
  }
  
  /* Send register address */
  if ((result = i2c_master_write_byte(cmd, reg_addr, true)) != ESP_OK) {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "Failed to write register address: %s", 
              esp_err_to_name(result));
    i2c_cmd_link_delete(cmd);
    return result;
  }
  
  /* Send data */
  if ((result = i2c_master_write(cmd, data, len, true)) != ESP_OK) {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "Failed to write data: %s", 
              esp_err_to_name(result));
    i2c_cmd_link_delete(cmd);
    return result;
  }
  
  /* Stop condition */
  if ((result = i2c_master_stop(cmd)) != ESP_OK) {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "Failed to add STOP condition: %s", 
              esp_err_to_name(result));
    i2c_cmd_link_delete(cmd);
    return result;
  }
  
  /* Execute command */
  result = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(1000));
  if (result != ESP_OK) {
    log_error(bus_i2c_tag, 
              "Write Error", 
              "Failed to execute I2C transaction on bus '%s' (port %d, addr 0x%02X): %s", 
              name, 
              port, 
              address, 
              esp_err_to_name(result));
  } else {
    log_debug(bus_i2c_tag, 
              "Write Success", 
              "Successfully wrote %zu bytes to I2C bus '%s' (port %d, addr 0x%02X, reg 0x%02X)",
              len, 
              name, 
              port, 
              address, 
              reg_addr);
    
    /* Set bytes written if provided */
    if (bytes_written) {
      *bytes_written = len;
    }
    
    /* Call callback if available */
    if (bus_config->config.i2c.callbacks.on_transfer_complete) {
      /* Create event structure */
      pstar_bus_event_t event = {
        .bus_type = k_pstar_bus_type_i2c,
        .bus_name = name,
        .data.i2c = {
          .port     = port,
          .address  = address,
          .is_write = true,
          .len      = len
        }
      };
      
      /* Call the callback */
      bus_config->config.i2c.callbacks.on_transfer_complete(&event, 
                                                            bus_config->user_ctx);
    }
  }
  
  /* Clean up */
  i2c_cmd_link_delete(cmd);
  
  return result;
}

/**
 * @brief Read data from an I2C device using the default implementation.
 * 
 * @param[in]  manager    Pointer to the bus manager.
 * @param[in]  name       Name of the I2C bus.
 * @param[out] data       Buffer to read data into.
 * @param[in]  len        Length of data to read.
 * @param[in]  reg_addr   Register address to read from.
 * @param[out] bytes_read Pointer to store the number of bytes read (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
static esp_err_t priv_pstar_bus_i2c_read(const pstar_bus_manager_t* manager,
                                         const char*                name,
                                         uint8_t*                   data, 
                                         size_t                     len, 
                                         uint8_t                    reg_addr, 
                                         size_t*                    bytes_read)
{
  esp_err_t result = ESP_OK;
  
  /* Validate input */
  if (!manager || !name || !data || len == 0) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is an I2C bus */
  if (bus_config->type != k_pstar_bus_type_i2c) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Bus '%s' is not an I2C bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Bus '%s' is not initialized", 
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Get I2C-specific configuration */
  i2c_port_t port    = bus_config->config.i2c.port;
  uint8_t    address = bus_config->config.i2c.address;
  
  /* Step 1: Write register address */
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if (cmd == NULL) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Failed to create I2C command link");
    return ESP_ERR_NO_MEM;
  }
  
  /* Start condition */
  if ((result = i2c_master_start(cmd)) != ESP_OK) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Failed to add START condition: %s", 
              esp_err_to_name(result));
    i2c_cmd_link_delete(cmd);
    return result;
  }
  
  /* Send device address (write mode) */
  if ((result = i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true)) != ESP_OK) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Failed to write device address: %s", 
              esp_err_to_name(result));
    i2c_cmd_link_delete(cmd);
    return result;
  }
  
  /* Send register address */
  if ((result = i2c_master_write_byte(cmd, reg_addr, true)) != ESP_OK) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Failed to write register address: %s", 
              esp_err_to_name(result));
    i2c_cmd_link_delete(cmd);
    return result;
  }
  
  /* Step 2: Read data */
  
  /* Repeated start */
  if ((result = i2c_master_start(cmd)) != ESP_OK) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Failed to add repeated START condition: %s", 
              esp_err_to_name(result));
    i2c_cmd_link_delete(cmd);
    return result;
  }
  
  /* Send device address (read mode) */
  if ((result = i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true)) != ESP_OK) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Failed to write device address (read mode): %s", 
              esp_err_to_name(result));
    i2c_cmd_link_delete(cmd);
    return result;
  }
  
  /* Read data (acknowledge all bytes except the last one) */
  if (len > 1) {
    if ((result = i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK)) != ESP_OK) {
      log_error(bus_i2c_tag, 
                "Read Error", 
                "Failed to read data with ACK: %s", 
                esp_err_to_name(result));
      i2c_cmd_link_delete(cmd);
      return result;
    }
  }
  
  /* Read last byte with NACK */
  if ((result = i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK)) != ESP_OK) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Failed to read last byte with NACK: %s", 
              esp_err_to_name(result));
    i2c_cmd_link_delete(cmd);
    return result;
  }
  
  /* Stop condition */
  if ((result = i2c_master_stop(cmd)) != ESP_OK) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Failed to add STOP condition: %s", 
              esp_err_to_name(result));
    i2c_cmd_link_delete(cmd);
    return result;
  }
  
  /* Execute command */
  result = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(1000));
  if (result != ESP_OK) {
    log_error(bus_i2c_tag, 
              "Read Error", 
              "Failed to execute I2C transaction on bus '%s' (port %d, addr 0x%02X): %s", 
              name, 
              port, 
              address, 
              esp_err_to_name(result));
  } else {
    log_debug(bus_i2c_tag, 
              "Read Success", 
              "Successfully read %zu bytes from I2C bus '%s' (port %d, addr 0x%02X, reg 0x%02X)",
              len, 
              name, 
              port, 
              address, 
              reg_addr);
    
    /* Set bytes read if provided */
    if (bytes_read) {
      *bytes_read = len;
    }
    
    /* Call callback if available */
    if (bus_config->config.i2c.callbacks.on_transfer_complete) {
      /* Create event structure */
      pstar_bus_event_t event = {
        .bus_type = k_pstar_bus_type_i2c,
        .bus_name = name,
        .data.i2c = {
          .port     = port,
          .address  = address,
          .is_write = false,
          .len      = len
        }
      };
      
      /* Call the callback */
      bus_config->config.i2c.callbacks.on_transfer_complete(&event, 
                                                            bus_config->user_ctx);
    }
  }
  
  /* Clean up */
  i2c_cmd_link_delete(cmd);
  
  return result;
}