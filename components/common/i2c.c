/* components/common/i2c.c */

#include "common/i2c.h"
#include "common/bus_manager.h"
#include "driver/i2c.h"
#include "log_handler.h"

/* Constants ******************************************************************/

const uint32_t i2c_timeout_ticks = pdMS_TO_TICKS(1000);

/* Private Functions **********************************************************/

esp_err_t priv_i2c_init(uint8_t           scl_io, 
                        uint8_t           sda_io, 
                        uint32_t          freq_hz,
                        i2c_port_t        i2c_bus, 
                        const char* const tag)
{
  /* Use the bus manager to initialize the I2C bus */
  esp_err_t err = bus_manager_i2c_init(scl_io, sda_io, freq_hz, i2c_bus);
  
  if (err != ESP_OK) {
    log_error(tag, 
              "Init Error", 
              "Failed to initialize I2C bus %d: %s", 
              i2c_bus,
              esp_err_to_name(err));
  }
  
  return err;
}

esp_err_t priv_i2c_write_byte(uint8_t           data, 
                              i2c_port_t        i2c_bus,
                              uint8_t           i2c_address, 
                              const char* const tag)
{
  /* Create an I2C command link handle */
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  /* Start I2C communication */
  i2c_master_start(cmd);

  /* Send the I2C address with the write flag */
  i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, true);

  /* Write the data byte to the target device */
  i2c_master_write_byte(cmd, data, true);

  /* Stop I2C communication */
  i2c_master_stop(cmd);

  /* Execute the I2C command */
  esp_err_t ret = i2c_master_cmd_begin(i2c_bus, cmd, i2c_timeout_ticks);

  /* Delete the command link after execution */
  i2c_cmd_link_delete(cmd);

  /* Check for errors in the I2C command */
  if (ret != ESP_OK) {
    log_error(tag, 
              "Write Error", 
              "Failed to write byte 0x%02X to address 0x%02X: %s", 
              data, 
              i2c_address, 
              esp_err_to_name(ret));
  }

  return ret; /* Return the error status or ESP_OK */
}

esp_err_t priv_i2c_read_bytes(uint8_t*          data, 
                              size_t            len, 
                              i2c_port_t        i2c_bus,
                              uint8_t           i2c_address, 
                              const char* const tag)
{
  /* Create an I2C command link handle */
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  /* Start I2C communication */
  i2c_master_start(cmd);

  /* Send the I2C address with the read flag */
  i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_READ, true);

  /* Read multiple bytes if length is greater than 1 */
  if (len > 1) {
    /* Read len-1 bytes with an ACK after each byte */
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  }

  /* Read the last byte with a NACK to signal the end of the transmission */
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);

  /* Stop I2C communication */
  i2c_master_stop(cmd);

  /* Execute the I2C command */
  esp_err_t ret = i2c_master_cmd_begin(i2c_bus, cmd, i2c_timeout_ticks);

  /* Delete the command link after execution */
  i2c_cmd_link_delete(cmd);

  /* Check for errors in the I2C command */
  if (ret != ESP_OK) {
    log_error(tag, 
              "Read Error", 
              "Failed to read %u bytes from address 0x%02X: %s", 
              len, 
              i2c_address, 
              esp_err_to_name(ret));
  }

  return ret; /* Return the error status or ESP_OK */
}

esp_err_t priv_i2c_write_reg_byte(uint8_t           reg_addr, 
                                  uint8_t           data,
                                  i2c_port_t        i2c_bus, 
                                  uint8_t           i2c_address,
                                  const char* const tag)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(i2c_bus, cmd, i2c_timeout_ticks);

  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    log_error(tag, 
              "Reg Write Error", 
              "Failed to write 0x%02X to register 0x%02X at address 0x%02X: %s", 
              data, 
              reg_addr, 
              i2c_address, 
              esp_err_to_name(ret));
  }

  return ret;
}

esp_err_t priv_i2c_read_reg_bytes(uint8_t           reg_addr, 
                                  uint8_t*          data, 
                                  size_t            len,
                                  i2c_port_t        i2c_bus, 
                                  uint8_t           i2c_address,
                                  const char* const tag)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  /* Start I2C communication */
  i2c_master_start(cmd);

  /* Send the I2C address with the write flag */
  i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, true);

  /* Write the register address */
  i2c_master_write_byte(cmd, reg_addr, true);

  /* Repeated start for read */
  i2c_master_start(cmd);

  /* Send the I2C address with the read flag */
  i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_READ, true);

  /* Read multiple bytes if length is greater than 1 */
  if (len > 1) {
    /* Read len-1 bytes with an ACK after each byte */
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  }

  /* Read the last byte with a NACK to signal the end of the transmission */
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);

  /* Stop I2C communication */
  i2c_master_stop(cmd);

  /* Execute the I2C command */
  esp_err_t ret = i2c_master_cmd_begin(i2c_bus, cmd, i2c_timeout_ticks);

  /* Delete the command link after execution */
  i2c_cmd_link_delete(cmd);

  /* Check for errors in the I2C command */
  if (ret != ESP_OK) {
    log_error(tag, 
              "Reg Read Error", 
              "Failed to read %u bytes from register 0x%02X at address 0x%02X: %s", 
              len, 
              reg_addr, 
              i2c_address, 
              esp_err_to_name(ret));
  }

  return ret; /* Return the error status or ESP_OK */
}

/**
 * @brief Deinitializes the I2C interface.
 *
 * Uses the bus manager to deinitialize the specified I2C bus.
 *
 * @param[in] i2c_bus I2C bus number to deinitialize.
 * @param[in] tag     Logging tag for error messages.
 *
 * @return
 * - `ESP_OK` on successful deinitialization.
 * - Error codes from `esp_err_t` on failure.
 */
esp_err_t priv_i2c_deinit(i2c_port_t i2c_bus, const char* const tag)
{
  esp_err_t err = bus_manager_i2c_deinit(i2c_bus);
  
  if (err != ESP_OK) {
    log_error(tag, 
              "Deinit Error", 
              "Failed to deinitialize I2C bus %d: %s", 
              i2c_bus,
              esp_err_to_name(err));
  } else {
    log_info(tag,
             "Deinit Success",
             "I2C bus %d deinitialized successfully",
             i2c_bus);
  }
  
  return err;
}

