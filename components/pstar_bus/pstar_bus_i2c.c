/* components/pstar_bus/pstar_bus_i2c.c */

#include "pstar_bus_i2c.h"

#include "pstar_bus_manager.h" /* Needed for pstar_bus_manager_find_bus */
#include "pstar_bus_types.h"   /* Needed for pstar_bus_config_t and union */

#include "freertos/FreeRTOS.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

/* --- Constants --- */

static const char* TAG = "BusI2C";
#define I2C_TIMEOUT_MS 1000 /* Default timeout for I2C operations */

/* --- Private Function Prototypes (Default Ops) --- */

static esp_err_t priv_pstar_bus_i2c_write(const pstar_bus_config_t* config,
                                          const uint8_t*            data,
                                          size_t                    len,
                                          uint8_t                   reg_addr,
                                          size_t*                   bytes_written);

static esp_err_t priv_pstar_bus_i2c_read(const pstar_bus_config_t* config,
                                         uint8_t*                  data,
                                         size_t                    len,
                                         uint8_t                   reg_addr,
                                         size_t*                   bytes_read);

static esp_err_t priv_pstar_bus_i2c_write_command(const pstar_bus_config_t* config,
                                                  uint8_t                   command);

static esp_err_t priv_pstar_bus_i2c_read_raw(const pstar_bus_config_t* config,
                                             uint8_t*                  data,
                                             size_t                    len,
                                             size_t*                   bytes_read);

/* --- Public Functions --- */

void pstar_bus_i2c_init_default_ops(pstar_i2c_ops_t* ops)
{
  if (ops == NULL) {
    ESP_LOGE(TAG, "Cannot initialize NULL ops pointer");
    return;
  }
  ops->write         = priv_pstar_bus_i2c_write;
  ops->read          = priv_pstar_bus_i2c_read;
  ops->write_command = priv_pstar_bus_i2c_write_command;
  ops->read_raw      = priv_pstar_bus_i2c_read_raw;
  ESP_LOGD(TAG, "Default I2C operations initialized");
}

esp_err_t pstar_bus_i2c_write(const pstar_bus_manager_t* manager,
                              const char*                name,
                              const uint8_t*             data,
                              size_t                     len,
                              uint8_t                    reg_addr,
                              size_t*                    bytes_written)
{
  ESP_RETURN_ON_FALSE(manager && name && data,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Manager, name, or data is NULL");
  ESP_RETURN_ON_FALSE(len > 0, ESP_ERR_INVALID_ARG, TAG, "Write length must be > 0");

  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  ESP_RETURN_ON_FALSE(bus_config, ESP_ERR_NOT_FOUND, TAG, "Bus '%s' not found", name);
  ESP_RETURN_ON_FALSE(bus_config->type == k_pstar_bus_type_i2c,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Bus '%s' is not an I2C bus",
                      name);
  /* --- CORRECTED ACCESS --- */
  ESP_RETURN_ON_FALSE(bus_config->proto.i2c.ops.write,
                      ESP_ERR_NOT_SUPPORTED,
                      TAG,
                      "No write op for '%s'",
                      name);

  /* Clear bytes_written initially */
  if (bytes_written) {
    *bytes_written = 0;
  }

  /* --- CORRECTED ACCESS --- */
  return bus_config->proto.i2c.ops.write(bus_config, data, len, reg_addr, bytes_written);
}

esp_err_t pstar_bus_i2c_read(const pstar_bus_manager_t* manager,
                             const char*                name,
                             uint8_t*                   data,
                             size_t                     len,
                             uint8_t                    reg_addr,
                             size_t*                    bytes_read)
{
  ESP_RETURN_ON_FALSE(manager && name && data,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Manager, name, or data is NULL");
  ESP_RETURN_ON_FALSE(len > 0, ESP_ERR_INVALID_ARG, TAG, "Read length must be > 0");

  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  ESP_RETURN_ON_FALSE(bus_config, ESP_ERR_NOT_FOUND, TAG, "Bus '%s' not found", name);
  ESP_RETURN_ON_FALSE(bus_config->type == k_pstar_bus_type_i2c,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Bus '%s' is not an I2C bus",
                      name);
  /* --- CORRECTED ACCESS --- */
  ESP_RETURN_ON_FALSE(bus_config->proto.i2c.ops.read,
                      ESP_ERR_NOT_SUPPORTED,
                      TAG,
                      "No read op for '%s'",
                      name);

  /* Clear bytes_read initially */
  if (bytes_read) {
    *bytes_read = 0;
  }

  /* --- CORRECTED ACCESS --- */
  return bus_config->proto.i2c.ops.read(bus_config, data, len, reg_addr, bytes_read);
}

esp_err_t
pstar_bus_i2c_write_command(const pstar_bus_manager_t* manager, const char* name, uint8_t command)
{
  ESP_RETURN_ON_FALSE(manager && name, ESP_ERR_INVALID_ARG, TAG, "Manager or name is NULL");

  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  ESP_RETURN_ON_FALSE(bus_config, ESP_ERR_NOT_FOUND, TAG, "Bus '%s' not found", name);
  ESP_RETURN_ON_FALSE(bus_config->type == k_pstar_bus_type_i2c,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Bus '%s' is not an I2C bus",
                      name);
  /* --- CORRECTED ACCESS --- */
  ESP_RETURN_ON_FALSE(bus_config->proto.i2c.ops.write_command,
                      ESP_ERR_NOT_SUPPORTED,
                      TAG,
                      "No write_command op for '%s'",
                      name);

  /* --- CORRECTED ACCESS --- */
  return bus_config->proto.i2c.ops.write_command(bus_config, command);
}

esp_err_t pstar_bus_i2c_read_raw(const pstar_bus_manager_t* manager,
                                 const char*                name,
                                 uint8_t*                   data,
                                 size_t                     len,
                                 size_t*                    bytes_read)
{
  ESP_RETURN_ON_FALSE(manager && name && data,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Manager, name, or data is NULL");
  ESP_RETURN_ON_FALSE(len > 0, ESP_ERR_INVALID_ARG, TAG, "Read length must be > 0");

  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  ESP_RETURN_ON_FALSE(bus_config, ESP_ERR_NOT_FOUND, TAG, "Bus '%s' not found", name);
  ESP_RETURN_ON_FALSE(bus_config->type == k_pstar_bus_type_i2c,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Bus '%s' is not an I2C bus",
                      name);
  /* --- CORRECTED ACCESS --- */
  ESP_RETURN_ON_FALSE(bus_config->proto.i2c.ops.read_raw,
                      ESP_ERR_NOT_SUPPORTED,
                      TAG,
                      "No read_raw op for '%s'",
                      name);

  /* Clear bytes_read initially */
  if (bytes_read) {
    *bytes_read = 0;
  }

  /* --- CORRECTED ACCESS --- */
  return bus_config->proto.i2c.ops.read_raw(bus_config, data, len, bytes_read);
}

/* --- Private Functions (Default Ops Implementations) --- */

static esp_err_t priv_pstar_bus_i2c_write(const pstar_bus_config_t* config,
                                          const uint8_t*            data,
                                          size_t                    len,
                                          uint8_t                   reg_addr,
                                          size_t*                   bytes_written)
{
  /* Basic validation (config, data assumed non-NULL, len > 0 by caller) */
  ESP_RETURN_ON_FALSE(config->initialized,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "Bus '%s' is not initialized",
                      config->name);

  /* --- CORRECTED ACCESS --- */
  i2c_port_t       port    = config->proto.i2c.port;
  uint8_t          address = config->proto.i2c.address;
  esp_err_t        ret     = ESP_OK;
  i2c_cmd_handle_t cmd     = NULL;

  cmd = i2c_cmd_link_create();
  ESP_RETURN_ON_FALSE(cmd,
                      ESP_ERR_NO_MEM,
                      TAG,
                      "Failed to create I2C command link for write '%s'",
                      config->name);

  ret = i2c_master_start(cmd);
  ESP_GOTO_ON_ERROR(ret,
                    write_fail,
                    TAG,
                    "Write '%s': START failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  ret = i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
  ESP_GOTO_ON_ERROR(ret,
                    write_fail,
                    TAG,
                    "Write '%s': Address (W) failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  /* Write register address */
  ret = i2c_master_write_byte(cmd, reg_addr, true);
  ESP_GOTO_ON_ERROR(ret,
                    write_fail,
                    TAG,
                    "Write '%s': Reg Addr failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  /* Write data */
  ret = i2c_master_write(cmd, data, len, true);
  ESP_GOTO_ON_ERROR(ret,
                    write_fail,
                    TAG,
                    "Write '%s': Data write failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  ret = i2c_master_stop(cmd);
  ESP_GOTO_ON_ERROR(ret,
                    write_fail,
                    TAG,
                    "Write '%s': STOP failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  ESP_GOTO_ON_ERROR(ret,
                    write_fail,
                    TAG,
                    "Write '%s': CMD Begin failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  /* Success path */
  i2c_cmd_link_delete(cmd);
  cmd = NULL; /* Prevent double delete in fail path */
  ESP_LOGD(TAG,
           "Write Success: %zu bytes to '%s' (Addr 0x%02X, Reg 0x%02X)",
           len,
           config->name,
           address,
           reg_addr);
  if (bytes_written) {
    *bytes_written = len;
  }

  /* Call success callback */
  /* --- CORRECTED ACCESS --- */
  if (config->proto.i2c.callbacks.on_transfer_complete) {
    pstar_bus_event_t event = {
      .bus_type = k_pstar_bus_type_i2c,
      .bus_name = config->name,
      .i2c      = {.port = port, .address = address, .is_write = true, .len = len}};
    /* --- CORRECTED ACCESS --- */
    config->proto.i2c.callbacks.on_transfer_complete(&event, config->user_ctx);
  }
  return ESP_OK;

write_fail:
  if (cmd) {
    i2c_cmd_link_delete(cmd);
  }
  ESP_LOGE(TAG, "Write failed for '%s': %s", config->name, esp_err_to_name(ret));
  /* Call error callback */
  /* --- CORRECTED ACCESS --- */
  if (config->proto.i2c.callbacks.on_error) {
    pstar_bus_event_t event = {.bus_type = k_pstar_bus_type_i2c,
                               .bus_name = config->name,
                               .i2c      = {.port     = port,
                                            .address  = address,
                                            .is_write = true,
                                            .len      = 0}}; /* Len is 0 on error */
    /* --- CORRECTED ACCESS --- */
    config->proto.i2c.callbacks.on_error(&event, ret, config->user_ctx);
  }
  return ret;
}

static esp_err_t priv_pstar_bus_i2c_read(const pstar_bus_config_t* config,
                                         uint8_t*                  data,
                                         size_t                    len,
                                         uint8_t                   reg_addr,
                                         size_t*                   bytes_read)
{
  /* Basic validation (config, data assumed non-NULL, len > 0 by caller) */
  ESP_RETURN_ON_FALSE(config->initialized,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "Bus '%s' is not initialized",
                      config->name);

  /* --- CORRECTED ACCESS --- */
  i2c_port_t       port    = config->proto.i2c.port;
  uint8_t          address = config->proto.i2c.address;
  esp_err_t        ret     = ESP_OK;
  i2c_cmd_handle_t cmd     = NULL;

  cmd = i2c_cmd_link_create();
  ESP_RETURN_ON_FALSE(cmd,
                      ESP_ERR_NO_MEM,
                      TAG,
                      "Failed to create I2C command link for read '%s'",
                      config->name);

  /* --- Write Phase (Device Address + Register Address) --- */
  ret = i2c_master_start(cmd);
  ESP_GOTO_ON_ERROR(ret,
                    read_fail,
                    TAG,
                    "Read '%s': START failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  ret = i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
  ESP_GOTO_ON_ERROR(ret,
                    read_fail,
                    TAG,
                    "Read '%s': Address (W) failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  ret = i2c_master_write_byte(cmd, reg_addr, true);
  ESP_GOTO_ON_ERROR(ret,
                    read_fail,
                    TAG,
                    "Read '%s': Reg Addr failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  /* --- Read Phase (Repeated Start + Device Address + Data) --- */
  ret = i2c_master_start(cmd); /* Repeated start */
  ESP_GOTO_ON_ERROR(ret,
                    read_fail,
                    TAG,
                    "Read '%s': Repeated START failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  ret = i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
  ESP_GOTO_ON_ERROR(ret,
                    read_fail,
                    TAG,
                    "Read '%s': Address (R) failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  if (len > 1) {
    ret = i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    ESP_GOTO_ON_ERROR(ret,
                      read_fail,
                      TAG,
                      "Read '%s': Data read (ACK) failed: %s",
                      config->name,
                      esp_err_to_name(ret));
  }

  /* Read last byte with NACK */
  ret = i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
  ESP_GOTO_ON_ERROR(ret,
                    read_fail,
                    TAG,
                    "Read '%s': Data read (NACK) failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  ret = i2c_master_stop(cmd);
  ESP_GOTO_ON_ERROR(ret,
                    read_fail,
                    TAG,
                    "Read '%s': STOP failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  ESP_GOTO_ON_ERROR(ret,
                    read_fail,
                    TAG,
                    "Read '%s': CMD Begin failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  /* Success path */
  i2c_cmd_link_delete(cmd);
  cmd = NULL; /* Prevent double delete in fail path */
  ESP_LOGD(TAG,
           "Read Success: %zu bytes from '%s' (Addr 0x%02X, Reg 0x%02X)",
           len,
           config->name,
           address,
           reg_addr);
  if (bytes_read) {
    *bytes_read = len;
  }

  /* Call success callback */
  /* --- CORRECTED ACCESS --- */
  if (config->proto.i2c.callbacks.on_transfer_complete) {
    pstar_bus_event_t event = {
      .bus_type = k_pstar_bus_type_i2c,
      .bus_name = config->name,
      .i2c      = {.port = port, .address = address, .is_write = false, .len = len}};
    /* --- CORRECTED ACCESS --- */
    config->proto.i2c.callbacks.on_transfer_complete(&event, config->user_ctx);
  }
  return ESP_OK;

read_fail:
  if (cmd) {
    i2c_cmd_link_delete(cmd);
  }
  ESP_LOGE(TAG, "Read failed for '%s': %s", config->name, esp_err_to_name(ret));
  /* Call error callback */
  /* --- CORRECTED ACCESS --- */
  if (config->proto.i2c.callbacks.on_error) {
    pstar_bus_event_t event = {
      .bus_type = k_pstar_bus_type_i2c,
      .bus_name = config->name,
      .i2c      = {.port = port, .address = address, .is_write = false, .len = 0}};
    /* --- CORRECTED ACCESS --- */
    config->proto.i2c.callbacks.on_error(&event, ret, config->user_ctx);
  }
  return ret;
}

static esp_err_t priv_pstar_bus_i2c_write_command(const pstar_bus_config_t* config, uint8_t command)
{
  /* Basic validation (config assumed non-NULL by caller) */
  ESP_RETURN_ON_FALSE(config->initialized,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "Bus '%s' is not initialized",
                      config->name);

  /* --- CORRECTED ACCESS --- */
  i2c_port_t       port    = config->proto.i2c.port;
  uint8_t          address = config->proto.i2c.address;
  esp_err_t        ret     = ESP_OK;
  i2c_cmd_handle_t cmd     = NULL;

  cmd = i2c_cmd_link_create();
  ESP_RETURN_ON_FALSE(cmd,
                      ESP_ERR_NO_MEM,
                      TAG,
                      "Failed to create I2C command link for write_command '%s'",
                      config->name);

  ret = i2c_master_start(cmd);
  ESP_GOTO_ON_ERROR(ret,
                    write_cmd_fail,
                    TAG,
                    "WriteCmd '%s': START failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  ret = i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
  ESP_GOTO_ON_ERROR(ret,
                    write_cmd_fail,
                    TAG,
                    "WriteCmd '%s': Address (W) failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  /* Write command byte */
  ret = i2c_master_write_byte(cmd, command, true);
  ESP_GOTO_ON_ERROR(ret,
                    write_cmd_fail,
                    TAG,
                    "WriteCmd '%s': Command byte failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  ret = i2c_master_stop(cmd);
  ESP_GOTO_ON_ERROR(ret,
                    write_cmd_fail,
                    TAG,
                    "WriteCmd '%s': STOP failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  ESP_GOTO_ON_ERROR(ret,
                    write_cmd_fail,
                    TAG,
                    "WriteCmd '%s': CMD Begin failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  /* Success path */
  i2c_cmd_link_delete(cmd);
  cmd = NULL; /* Prevent double delete in fail path */
  ESP_LOGD(TAG,
           "Write Command Success: 0x%02X to '%s' (Addr 0x%02X)",
           command,
           config->name,
           address);

  /* Call success callback (treat as write with len=0?) */
  /* --- CORRECTED ACCESS --- */
  if (config->proto.i2c.callbacks.on_transfer_complete) {
    pstar_bus_event_t event = {.bus_type = k_pstar_bus_type_i2c,
                               .bus_name = config->name,
                               .i2c      = {.port     = port,
                                            .address  = address,
                                            .is_write = true,
                                            .len      = 0}}; /* Indicate command write */
    /* --- CORRECTED ACCESS --- */
    config->proto.i2c.callbacks.on_transfer_complete(&event, config->user_ctx);
  }
  return ESP_OK;

write_cmd_fail:
  if (cmd) {
    i2c_cmd_link_delete(cmd);
  }
  ESP_LOGE(TAG, "Write Command failed for '%s': %s", config->name, esp_err_to_name(ret));
  /* Call error callback */
  /* --- CORRECTED ACCESS --- */
  if (config->proto.i2c.callbacks.on_error) {
    pstar_bus_event_t event = {
      .bus_type = k_pstar_bus_type_i2c,
      .bus_name = config->name,
      .i2c      = {.port = port, .address = address, .is_write = true, .len = 0}};
    /* --- CORRECTED ACCESS --- */
    config->proto.i2c.callbacks.on_error(&event, ret, config->user_ctx);
  }
  return ret;
}

static esp_err_t priv_pstar_bus_i2c_read_raw(const pstar_bus_config_t* config,
                                             uint8_t*                  data,
                                             size_t                    len,
                                             size_t*                   bytes_read)
{
  /* Basic validation (config, data assumed non-NULL, len > 0 by caller) */
  ESP_RETURN_ON_FALSE(config->initialized,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "Bus '%s' is not initialized",
                      config->name);

  /* --- CORRECTED ACCESS --- */
  i2c_port_t       port    = config->proto.i2c.port;
  uint8_t          address = config->proto.i2c.address;
  esp_err_t        ret     = ESP_OK;
  i2c_cmd_handle_t cmd     = NULL;

  cmd = i2c_cmd_link_create();
  ESP_RETURN_ON_FALSE(cmd,
                      ESP_ERR_NO_MEM,
                      TAG,
                      "Failed to create I2C command link for read_raw '%s'",
                      config->name);

  /* --- Read Phase (Start + Device Address + Data) --- */
  ret = i2c_master_start(cmd);
  ESP_GOTO_ON_ERROR(ret,
                    read_raw_fail,
                    TAG,
                    "ReadRaw '%s': START failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  ret = i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
  ESP_GOTO_ON_ERROR(ret,
                    read_raw_fail,
                    TAG,
                    "ReadRaw '%s': Address (R) failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  if (len > 1) {
    ret = i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    ESP_GOTO_ON_ERROR(ret,
                      read_raw_fail,
                      TAG,
                      "ReadRaw '%s': Data read (ACK) failed: %s",
                      config->name,
                      esp_err_to_name(ret));
  }

  /* Read last byte with NACK */
  ret = i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
  ESP_GOTO_ON_ERROR(ret,
                    read_raw_fail,
                    TAG,
                    "ReadRaw '%s': Data read (NACK) failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  ret = i2c_master_stop(cmd);
  ESP_GOTO_ON_ERROR(ret,
                    read_raw_fail,
                    TAG,
                    "ReadRaw '%s': STOP failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  ESP_GOTO_ON_ERROR(ret,
                    read_raw_fail,
                    TAG,
                    "ReadRaw '%s': CMD Begin failed: %s",
                    config->name,
                    esp_err_to_name(ret));

  /* Success path */
  i2c_cmd_link_delete(cmd);
  cmd = NULL; /* Prevent double delete in fail path */
  ESP_LOGD(TAG, "Read Raw Success: %zu bytes from '%s' (Addr 0x%02X)", len, config->name, address);
  if (bytes_read) {
    *bytes_read = len;
  }

  /* Call success callback */
  /* --- CORRECTED ACCESS --- */
  if (config->proto.i2c.callbacks.on_transfer_complete) {
    pstar_bus_event_t event = {
      .bus_type = k_pstar_bus_type_i2c,
      .bus_name = config->name,
      .i2c      = {.port = port, .address = address, .is_write = false, .len = len}};
    /* --- CORRECTED ACCESS --- */
    config->proto.i2c.callbacks.on_transfer_complete(&event, config->user_ctx);
  }
  return ESP_OK;

read_raw_fail:
  if (cmd) {
    i2c_cmd_link_delete(cmd);
  }
  ESP_LOGE(TAG, "Read Raw failed for '%s': %s", config->name, esp_err_to_name(ret));
  /* Call error callback */
  /* --- CORRECTED ACCESS --- */
  if (config->proto.i2c.callbacks.on_error) {
    pstar_bus_event_t event = {
      .bus_type = k_pstar_bus_type_i2c,
      .bus_name = config->name,
      .i2c      = {.port = port, .address = address, .is_write = false, .len = 0}};
    /* --- CORRECTED ACCESS --- */
    config->proto.i2c.callbacks.on_error(&event, ret, config->user_ctx);
  }
  return ret;
}