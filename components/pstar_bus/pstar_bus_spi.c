/* components/pstar_bus/pstar_bus_spi.c */

#include "pstar_bus_spi.h"

#include "pstar_bus_manager.h"

#include "freertos/FreeRTOS.h"

#include <string.h>

#include "esp_check.h"
#include "esp_heap_caps.h" /* For DMA-capable memory */
#include "esp_log.h"

/* --- Constants --- */

static const char* TAG = "BusSPI";
#define SPI_TIMEOUT_MS 1000 /* Default timeout for SPI operations (if polling) */

/* --- Private Function Prototypes (Default Ops) --- */

static esp_err_t priv_pstar_bus_spi_transmit(const pstar_bus_config_t* config,
                                             const void*               tx_buffer,
                                             size_t                    len,
                                             uint32_t                  user_flags);

static esp_err_t priv_pstar_bus_spi_receive(const pstar_bus_config_t* config,
                                            void*                     rx_buffer,
                                            size_t                    len,
                                            uint32_t                  user_flags);

static esp_err_t priv_pstar_bus_spi_transfer(const pstar_bus_config_t* config,
                                             const void*               tx_buffer,
                                             void*                     rx_buffer,
                                             size_t                    len,
                                             uint32_t                  user_flags);

/* --- Public Functions --- */

void pstar_bus_spi_init_default_ops(pstar_spi_ops_t* ops)
{
  if (ops == NULL) {
    ESP_LOGE(TAG, "Cannot initialize NULL ops pointer");
    return;
  }
  ops->transmit = priv_pstar_bus_spi_transmit;
  ops->receive  = priv_pstar_bus_spi_receive;
  ops->transfer = priv_pstar_bus_spi_transfer;
  ESP_LOGD(TAG, "Default SPI operations initialized");
}

esp_err_t pstar_bus_spi_transmit(const pstar_bus_manager_t* manager,
                                 const char*                name,
                                 const void*                tx_buffer,
                                 size_t                     len,
                                 uint32_t                   user_flags)
{
  ESP_RETURN_ON_FALSE(manager && name && tx_buffer,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Manager, name, or tx_buffer is NULL");
  /* Allow len == 0 for command-only transfers? Maybe not useful for SPI. */
  ESP_RETURN_ON_FALSE(len > 0, ESP_ERR_INVALID_ARG, TAG, "Transmit length must be > 0");

  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  ESP_RETURN_ON_FALSE(bus_config, ESP_ERR_NOT_FOUND, TAG, "Bus '%s' not found", name);
  ESP_RETURN_ON_FALSE(bus_config->type == k_pstar_bus_type_spi,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Bus '%s' is not an SPI bus",
                      name);
  ESP_RETURN_ON_FALSE(bus_config->proto.spi.ops.transmit,
                      ESP_ERR_NOT_SUPPORTED,
                      TAG,
                      "No transmit op for '%s'",
                      name);

  return bus_config->proto.spi.ops.transmit(bus_config, tx_buffer, len, user_flags);
}

esp_err_t pstar_bus_spi_receive(const pstar_bus_manager_t* manager,
                                const char*                name,
                                void*                      rx_buffer,
                                size_t                     len,
                                uint32_t                   user_flags)
{
  ESP_RETURN_ON_FALSE(manager && name && rx_buffer,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Manager, name, or rx_buffer is NULL");
  ESP_RETURN_ON_FALSE(len > 0, ESP_ERR_INVALID_ARG, TAG, "Receive length must be > 0");

  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  ESP_RETURN_ON_FALSE(bus_config, ESP_ERR_NOT_FOUND, TAG, "Bus '%s' not found", name);
  ESP_RETURN_ON_FALSE(bus_config->type == k_pstar_bus_type_spi,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Bus '%s' is not an SPI bus",
                      name);
  ESP_RETURN_ON_FALSE(bus_config->proto.spi.ops.receive,
                      ESP_ERR_NOT_SUPPORTED,
                      TAG,
                      "No receive op for '%s'",
                      name);

  return bus_config->proto.spi.ops.receive(bus_config, rx_buffer, len, user_flags);
}

esp_err_t pstar_bus_spi_transfer(const pstar_bus_manager_t* manager,
                                 const char*                name,
                                 const void*                tx_buffer,
                                 void*                      rx_buffer,
                                 size_t                     len,
                                 uint32_t                   user_flags)
{
  ESP_RETURN_ON_FALSE(manager && name && tx_buffer && rx_buffer,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Manager, name, tx_buffer, or rx_buffer is NULL");
  ESP_RETURN_ON_FALSE(len > 0, ESP_ERR_INVALID_ARG, TAG, "Transfer length must be > 0");

  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  ESP_RETURN_ON_FALSE(bus_config, ESP_ERR_NOT_FOUND, TAG, "Bus '%s' not found", name);
  ESP_RETURN_ON_FALSE(bus_config->type == k_pstar_bus_type_spi,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Bus '%s' is not an SPI bus",
                      name);
  ESP_RETURN_ON_FALSE(bus_config->proto.spi.ops.transfer,
                      ESP_ERR_NOT_SUPPORTED,
                      TAG,
                      "No transfer op for '%s'",
                      name);

  return bus_config->proto.spi.ops.transfer(bus_config, tx_buffer, rx_buffer, len, user_flags);
}

/* --- Private Functions (Default Ops Implementations) --- */

/* Default implementation using queued transactions */
static esp_err_t priv_pstar_bus_spi_transmit(const pstar_bus_config_t* config,
                                             const void*               tx_buffer,
                                             size_t                    len,
                                             uint32_t                  user_flags)
{
  ESP_RETURN_ON_FALSE(config->initialized,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "SPI device '%s' is not initialized",
                      config->name);
  ESP_RETURN_ON_FALSE(config->handle,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "SPI device '%s' handle is NULL",
                      config->name);

  spi_transaction_t trans;
  memset(&trans, 0, sizeof(trans));
  trans.length    = len * 8; /* Length is in bits */
  trans.tx_buffer = tx_buffer;
  trans.rx_buffer = NULL;
  trans.user      = (void*)user_flags; /* Pass user flags (e.g., for DC pin) */

  esp_err_t ret = spi_device_polling_transmit((spi_device_handle_t)config->handle, &trans);
  /* Alternative: Queued transmit */
  /* esp_err_t ret = spi_device_queue_trans((spi_device_handle_t)config->handle, &trans, portMAX_DELAY); */
  /* if (ret == ESP_OK) { */
  /*     spi_transaction_t *ret_trans; */
  /*     ret = spi_device_get_trans_result((spi_device_handle_t)config->handle, &ret_trans, portMAX_DELAY); */
  /* } */

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI transmit failed for '%s': %s", config->name, esp_err_to_name(ret));
    /* Call error callback */
    if (config->proto.spi.callbacks.on_error) {
      pstar_bus_event_t event = {.bus_type = k_pstar_bus_type_spi,
                                 .bus_name = config->name,
                                 .spi      = {.host     = config->proto.spi.host,
                                              .cs_pin   = config->proto.spi.dev_cfg.spics_io_num,
                                              .is_write = true,
                                              .tx_len   = 0, /* Indicate failure */
                                              .rx_len   = 0}};
      config->proto.spi.callbacks.on_error(&event, ret, config->user_ctx);
    }
  } else {
    ESP_LOGD(TAG, "SPI transmit success: %zu bytes from '%s'", len, config->name);
    /* Call success callback */
    if (config->proto.spi.callbacks.on_transfer_complete) {
      pstar_bus_event_t event = {.bus_type = k_pstar_bus_type_spi,
                                 .bus_name = config->name,
                                 .spi      = {.host     = config->proto.spi.host,
                                              .cs_pin   = config->proto.spi.dev_cfg.spics_io_num,
                                              .is_write = true,
                                              .tx_len   = len,
                                              .rx_len   = 0}};
      config->proto.spi.callbacks.on_transfer_complete(&event, config->user_ctx);
    }
  }
  return ret;
}

static esp_err_t priv_pstar_bus_spi_receive(const pstar_bus_config_t* config,
                                            void*                     rx_buffer,
                                            size_t                    len,
                                            uint32_t                  user_flags)
{
  ESP_RETURN_ON_FALSE(config->initialized,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "SPI device '%s' is not initialized",
                      config->name);
  ESP_RETURN_ON_FALSE(config->handle,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "SPI device '%s' handle is NULL",
                      config->name);

  spi_transaction_t trans;
  memset(&trans, 0, sizeof(trans));
  trans.length    = len * 8; /* Length is in bits */
  trans.tx_buffer = NULL;
  trans.rxlength  = len * 8; /* Specify receive length */
  trans.rx_buffer = rx_buffer;
  trans.user      = (void*)user_flags;

  /* Using polling for simplicity, consider queueing for complex apps */
  esp_err_t ret = spi_device_polling_transmit((spi_device_handle_t)config->handle, &trans);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI receive failed for '%s': %s", config->name, esp_err_to_name(ret));
    /* Call error callback */
    if (config->proto.spi.callbacks.on_error) {
      pstar_bus_event_t event = {.bus_type = k_pstar_bus_type_spi,
                                 .bus_name = config->name,
                                 .spi      = {.host     = config->proto.spi.host,
                                              .cs_pin   = config->proto.spi.dev_cfg.spics_io_num,
                                              .is_write = false,
                                              .tx_len   = 0,
                                              .rx_len   = 0}}; /* Indicate failure */
      config->proto.spi.callbacks.on_error(&event, ret, config->user_ctx);
    }
  } else {
    ESP_LOGD(TAG, "SPI receive success: %zu bytes to '%s'", len, config->name);
    /* Call success callback */
    if (config->proto.spi.callbacks.on_transfer_complete) {
      pstar_bus_event_t event = {.bus_type = k_pstar_bus_type_spi,
                                 .bus_name = config->name,
                                 .spi      = {.host     = config->proto.spi.host,
                                              .cs_pin   = config->proto.spi.dev_cfg.spics_io_num,
                                              .is_write = false,
                                              .tx_len   = 0,
                                              .rx_len   = len}};
      config->proto.spi.callbacks.on_transfer_complete(&event, config->user_ctx);
    }
  }
  return ret;
}

static esp_err_t priv_pstar_bus_spi_transfer(const pstar_bus_config_t* config,
                                             const void*               tx_buffer,
                                             void*                     rx_buffer,
                                             size_t                    len,
                                             uint32_t                  user_flags)
{
  ESP_RETURN_ON_FALSE(config->initialized,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "SPI device '%s' is not initialized",
                      config->name);
  ESP_RETURN_ON_FALSE(config->handle,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "SPI device '%s' handle is NULL",
                      config->name);

  spi_transaction_t trans;
  memset(&trans, 0, sizeof(trans));
  trans.length    = len * 8; /* Length is in bits */
  trans.tx_buffer = tx_buffer;
  trans.rxlength  = len * 8; /* Specify receive length */
  trans.rx_buffer = rx_buffer;
  trans.user      = (void*)user_flags;

  /* Using polling for simplicity */
  esp_err_t ret = spi_device_polling_transmit((spi_device_handle_t)config->handle, &trans);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI transfer failed for '%s': %s", config->name, esp_err_to_name(ret));
    /* Call error callback */
    if (config->proto.spi.callbacks.on_error) {
      pstar_bus_event_t event = {.bus_type = k_pstar_bus_type_spi,
                                 .bus_name = config->name,
                                 .spi      = {.host     = config->proto.spi.host,
                                              .cs_pin   = config->proto.spi.dev_cfg.spics_io_num,
                                              .is_write = true, /* Treat transfer as primarily write */
                                              .tx_len   = 0,
                                              .rx_len   = 0}}; /* Indicate failure */
      config->proto.spi.callbacks.on_error(&event, ret, config->user_ctx);
    }
  } else {
    ESP_LOGD(TAG, "SPI transfer success: %zu bytes for '%s'", len, config->name);
    /* Call success callback */
    if (config->proto.spi.callbacks.on_transfer_complete) {
      pstar_bus_event_t event = {.bus_type = k_pstar_bus_type_spi,
                                 .bus_name = config->name,
                                 .spi      = {.host     = config->proto.spi.host,
                                              .cs_pin   = config->proto.spi.dev_cfg.spics_io_num,
                                              .is_write = true, /* Treat transfer as primarily write */
                                              .tx_len   = len,
                                              .rx_len   = len}};
      config->proto.spi.callbacks.on_transfer_complete(&event, config->user_ctx);
    }
  }
  return ret;
}