/* components/pstar_bus/bus_sdio.c */

#include "bus_sdio.h"
#include "bus_manager.h"
#include "log_handler.h"
#include "bus_types.h"
#include "bus_event.h"
#include <string.h>
#include "sd_card_hal.h"

/* Constants ******************************************************************/

static const char* const bus_sdio_tag = "Bus SDIO";

/* Private Function Prototypes ************************************************/

static esp_err_t priv_pstar_bus_sdio_write(const pstar_bus_manager_t* manager,
                                           const char*                name,
                                           const uint8_t*             data, 
                                           size_t                     len,
                                           size_t                     offset,
                                           size_t*                    bytes_written);

static esp_err_t priv_pstar_bus_sdio_read(const pstar_bus_manager_t* manager,
                                          const char*                name,
                                          uint8_t*                   data, 
                                          size_t                     len,
                                          size_t                     offset,
                                          size_t*                    bytes_read);

static esp_err_t priv_pstar_bus_sdio_ioctl(const pstar_bus_manager_t* manager,
                                           const char*                name,
                                           int                        cmd,
                                           void*                      arg);

/* Public Functions ***********************************************************/

void pstar_bus_sdio_init_default_ops(pstar_sdio_ops_t* ops)
{
  if (ops == NULL) {
    log_error(bus_sdio_tag, 
              "Init Error", 
              "SDIO operations pointer is NULL");
    return;
  }
  
  ops->write = priv_pstar_bus_sdio_write;
  ops->read  = priv_pstar_bus_sdio_read;
  ops->ioctl = priv_pstar_bus_sdio_ioctl;
  
  log_info(bus_sdio_tag, 
           "Default Ops", 
           "Initialized default SDIO operations");
}

esp_err_t pstar_bus_sdio_write(const pstar_bus_manager_t* manager,
                               const char*                name,
                               const uint8_t*             data, 
                               size_t                     len,
                               size_t                     offset,
                               size_t*                    bytes_written)
{
  /* Validate input */
  if (!manager || !name) {
    log_error(bus_sdio_tag, 
              "Write Error", 
              "Invalid parameters: manager or name is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(bus_sdio_tag, 
              "Write Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is an SDIO bus */
  if (bus_config->type != k_pstar_bus_type_sdio) {
    log_error(bus_sdio_tag, 
              "Write Error", 
              "Bus '%s' is not an SDIO bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Call the write operation function */
  if (bus_config->config.sdio.ops.write) {
    return bus_config->config.sdio.ops.write(manager, name, data, len, offset, bytes_written);
  } else {
    log_error(bus_sdio_tag, 
              "Write Error", 
              "No write operation defined for SDIO bus '%s'", 
              name);
    return ESP_ERR_NOT_SUPPORTED;
  }
}

esp_err_t pstar_bus_sdio_read(const pstar_bus_manager_t* manager,
                              const char*                name,
                              uint8_t*                   data, 
                              size_t                     len,
                              size_t                     offset,
                              size_t*                    bytes_read)
{
  /* Validate input */
  if (!manager || !name) {
    log_error(bus_sdio_tag, 
              "Read Error", 
              "Invalid parameters: manager or name is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(bus_sdio_tag, 
              "Read Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is an SDIO bus */
  if (bus_config->type != k_pstar_bus_type_sdio) {
    log_error(bus_sdio_tag, 
              "Read Error", 
              "Bus '%s' is not an SDIO bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Call the read operation function */
  if (bus_config->config.sdio.ops.read) {
    return bus_config->config.sdio.ops.read(manager, name, data, len, offset, bytes_read);
  } else {
    log_error(bus_sdio_tag, 
              "Read Error", 
              "No read operation defined for SDIO bus '%s'", 
              name);
    return ESP_ERR_NOT_SUPPORTED;
  }
}

esp_err_t pstar_bus_sdio_ioctl(const pstar_bus_manager_t* manager,
                               const char*                name,
                               int                        cmd,
                               void*                      arg)
{
  /* Validate input */
  if (!manager || !name) {
    log_error(bus_sdio_tag, 
              "IOCTL Error", 
              "Invalid parameters: manager or name is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(bus_sdio_tag, 
              "IOCTL Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is an SDIO bus */
  if (bus_config->type != k_pstar_bus_type_sdio) {
    log_error(bus_sdio_tag, 
              "IOCTL Error", 
              "Bus '%s' is not an SDIO bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Call the ioctl operation function */
  if (bus_config->config.sdio.ops.ioctl) {
    return bus_config->config.sdio.ops.ioctl(manager, name, cmd, arg);
  } else {
    log_error(bus_sdio_tag, 
              "IOCTL Error", 
              "No ioctl operation defined for SDIO bus '%s'", 
              name);
    return ESP_ERR_NOT_SUPPORTED;
  }
}

/* Private Functions **********************************************************/

/**
 * @brief Write data to an SDIO device using the default implementation.
 * 
 * @param[in]  manager       Pointer to the bus manager.
 * @param[in]  name          Name of the SDIO bus.
 * @param[in]  data          Data to write.
 * @param[in]  len           Length of data to write.
 * @param[in]  offset        Offset to write at.
 * @param[out] bytes_written Pointer to store the number of bytes written (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
static esp_err_t priv_pstar_bus_sdio_write(const pstar_bus_manager_t* manager,
                                           const char*                name,
                                           const uint8_t*             data, 
                                           size_t                     len,
                                           size_t                     offset,
                                           size_t*                    bytes_written)
{
  esp_err_t result = ESP_OK;
  
  /* Validate input */
  if (!manager || !name || !data || len == 0) {
    log_error(bus_sdio_tag, 
              "Write Error", 
              "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(bus_sdio_tag, 
              "Write Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is an SDIO bus */
  if (bus_config->type != k_pstar_bus_type_sdio) {
    log_error(bus_sdio_tag, 
              "Write Error", 
              "Bus '%s' is not an SDIO bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(bus_sdio_tag, 
              "Write Error", 
              "Bus '%s' is not initialized", 
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Get SDIO card handle */
  sdmmc_card_t* card = bus_config->config.sdio.card;
  if (!card) {
    log_error(bus_sdio_tag, 
              "Write Error", 
              "No SDIO card found for bus '%s'", 
              name);
    return ESP_ERR_INVALID_STATE;
  }

  /* Validate sector size */
  if (card->csd.sector_size == 0) {
    log_error(bus_sdio_tag,
              "Write Error",
              "Invalid sector size (0) for SDIO card on bus '%s'",
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Check if length is aligned to sector size */
  if (len % card->csd.sector_size != 0) {
    log_error(bus_sdio_tag,
              "Write Error",
              "Data length (%zu) is not a multiple of sector size (%d) on bus '%s'",
              len,
              card->csd.sector_size,
              name);
    return ESP_ERR_INVALID_SIZE;
  }
  
  /* Calculate number of sectors to write */
  size_t sectors = len / card->csd.sector_size;
  if (sectors == 0) {
    log_error(bus_sdio_tag,
              "Write Error",
              "Data length too small to write a complete sector on bus '%s'",
              name);
    return ESP_ERR_INVALID_SIZE;
  }
  
  /* Write data to card */
  result = sdmmc_write_sectors(card, data, offset, sectors);
  if (result != ESP_OK) {
    log_error(bus_sdio_tag, 
              "Write Error", 
              "Failed to write to SDIO card on bus '%s': %s", 
              name, 
              esp_err_to_name(result));
    return result;
  }
  
  log_debug(bus_sdio_tag, 
            "Write Success", 
            "Successfully wrote %zu bytes (%zu sectors) to SDIO bus '%s' at offset %zu",
            len, 
            sectors,
            name, 
            offset);
  
  /* Set bytes written if provided */
  if (bytes_written) {
    *bytes_written = len;
  }
  
  /* Call callback if available */
  if (bus_config->config.sdio.callbacks.on_transfer_complete) {
    /* Create event structure */
    pstar_bus_event_t event = {
      .bus_type  = k_pstar_bus_type_sdio,
      .bus_name  = name,
      .data.sdio = {
        .is_write     = true,
        .len          = len,
        .offset       = offset,
        .card_present = (card != NULL)
      }
    };
    
    /* Call the callback */
    bus_config->config.sdio.callbacks.on_transfer_complete(&event, bus_config->user_ctx);
  }
  
  return ESP_OK;
}

/**
 * @brief Read data from an SDIO device using the default implementation.
 * 
 * @param[in]  manager    Pointer to the bus manager.
 * @param[in]  name       Name of the SDIO bus.
 * @param[out] data       Buffer to read data into.
 * @param[in]  len        Length of data to read.
 * @param[in]  offset     Offset to read from.
 * @param[out] bytes_read Pointer to store the number of bytes read (can be NULL).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
static esp_err_t priv_pstar_bus_sdio_read(const pstar_bus_manager_t* manager,
                                          const char*                name,
                                          uint8_t*                   data, 
                                          size_t                     len,
                                          size_t                     offset,
                                          size_t*                    bytes_read)
{
  esp_err_t result = ESP_OK;
  
  /* Validate input */
  if (!manager || !name || !data || len == 0) {
    log_error(bus_sdio_tag, 
              "Read Error", 
              "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(bus_sdio_tag, 
              "Read Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is an SDIO bus */
  if (bus_config->type != k_pstar_bus_type_sdio) {
    log_error(bus_sdio_tag, 
              "Read Error", 
              "Bus '%s' is not an SDIO bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(bus_sdio_tag, 
              "Read Error", 
              "Bus '%s' is not initialized", 
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Get SDIO card handle */
  sdmmc_card_t* card = bus_config->config.sdio.card;
  if (!card) {
    log_error(bus_sdio_tag, 
              "Read Error", 
              "No SDIO card found for bus '%s'", 
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Validate sector size */
  if (card->csd.sector_size == 0) {
    log_error(bus_sdio_tag,
              "Read Error",
              "Invalid sector size (0) for SDIO card on bus '%s'",
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Check if length is aligned to sector size */
  if (len % card->csd.sector_size != 0) {
    log_error(bus_sdio_tag,
              "Read Error",
              "Data length (%zu) is not a multiple of sector size (%d) on bus '%s'",
              len,
              card->csd.sector_size,
              name);
    return ESP_ERR_INVALID_SIZE;
  }
  
  /* Calculate number of sectors to read */
  size_t sectors = len / card->csd.sector_size;
  if (sectors == 0) {
    log_error(bus_sdio_tag,
              "Read Error",
              "Data length too small to read a complete sector on bus '%s'",
              name);
    return ESP_ERR_INVALID_SIZE;
  }
  
  /* Read data from card */
  result = sdmmc_read_sectors(card, data, offset, sectors);
  if (result != ESP_OK) {
    log_error(bus_sdio_tag, 
              "Read Error", 
              "Failed to read from SDIO card on bus '%s': %s", 
              name, 
              esp_err_to_name(result));
    return result;
  }
  
  log_debug(bus_sdio_tag, 
            "Read Success", 
            "Successfully read %zu bytes (%zu sectors) from SDIO bus '%s' at offset %zu",
            len, 
            sectors,
            name, 
            offset);
  
  /* Set bytes read if provided */
  if (bytes_read) {
    *bytes_read = len;
  }
  
  /* Call callback if available */
  if (bus_config->config.sdio.callbacks.on_transfer_complete) {
    /* Create event structure */
    pstar_bus_event_t event = {
      .bus_type  = k_pstar_bus_type_sdio,
      .bus_name  = name,
      .data.sdio = {
        .is_write     = false,
        .len          = len,
        .offset       = offset,
        .card_present = (card != NULL)
      }
    };
    
    /* Call the callback */
    bus_config->config.sdio.callbacks.on_transfer_complete(&event, 
                                                           bus_config->user_ctx);
  }
  
  return ESP_OK;
}

/**
 * @brief Perform an IOCTL operation on an SDIO device using the default implementation.
 * 
 * @param[in] manager Pointer to the bus manager.
 * @param[in] name    Name of the SDIO bus.
 * @param[in] cmd     IOCTL command.
 * @param[in] arg     Argument for the command.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
static esp_err_t priv_pstar_bus_sdio_ioctl(const pstar_bus_manager_t* manager,
                                           const char*                name,
                                           int                        cmd,
                                           void*                      arg)
{
  /* Validate input */
  if (!manager || !name) {
    log_error(bus_sdio_tag, 
              "IOCTL Error", 
              "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(bus_sdio_tag, 
              "IOCTL Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is an SDIO bus */
  if (bus_config->type != k_pstar_bus_type_sdio) {
    log_error(bus_sdio_tag, 
              "IOCTL Error", 
              "Bus '%s' is not an SDIO bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(bus_sdio_tag, 
              "IOCTL Error", 
              "Bus '%s' is not initialized", 
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Get SDIO card handle */
  sdmmc_card_t* card = bus_config->config.sdio.card;
  
  /* Check if card is NULL for commands that require card access */
  if (card == NULL && 
      (cmd == k_pstar_sdio_ioctl_get_csd || 
       cmd == k_pstar_sdio_ioctl_get_cid || 
       cmd == k_pstar_sdio_ioctl_get_scr || 
       cmd == k_pstar_sdio_ioctl_get_ocr || 
       cmd == k_pstar_sdio_ioctl_get_rca || 
       cmd == k_pstar_sdio_ioctl_get_bus_width || 
       cmd == k_pstar_sdio_ioctl_get_bus_freq)) {
    log_error(bus_sdio_tag, 
              "IOCTL Error", 
              "Card not available for requested operation");
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Handle IOCTL command */
  switch (cmd) {
    case k_pstar_sdio_ioctl_get_card_info:
      if (!arg) {
        log_error(bus_sdio_tag, 
                  "IOCTL Error", 
                  "Argument is NULL for get_card_info");
        return ESP_ERR_INVALID_ARG;
      }
      *(sdmmc_card_t**)arg = card;
      break;
      
    case k_pstar_sdio_ioctl_get_status:
      /* Get card status */
      if (!arg) {
        log_error(bus_sdio_tag, 
                  "IOCTL Error", 
                  "Argument is NULL for get_status");
        return ESP_ERR_INVALID_ARG;
      }
      /* Just check if card pointer is not NULL */
      *(uint8_t*)arg = (card != NULL) ? 1 : 0;
      break;
      
    case k_pstar_sdio_ioctl_get_csd:
      /* Get CSD register */
      if (!arg) {
        log_error(bus_sdio_tag, 
                  "IOCTL Error", 
                  "Argument is NULL for get_csd");
        return ESP_ERR_INVALID_ARG;
      }
      /* Card pointer is already validated above */
      memcpy(arg, &card->csd, sizeof(sdmmc_csd_t));
      break;
      
    case k_pstar_sdio_ioctl_get_cid:
      /* Get CID register */
      if (!arg) {
        log_error(bus_sdio_tag, 
                  "IOCTL Error", 
                  "Argument is NULL for get_cid");
        return ESP_ERR_INVALID_ARG;
      }
      /* Card pointer is already validated above */
      memcpy(arg, &card->cid, sizeof(sdmmc_cid_t));
      break;
      
    case k_pstar_sdio_ioctl_get_scr:
      /* Get SCR register */
      if (!arg) {
        log_error(bus_sdio_tag, 
                  "IOCTL Error", 
                  "Argument is NULL for get_scr");
        return ESP_ERR_INVALID_ARG;
      }
      /* Card pointer is already validated above */
      if (card->scr.sd_spec == 0) {
        log_error(bus_sdio_tag, 
                  "IOCTL Error", 
                  "SCR register not available");
        return ESP_ERR_NOT_SUPPORTED;
      }
      memcpy(arg, &card->scr, sizeof(sdmmc_scr_t));
      break;
      
    case k_pstar_sdio_ioctl_get_ocr:
      /* Get OCR register */
      if (!arg) {
        log_error(bus_sdio_tag, 
                  "IOCTL Error", 
                  "Argument is NULL for get_ocr");
        return ESP_ERR_INVALID_ARG;
      }
      /* Card pointer is already validated above */
      *(uint32_t*)arg = card->ocr;
      break;
      
    case k_pstar_sdio_ioctl_get_rca:
      /* Get RCA register */
      if (!arg) {
        log_error(bus_sdio_tag, 
                  "IOCTL Error", 
                  "Argument is NULL for get_rca");
        return ESP_ERR_INVALID_ARG;
      }
      /* Card pointer is already validated above */
      *(uint16_t*)arg = card->rca;
      break;
      
    case k_pstar_sdio_ioctl_get_bus_width:
      /* Get bus width */
      if (!arg) {
        log_error(bus_sdio_tag, 
                  "IOCTL Error", 
                  "Argument is NULL for get_bus_width");
        return ESP_ERR_INVALID_ARG;
      }
      /* Card pointer is already validated above */
      /* Fixed bug: Return actual bus width instead of frequency */
      if (card->host.flags & SDMMC_HOST_FLAG_4BIT) {
        *(uint8_t*)arg = 4; /* 4-bit bus width */
      } else {
        *(uint8_t*)arg = 1; /* 1-bit bus width */
      }
      break;
      
    case k_pstar_sdio_ioctl_set_bus_width:
      /* Set bus width - not supported */
      log_error(bus_sdio_tag, 
                "IOCTL Error", 
                "Setting bus width not supported");
      return ESP_ERR_NOT_SUPPORTED;
      
    case k_pstar_sdio_ioctl_get_bus_freq:
      /* Get bus frequency */
      if (!arg) {
        log_error(bus_sdio_tag, 
                  "IOCTL Error", 
                  "Argument is NULL for get_bus_freq");
        return ESP_ERR_INVALID_ARG;
      }
      /* Card pointer is already validated above */
      *(uint32_t*)arg = card->max_freq_khz;
      break;
      
    case k_pstar_sdio_ioctl_set_bus_freq:
      /* Set bus frequency - not supported */
      log_error(bus_sdio_tag, 
                "IOCTL Error", 
                "Setting bus frequency not supported");
      return ESP_ERR_NOT_SUPPORTED;
      
    case k_pstar_sdio_ioctl_enable_card_detect:
      /* Enable/disable card detect is not supported in the current implementation */
      log_error(bus_sdio_tag, 
                "IOCTL Error", 
                "Card detection control not supported");
      return ESP_ERR_NOT_SUPPORTED;
      
    case k_pstar_sdio_ioctl_custom:
      /* Custom IOCTL - not supported */
      log_error(bus_sdio_tag, 
                "IOCTL Error", 
                "Custom IOCTL not supported");
      return ESP_ERR_NOT_SUPPORTED;
      
    default:
      log_error(bus_sdio_tag, 
                "IOCTL Error", 
                "Unknown IOCTL command: %d", 
                cmd);
      return ESP_ERR_INVALID_ARG;
  }
  
  log_debug(bus_sdio_tag, 
            "IOCTL Success", 
            "Successfully executed IOCTL command %d on SDIO bus '%s'",
            cmd, 
            name);
  
  return ESP_OK;
}
