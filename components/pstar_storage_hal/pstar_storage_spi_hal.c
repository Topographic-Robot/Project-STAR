/* components/pstar_storage_hal/pstar_storage_spi_hal.c */

#include "pstar_storage_spi_hal.h"

#include "pstar_bus_config.h"
#include "pstar_bus_manager.h"
#include "pstar_bus_spi.h"
#include "pstar_log_handler.h"
#include "pstar_storage_common.h"

#include "driver/sdspi_host.h"
#include "driver/spi_common.h"

#include <string.h>

/* Constants ******************************************************************/
static const char* TAG = "Storage SPI HAL";

/* Public Functions ***********************************************************/

bool storage_spi_is_supported(void)
{
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  return true;
#else
  return false;
#endif
}

esp_err_t storage_spi_setup(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    log_error(TAG, "SPI Setup Error", "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
  log_error(TAG, "SPI Setup Error", "SPI Mode is disabled in Kconfig");
  return ESP_ERR_NOT_SUPPORTED;
#endif

  log_info(sd_card->tag, "SPI Setup", "Setting up SPI interface for SD card");

  /* Verify SPI pins are valid */
  if (sd_card->pin_config.spi_do_pin < 0 || sd_card->pin_config.spi_di_pin < 0 ||
      sd_card->pin_config.spi_sclk_pin < 0 || sd_card->pin_config.spi_cs_pin < 0) {
    log_error(sd_card->tag, "SPI Setup Error", "Invalid SPI pin configuration");
    return ESP_ERR_INVALID_ARG;
  }

  /* Create SPI bus configuration */
  pstar_bus_config_t* spi_config = NULL;
  esp_err_t           err        = ESP_OK;

  /* Check if the bus already exists */
  pstar_bus_config_t* existing_spi =
    pstar_bus_manager_find_bus(&sd_card->bus_manager, CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_BUS_NAME);

  if (existing_spi != NULL) {
    log_info(sd_card->tag, "SPI Info", "Re-using existing SPI bus configuration.");
    if (existing_spi->handle != NULL) {
      log_debug(sd_card->tag, "SPI Info", "Clearing old device handle from bus config.");
      existing_spi->handle = NULL; /* Clear the handle */
    }
    spi_config = existing_spi; /* Use existing config */
  } else {
    /* Create new SPI bus configuration */
    spi_config = pstar_bus_config_create_spi(CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_BUS_NAME,
                                             SPI2_HOST, // Assuming SPI2_HOST for Card 1
                                             k_pstar_mode_polling);
    if (spi_config == NULL) {
      log_error(sd_card->tag, "SPI Setup Error", "Failed to create SPI bus configuration");
      return ESP_ERR_NO_MEM;
    }

    /* Configure SPI pins from the pin configuration */
    spi_config->config.spi.bus_config.miso_io_num = sd_card->pin_config.spi_do_pin; /* MISO (DO) */
    spi_config->config.spi.bus_config.mosi_io_num = sd_card->pin_config.spi_di_pin; /* MOSI (DI) */
    spi_config->config.spi.bus_config.sclk_io_num = sd_card->pin_config.spi_sclk_pin; /* CLK */
    spi_config->config.spi.bus_config.quadwp_io_num   = -1;                           /* Not used */
    spi_config->config.spi.bus_config.quadhd_io_num   = -1;                           /* Not used */
    spi_config->config.spi.bus_config.max_transfer_sz = 4092; /* Max SPI transfer size */

    /* Configure SPI device - Start with lower speed for initialization */
    spi_config->config.spi.dev_config.clock_speed_hz = 400 * 1000; /* 400 kHz initial speed */
    spi_config->config.spi.dev_config.mode           = 0;          /* SPI mode 0 */
    spi_config->config.spi.dev_config.spics_io_num =
      sd_card->pin_config.spi_cs_pin;                 /* Chip Select */
    spi_config->config.spi.dev_config.queue_size = 7; /* Queue size for transactions */
    spi_config->config.spi.dev_config.flags      = 0; /* No special flags */

    /* Add SPI bus to the manager */
    err = pstar_bus_manager_add_bus(&sd_card->bus_manager, spi_config);
    if (err != ESP_OK) {
      log_error(sd_card->tag,
                "SPI Setup Error",
                "Failed to add SPI bus to manager: %s",
                esp_err_to_name(err));
      pstar_bus_config_destroy(spi_config);
      return err;
    }

    /* Initialize the SPI bus */
    err = pstar_bus_config_init(spi_config);
    if (err != ESP_OK) {
      log_error(sd_card->tag,
                "SPI Setup Error",
                "Failed to initialize SPI bus: %s",
                esp_err_to_name(err));
      pstar_bus_manager_remove_bus(&sd_card->bus_manager,
                                   CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_BUS_NAME);
      return err;
    }

    /* Initialize default SPI operations */
    pstar_bus_spi_init_default_ops(&spi_config->config.spi.ops);
  }

  /* Initialize SD SPI driver (only if not already done) */
  err = sdspi_host_init(); /* Store result of init call */
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    log_error(sd_card->tag,
              "SPI Setup Error",
              "Failed to initialize SDSPI host: %s",
              esp_err_to_name(err));
    if (existing_spi == NULL) { /* Only cleanup if we created it */
      pstar_bus_manager_remove_bus(&sd_card->bus_manager,
                                   CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_BUS_NAME);
    }
    return err;
  }

  // --- REMOVED sdspi_host_init_device CALL ---
  // Rationale: Let esp_vfs_fat_sdspi_mount handle device initialization implicitly.

  log_info(sd_card->tag,
           "SPI Setup Complete",
           "SPI interface configured successfully on pins MISO(DO):%d, MOSI(DI):%d, CLK:%d, CS:%d",
           sd_card->pin_config.spi_do_pin,   // MISO is DO
           sd_card->pin_config.spi_di_pin,   // MOSI is DI
           sd_card->pin_config.spi_sclk_pin, // CLK
           sd_card->pin_config.spi_cs_pin    // CS
  );
  return ESP_OK;
}
