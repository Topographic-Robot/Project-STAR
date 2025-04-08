/* components/pstar_storage_hal/pstar_storage_sdio_hal.c */

#include "pstar_storage_sdio_hal.h"

#include "pstar_log_handler.h"
#include "pstar_storage_common.h"

#include "driver/sdmmc_host.h"

/* Constants ******************************************************************/
static const char* TAG = "Storage SDIO HAL";

/* Public Functions ***********************************************************/

bool storage_sdio_is_supported(void)
{
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
  return true;
#else
  return false;
#endif
}

esp_err_t storage_sdio_setup(sd_card_hal_t* sd_card)
{
  if (sd_card == NULL) {
    log_error(TAG, "SDIO Setup Error", "Invalid SD card HAL pointer");
    return ESP_ERR_INVALID_ARG;
  }

#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED
  log_error(TAG, "SDIO Setup Error", "SDIO Mode is disabled in Kconfig");
  return ESP_ERR_NOT_SUPPORTED;
#endif

  log_info(sd_card->tag,
           "SDIO Setup",
           "Setting up SDIO interface for SD card with %s bus mode",
           storage_bus_width_to_string(sd_card->bus_width));

  /* Check for pin conflicts when using 4-bit mode */
  bool           pin_conflict       = false;
  sd_bus_width_t original_bus_width = sd_card->bus_width;

  if (sd_card->bus_width == k_sd_bus_width_4bit) {
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
    /* Check if card detect pin conflicts with SDIO D3 */
    if (sd_card->pin_config.gpio_det_pin >= 0 &&
        sd_card->pin_config.gpio_det_pin == sd_card->pin_config.sdio_d3_pin) {
      log_warn(sd_card->tag,
               "Pin Conflict",
               "Card detect pin (GPIO %d) conflicts with SDIO D3 for 4-bit mode. "
               "Falling back to 1-bit mode",
               sd_card->pin_config.gpio_det_pin);

      /* Automatically change to 1-bit mode when conflict is detected */
      sd_card->bus_width = k_sd_bus_width_1bit;
      pin_conflict       = true;
    }
#endif

    /* Also check for other potential pin conflicts or unavailable pins */
    if (sd_card->pin_config.sdio_d1_pin < 0 || sd_card->pin_config.sdio_d2_pin < 0 ||
        sd_card->pin_config.sdio_d3_pin < 0) {
      log_warn(sd_card->tag,
               "Pin Configuration",
               "Missing required pins for 4-bit SDIO mode. Falling back to 1-bit mode");

      /* Automatically change to 1-bit mode when pins are unavailable */
      sd_card->bus_width = k_sd_bus_width_1bit;
      pin_conflict       = true;
    }
  }

  /* Create SDIO bus configuration with all required parameters */
  sdmmc_host_t sdmmc_host = SDMMC_HOST_DEFAULT();

  /* Configure the SDMMC host */
  sdmmc_host.max_freq_khz = SDMMC_FREQ_DEFAULT; /* Use default frequency initially */
  sdmmc_host.flags        = 0;                  /* Start with no flags */
  if (sd_card->bus_width == k_sd_bus_width_4bit && !pin_conflict) {
    sdmmc_host.flags |= SDMMC_HOST_FLAG_4BIT;
  } else {
    sdmmc_host.flags |= SDMMC_HOST_FLAG_1BIT;
  }

  /* Set up standard SDMMC configuration for ESP32 */
  sdmmc_host.command_timeout_ms = 5000; /* Increased timeout */
  /*
   * Set the expected I/O voltage for the SD card communication.
   * This value informs the SDMMC driver about the card's operating voltage
   * standard (typically 3.3V) for correct protocol negotiation and timing.
   * It does NOT change the actual hardware voltage output by the ESP32 pins,
   * which is fixed (usually 3.3V). Setting this incorrectly will likely
   * cause card initialization failures.
   */
  sdmmc_host.io_voltage = 3.3f;

  /* Initialize SDMMC host controller */
  esp_err_t err = sdmmc_host_init();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { /* Allow re-init */
    log_error(sd_card->tag,
              "SDIO Setup Error",
              "Failed to initialize SDMMC host: %s",
              esp_err_to_name(err));
    return err;
  }

  /* Configure slot */
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.clk                 = sd_card->pin_config.sdio_clk_pin;
  slot_config.cmd                 = sd_card->pin_config.sdio_cmd_pin;
  slot_config.d0                  = sd_card->pin_config.sdio_d0_pin;
  slot_config.width               = sd_card->bus_width; /* Set width here */

  if (sd_card->bus_width == k_sd_bus_width_4bit && !pin_conflict) {
    slot_config.d1 = sd_card->pin_config.sdio_d1_pin;
    slot_config.d2 = sd_card->pin_config.sdio_d2_pin;
    slot_config.d3 = sd_card->pin_config.sdio_d3_pin;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP; /* Enable pullups for 4-bit */
  } else {
    /* Ensure 1-bit mode if conflict or configured */
    slot_config.width = 1;
    sdmmc_host.flags &= ~SDMMC_HOST_FLAG_4BIT;
    sdmmc_host.flags |= SDMMC_HOST_FLAG_1BIT;
    sd_card->bus_width = k_sd_bus_width_1bit; /* Update HAL state */
  }

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
  slot_config.gpio_cd = sd_card->pin_config.gpio_det_pin;
#else
  slot_config.gpio_cd = SDMMC_SLOT_NO_CD;
#endif
  slot_config.gpio_wp = SDMMC_SLOT_NO_WP; /* Write protect not used */

  /* Initialize SDMMC slot */
  err = sdmmc_host_init_slot(sdmmc_host.slot, &slot_config);
  if (err != ESP_OK) {
    log_error(sd_card->tag,
              "SDIO Setup Error",
              "Failed to initialize SDMMC slot: %s",
              esp_err_to_name(err));
    sdmmc_host_deinit(); /* Clean up host init */
    return err;
  }

  /* Free old card structure if it exists */
  if (sd_card->card != NULL) {
    free(sd_card->card);
    sd_card->card = NULL;
  }

  /* Allocate memory for the card structure */
  sd_card->card = malloc(sizeof(sdmmc_card_t));
  if (sd_card->card == NULL) {
    log_error(sd_card->tag, "SDIO Setup Error", "Failed to allocate memory for SD card");
    sdmmc_host_deinit();
    return ESP_ERR_NO_MEM;
  }

  /* Copy host configuration to our card structure */
  memcpy(&sd_card->card->host, &sdmmc_host, sizeof(sdmmc_host_t));

  /* If we changed bus width due to conflicts, log it */
  if (original_bus_width != sd_card->bus_width) {
    log_info(sd_card->tag,
             "SDIO Bus Width Changed",
             "Changed from %s to %s mode due to pin conflicts or configuration issues",
             storage_bus_width_to_string(original_bus_width),
             storage_bus_width_to_string(sd_card->bus_width));
  }

  log_info(sd_card->tag,
           "SDIO Setup Complete",
           "SDIO interface configured successfully with %s bus mode",
           storage_bus_width_to_string(sd_card->bus_width));
  return ESP_OK;
}
