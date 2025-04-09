/* components/pstar_bus/pstar_bus_config.c */

#include "pstar_bus_config.h"

#include "pstar_log_handler.h"
#include "pstar_pin_validator.h"

#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"

/* Constants ******************************************************************/

static const char* TAG = "Bus Config";

/* Private Function Prototypes ************************************************/

static void priv_pstar_bus_config_cleanup(pstar_bus_config_t* config);

static pstar_bus_config_t* priv_pstar_bus_config_create_common(const char*         name,
                                                               pstar_bus_type_t    type,
                                                               pstar_common_mode_t mode);

/* Public Functions ***********************************************************/

pstar_bus_config_t* pstar_bus_config_create_i2c(const char*         name,
                                                i2c_port_t          port,
                                                uint8_t             address,
                                                pstar_common_mode_t mode)
{
  pstar_bus_config_t* config =
    priv_pstar_bus_config_create_common(name, k_pstar_bus_type_i2c, mode);
  if (config == NULL) {
    return NULL;
  }

  /* Configure I2C-specific parameters */
  config->config.i2c.port    = port;
  config->config.i2c.address = address;

  /* Initialize with default I2C configuration */
  memset(&config->config.i2c.config, 0, sizeof(i2c_config_t));
  config->config.i2c.config.mode             = I2C_MODE_MASTER;
  config->config.i2c.config.sda_io_num       = -1;     /* Set to -1, to be configured by user */
  config->config.i2c.config.scl_io_num       = -1;     /* Set to -1, to be configured by user */
  config->config.i2c.config.master.clk_speed = 100000; /* Default to 100kHz */

  log_info(TAG,
           "I2C Created",
           "Created I2C bus configuration for '%s', port %d, address 0x%02X",
           name,
           port,
           address);
  return config;
}

pstar_bus_config_t*
pstar_bus_config_create_spi(const char* name, spi_host_device_t host, pstar_common_mode_t mode)
{
  pstar_bus_config_t* config =
    priv_pstar_bus_config_create_common(name, k_pstar_bus_type_spi, mode);
  if (config == NULL) {
    return NULL;
  }

  /* Configure SPI-specific parameters */
  config->config.spi.host = host;

  /* Initialize with default SPI bus configuration */
  memset(&config->config.spi.bus_config, 0, sizeof(spi_bus_config_t));
  config->config.spi.bus_config.mosi_io_num     = -1; /* Set to -1, to be configured by user */
  config->config.spi.bus_config.miso_io_num     = -1; /* Set to -1, to be configured by user */
  config->config.spi.bus_config.sclk_io_num     = -1; /* Set to -1, to be configured by user */
  config->config.spi.bus_config.quadwp_io_num   = -1; /* Not used */
  config->config.spi.bus_config.quadhd_io_num   = -1; /* Not used */
  config->config.spi.bus_config.max_transfer_sz = 0;  /* Use default value */

  /* Initialize with default SPI device configuration */
  memset(&config->config.spi.dev_config, 0, sizeof(spi_device_interface_config_t));
  config->config.spi.dev_config.clock_speed_hz = 1000000; /* Default to 1MHz */
  config->config.spi.dev_config.mode           = 0;       /* SPI mode 0 */
  config->config.spi.dev_config.spics_io_num   = -1;      /* Set to -1, to be configured by user */
  config->config.spi.dev_config.queue_size     = 1;       /* Default queue size */

  log_info(TAG, "SPI Created", "Created SPI bus configuration for '%s', host %d", name, host);
  return config;
}

pstar_bus_config_t* pstar_bus_config_create_uart(const char*         name,
                                                 uart_port_t         port,
                                                 size_t              rx_buffer_size,
                                                 size_t              tx_buffer_size,
                                                 pstar_common_mode_t mode)
{
  pstar_bus_config_t* config =
    priv_pstar_bus_config_create_common(name, k_pstar_bus_type_uart, mode);
  if (config == NULL) {
    return NULL;
  }

  /* Configure UART-specific parameters */
  config->config.uart.port           = port;
  config->config.uart.rx_buffer_size = rx_buffer_size;
  config->config.uart.tx_buffer_size = tx_buffer_size;

  /* Initialize with default UART configuration */
  memset(&config->config.uart.config, 0, sizeof(uart_config_t));
  config->config.uart.config.baud_rate           = 115200; /* Default baud rate */
  config->config.uart.config.data_bits           = UART_DATA_8_BITS;
  config->config.uart.config.parity              = UART_PARITY_DISABLE;
  config->config.uart.config.stop_bits           = UART_STOP_BITS_1;
  config->config.uart.config.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
  config->config.uart.config.rx_flow_ctrl_thresh = 0; /* Not used without flow control */
  config->config.uart.config.source_clk          = UART_SCLK_APB;

  log_info(TAG,
           "UART Created",
           "Created UART bus configuration for '%s', port %d, RX buffer %zu, TX buffer %zu",
           name,
           port,
           rx_buffer_size,
           tx_buffer_size);
  return config;
}

pstar_bus_config_t* pstar_bus_config_create_gpio(const char* name, pstar_common_mode_t mode)
{
  pstar_bus_config_t* config =
    priv_pstar_bus_config_create_common(name, k_pstar_bus_type_gpio, mode);
  if (config == NULL) {
    return NULL;
  }

  /* Configure GPIO-specific parameters */
  memset(&config->config.gpio.config, 0, sizeof(gpio_config_t));
  config->config.gpio.config.pin_bit_mask = 0; /* No pins configured by default */
  config->config.gpio.config.mode         = GPIO_MODE_INPUT_OUTPUT; /* Default mode */
  config->config.gpio.config.pull_up_en   = GPIO_PULLUP_DISABLE;
  config->config.gpio.config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config->config.gpio.config.intr_type    = GPIO_INTR_DISABLE; /* Interrupts disabled by default */

  log_info(TAG, "GPIO Created", "Created GPIO bus configuration for '%s'", name);
  return config;
}

pstar_bus_config_t* pstar_bus_config_create_sdio(const char*         name,
                                                 uint32_t            host_flags,
                                                 uint8_t             slot,
                                                 pstar_common_mode_t mode)
{
  pstar_bus_config_t* config =
    priv_pstar_bus_config_create_common(name, k_pstar_bus_type_sdio, mode);
  if (config == NULL) {
    return NULL;
  }

  /* Configure SDIO-specific parameters */
  /* Initialize host configuration with default values */
  config->config.sdio.host = (sdmmc_host_t)SDMMC_HOST_DEFAULT();

  /* Apply custom host flags if provided */
  if (host_flags != 0) {
    config->config.sdio.host.flags = host_flags;
  }

  /* Apply slot if provided */
  config->config.sdio.host.slot = slot;

  /* Initialize slot configuration with default values */
  config->config.sdio.slot_config = (sdmmc_slot_config_t)SDMMC_SLOT_CONFIG_DEFAULT();

  /* Initialize card pointer to NULL */
  config->config.sdio.card = NULL;

  log_info(TAG, "SDIO Created", "Created SDIO bus configuration for '%s', slot %d", name, slot);
  return config;
}

esp_err_t pstar_bus_config_destroy(pstar_bus_config_t* config)
{
  if (config == NULL) {
    log_error(TAG, "Destroy Error", "Bus configuration pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  /* If the bus is initialized, deinitialize it first */
  if (config->initialized) {
    esp_err_t result = pstar_bus_config_deinit(config);
    if (result != ESP_OK) {
      log_error(TAG,
                "Deinit Error",
                "Failed to deinitialize bus '%s' before destroying: %s",
                config->name ? config->name : "UNKNOWN",
                esp_err_to_name(result));

      /* Despite the error, we'll proceed with cleanup to avoid resource leaks */
      /* This is a conscious decision to continue with cleanup, even if deinit failed */
      log_warn(TAG,
               "Destroy Warning",
               "Proceeding with cleanup of bus '%s' despite deinit failure",
               config->name ? config->name : "UNKNOWN");
    }
  }

  /* Log the destruction */
  log_info(TAG,
           "Destroying Bus",
           "Destroying bus configuration for '%s'",
           config->name ? config->name : "UNKNOWN");

  /* Clean up and free the configuration */
  priv_pstar_bus_config_cleanup(config);

  /* IMPORTANT: After this call, the config pointer is invalid.
   * Callers should set their copy of this pointer to NULL after calling this function.
   */
  return ESP_OK;
}

esp_err_t pstar_bus_config_init(pstar_bus_config_t* config)
{
  if (config == NULL) {
    log_error(TAG, "Init Error", "Bus configuration pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if already initialized */
  if (config->initialized) {
    log_warn(TAG,
             "Already Init",
             "Bus '%s' is already initialized",
             config->name ? config->name : "UNKNOWN");
    return ESP_OK;
  }

  esp_err_t result = ESP_OK;

  /* Initialize the bus based on its type */
  switch (config->type) {
    case k_pstar_bus_type_i2c:
      log_info(TAG,
               "I2C Init",
               "Initializing I2C bus '%s', port %d",
               config->name ? config->name : "UNKNOWN",
               config->config.i2c.port);

      /* Register I2C pins with pin validator */
      if (config->config.i2c.config.sda_io_num != -1) {
        result = pin_validator_register_pin(config->config.i2c.config.sda_io_num,
                                            config->name ? config->name : "UNKNOWN",
                                            "I2C SDA",
                                            false);
        if (result != ESP_OK) {
          log_error(TAG,
                    "Pin Registration Error",
                    "Failed to register SDA pin for I2C bus '%s': %s",
                    config->name ? config->name : "UNKNOWN",
                    esp_err_to_name(result));
          return result;
        }
      }

      if (config->config.i2c.config.scl_io_num != -1) {
        result = pin_validator_register_pin(config->config.i2c.config.scl_io_num,
                                            config->name ? config->name : "UNKNOWN",
                                            "I2C SCL",
                                            false);
        if (result != ESP_OK) {
          log_error(TAG,
                    "Pin Registration Error",
                    "Failed to register SCL pin for I2C bus '%s': %s",
                    config->name ? config->name : "UNKNOWN",
                    esp_err_to_name(result));
          return result;
        }
      }

      /* Configure I2C bus */
      result = i2c_param_config(config->config.i2c.port, &config->config.i2c.config);
      if (result == ESP_OK) {
        result = i2c_driver_install(config->config.i2c.port,
                                    config->config.i2c.config.mode,
                                    0,  /* RX buffer size (not used in master mode) */
                                    0,  /* TX buffer size (not used in master mode) */
                                    0); /* Flags */
      }
      break;

    case k_pstar_bus_type_spi:
      log_info(TAG,
               "SPI Init",
               "Initializing SPI bus '%s', host %d",
               config->name ? config->name : "UNKNOWN",
               config->config.spi.host);

      /* Register SPI pins with pin validator */
      if (config->config.spi.bus_config.mosi_io_num != -1) {
        // --- FIX --- MOSI (Master Out) is DI (Data In) for the peripheral
        result = pin_validator_register_pin(config->config.spi.bus_config.mosi_io_num,
                                            config->name ? config->name : "UNKNOWN",
                                            "SPI DI (MOSI)", // Updated comment
                                            false);
        if (result != ESP_OK) {
          log_error(TAG,
                    "Pin Registration Error",
                    "Failed to register DI (MOSI) pin for SPI bus '%s': %s", // Updated log
                    config->name ? config->name : "UNKNOWN",
                    esp_err_to_name(result));
          return result;
        }
      }

      if (config->config.spi.bus_config.miso_io_num != -1) {
        // --- FIX --- MISO (Master In) is DO (Data Out) for the peripheral
        result = pin_validator_register_pin(config->config.spi.bus_config.miso_io_num,
                                            config->name ? config->name : "UNKNOWN",
                                            "SPI DO (MISO)", // Updated comment
                                            false);
        if (result != ESP_OK) {
          log_error(TAG,
                    "Pin Registration Error",
                    "Failed to register DO (MISO) pin for SPI bus '%s': %s", // Updated log
                    config->name ? config->name : "UNKNOWN",
                    esp_err_to_name(result));
          return result;
        }
      }

      if (config->config.spi.bus_config.sclk_io_num != -1) {
        result = pin_validator_register_pin(config->config.spi.bus_config.sclk_io_num,
                                            config->name ? config->name : "UNKNOWN",
                                            "SPI SCLK",
                                            false);
        if (result != ESP_OK) {
          log_error(TAG,
                    "Pin Registration Error",
                    "Failed to register SCLK pin for SPI bus '%s': %s",
                    config->name ? config->name : "UNKNOWN",
                    esp_err_to_name(result));
          return result;
        }
      }

      if (config->config.spi.dev_config.spics_io_num != -1) {
        result = pin_validator_register_pin(config->config.spi.dev_config.spics_io_num,
                                            config->name ? config->name : "UNKNOWN",
                                            "SPI CS",
                                            false);
        if (result != ESP_OK) {
          log_error(TAG,
                    "Pin Registration Error",
                    "Failed to register CS pin for SPI bus '%s': %s",
                    config->name ? config->name : "UNKNOWN",
                    esp_err_to_name(result));
          return result;
        }
      }

      /* Initialize SPI bus ONLY. Device added on first use. */
      result = spi_bus_initialize(config->config.spi.host,
                                  &config->config.spi.bus_config,
                                  SPI_DMA_CH_AUTO);
      /* Clear handle, device will be added later */
      config->handle = NULL;
      break;

    case k_pstar_bus_type_uart:
      log_info(TAG,
               "UART Init",
               "Initializing UART bus '%s', port %d",
               config->name ? config->name : "UNKNOWN",
               config->config.uart.port);

      /* Register UART pins with pin validator */
      /* Note/TODO: UART pins are configured internally by the ESP-IDF UART driver.
       * We need to extract pin numbers from the uart_config_t or find another
       * way to get these pins. For now, we won't register UART pins. */

      /* Configure UART */
      result = uart_param_config(config->config.uart.port, &config->config.uart.config);
      if (result == ESP_OK) {
        result = uart_driver_install(config->config.uart.port,
                                     config->config.uart.rx_buffer_size,
                                     config->config.uart.tx_buffer_size,
                                     0,    /* Queue size (0 for none) */
                                     NULL, /* Queue handle (not used) */
                                     0);   /* Interrupt flags */
      }
      break;

    case k_pstar_bus_type_gpio:
      log_info(TAG,
               "GPIO Init",
               "Initializing GPIO bus '%s'",
               config->name ? config->name : "UNKNOWN");

      /* Register GPIO pins with pin validator */
      uint64_t pin_mask = config->config.gpio.config.pin_bit_mask;
      for (int pin = 0; pin < GPIO_NUM_MAX; pin++) {
        if ((pin_mask >> pin) & 0x1) {
          bool skip_registration = false;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_DETECTION_ENABLED
          /* Check if this is the SD card GPIO bus and the specific CD pin */
          /* Prevent double registration - SD HAL will register it. */
          if (config->name &&
              strcmp(config->name, CONFIG_PSTAR_KCONFIG_SD_CARD_GPIO_BUS_NAME) == 0 &&
              pin == CONFIG_PSTAR_KCONFIG_SD_CARD_DET_GPIO) {
            log_debug(
              TAG,
              "Skip Registration",
              "Skipping registration of SD Card Detect pin %d in GPIO bus config (handled by SD HAL)",
              pin);
            skip_registration = true;
          }
#endif /* DETECTION_ENABLED */
#endif /* SD_CARD_ENABLED */

          if (!skip_registration) {
            /* This pin is used, register it */
            result = pin_validator_register_pin(pin,
                                                config->name ? config->name : "UNKNOWN",
                                                "GPIO",
                                                false); /* Default non-shareable for generic GPIO */
            if (result != ESP_OK) {
              log_error(TAG,
                        "Pin Registration Error",
                        "Failed to register GPIO pin %d for bus '%s': %s",
                        pin,
                        config->name ? config->name : "UNKNOWN",
                        esp_err_to_name(result));
              return result;
            }
          }
        }
      }

      /* Only configure GPIO if pins are specified (non-zero mask) */
      if (config->config.gpio.config.pin_bit_mask != 0) {
        result = gpio_config(&config->config.gpio.config);
      }
      break;

    case k_pstar_bus_type_sdio:
      log_info(TAG,
               "SDIO Init",
               "Initializing SDIO bus '%s', slot %d",
               config->name ? config->name : "UNKNOWN",
               config->config.sdio.host.slot);

      /* Register SDIO pins with pin validator */
      /* Note/TODO: SDIO pins are configured internally by the ESP-IDF SDMMC driver.
       * We need to find a way to get these pins. For now, we won't register SDIO pins. */

      /* For SDIO, we just set the initialized flag here.
       * The actual card detection and mounting is handled
       * by the SD card HAL. */
      result = ESP_OK;
      break;

    default:
      log_error(TAG,
                "Unknown Type",
                "Unknown bus type %d for bus '%s'",
                config->type,
                config->name ? config->name : "UNKNOWN");
      return ESP_ERR_INVALID_ARG;
  }

  if (result != ESP_OK) {
    log_error(TAG,
              "Init Failed",
              "Failed to initialize bus '%s': %s",
              config->name ? config->name : "UNKNOWN",
              esp_err_to_name(result));
    return result;
  }

  /* Mark as initialized */
  config->initialized = true;

  log_info(TAG,
           "Init Success",
           "Successfully initialized bus '%s' (%s)",
           config->name ? config->name : "UNKNOWN",
           pstar_bus_type_to_string(config->type));

  return ESP_OK;
}

esp_err_t pstar_bus_config_deinit(pstar_bus_config_t* config)
{
  if (config == NULL) {
    log_error(TAG, "Deinit Error", "Bus configuration pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  /* If not initialized, nothing to do */
  if (!config->initialized) {
    log_warn(TAG,
             "Not Init",
             "Bus '%s' is not initialized, nothing to deinitialize",
             config->name ? config->name : "UNKNOWN");
    return ESP_OK;
  }

  esp_err_t result = ESP_OK;

  /* Deinitialize the bus based on its type */
  switch (config->type) {
    case k_pstar_bus_type_i2c:
      log_info(TAG,
               "I2C Deinit",
               "Deinitializing I2C bus '%s', port %d",
               config->name ? config->name : "UNKNOWN",
               config->config.i2c.port);

      /* Unregister I2C pins from pin validator */
      if (config->config.i2c.config.sda_io_num != -1) {
        pin_validator_unregister_pin(config->config.i2c.config.sda_io_num,
                                     config->name ? config->name : "UNKNOWN");
      }

      if (config->config.i2c.config.scl_io_num != -1) {
        pin_validator_unregister_pin(config->config.i2c.config.scl_io_num,
                                     config->name ? config->name : "UNKNOWN");
      }

      result = i2c_driver_delete(config->config.i2c.port);
      break;

    case k_pstar_bus_type_spi:
      log_info(TAG,
               "SPI Deinit",
               "Deinitializing SPI bus '%s', host %d",
               config->name ? config->name : "UNKNOWN",
               config->config.spi.host);

      /* Unregister SPI pins from pin validator */
      if (config->config.spi.bus_config.mosi_io_num != -1) {
        pin_validator_unregister_pin(config->config.spi.bus_config.mosi_io_num,
                                     config->name ? config->name : "UNKNOWN");
      }

      if (config->config.spi.bus_config.miso_io_num != -1) {
        pin_validator_unregister_pin(config->config.spi.bus_config.miso_io_num,
                                     config->name ? config->name : "UNKNOWN");
      }

      if (config->config.spi.bus_config.sclk_io_num != -1) {
        pin_validator_unregister_pin(config->config.spi.bus_config.sclk_io_num,
                                     config->name ? config->name : "UNKNOWN");
      }

      if (config->config.spi.dev_config.spics_io_num != -1) {
        pin_validator_unregister_pin(config->config.spi.dev_config.spics_io_num,
                                     config->name ? config->name : "UNKNOWN");
      }

      if (config->handle != NULL) {
        esp_err_t dev_result = spi_bus_remove_device((spi_device_handle_t)config->handle);
        if (dev_result != ESP_OK) {
          log_warn(TAG,
                   "SPI Device Warning",
                   "Failed to remove SPI device during deinit: %s",
                   esp_err_to_name(dev_result));
          /* Continue with bus deinitialization despite the warning */
        } else {
          log_debug(TAG, "SPI Device Removed", "Successfully removed SPI device during deinit");
        }
        config->handle = NULL; /* Clear handle regardless of removal result */
      }

      /* Then free the entire bus */
      result = spi_bus_free(config->config.spi.host);
      break;

    case k_pstar_bus_type_uart:
      log_info(TAG,
               "UART Deinit",
               "Deinitializing UART bus '%s', port %d",
               config->name ? config->name : "UNKNOWN",
               config->config.uart.port);

      /* No pins to unregister for UART as we didn't register them */

      result = uart_driver_delete(config->config.uart.port);
      break;

    case k_pstar_bus_type_gpio:
      log_info(TAG,
               "GPIO Deinit",
               "Deinitializing GPIO bus '%s'",
               config->name ? config->name : "UNKNOWN");

      /* Unregister GPIO pins from pin validator */
      uint64_t pin_mask = config->config.gpio.config.pin_bit_mask;
      for (int pin = 0; pin < GPIO_NUM_MAX; pin++) {
        if ((pin_mask >> pin) & 0x1) {
          /* This pin is used, unregister it */
          pin_validator_unregister_pin(pin, config->name ? config->name : "UNKNOWN");
        }
      }

      /* No specific deinitialize for GPIO, just set the flag */
      result = ESP_OK;
      break;

    case k_pstar_bus_type_sdio:
      log_info(TAG,
               "SDIO Deinit",
               "Deinitializing SDIO bus '%s'",
               config->name ? config->name : "UNKNOWN");

      /* No pins to unregister for SDIO as we didn't register them */

      /* For SDIO, we just clear the initialized flag here.
       * The actual unmounting is handled by the SD card HAL. */
      result = ESP_OK;
      break;

    default:
      log_error(TAG,
                "Unknown Type",
                "Unknown bus type %d for bus '%s'",
                config->type,
                config->name ? config->name : "UNKNOWN");
      return ESP_ERR_INVALID_ARG;
  }

  if (result != ESP_OK) {
    log_error(TAG,
              "Deinit Failed",
              "Failed to deinitialize bus '%s': %s",
              config->name ? config->name : "UNKNOWN",
              esp_err_to_name(result));
    return result;
  }

  /* Mark as not initialized */
  config->initialized = false;

  log_info(TAG,
           "Deinit Success",
           "Successfully deinitialized bus '%s'",
           config->name ? config->name : "UNKNOWN");

  return ESP_OK;
}

/* Private Functions **********************************************************/

/**
 * @brief Clean up and free memory for a bus configuration.
 *
 * @param[in] config Pointer to the bus configuration to clean up.
 */
static void priv_pstar_bus_config_cleanup(pstar_bus_config_t* config)
{
  if (config == NULL) {
    return;
  }

  /* Free the name if it was allocated
   * Important: The name field is a const char* for API usage, but
   * we allocated it with strdup() and own its memory, so we can safely free it.
   * We use a void* cast which is safe since we're freeing memory that we allocated.
   */
  if (config->name) {
    free((void*)config->name); /* Safe cast for memory we own */
    config->name = NULL;
  }

  /* Set any other pointers to NULL before freeing config */
  config->handle   = NULL;
  config->user_ctx = NULL;
  config->next     = NULL;

  /* Free the bus configuration structure itself */
  free(config);
  /* Note: Cannot set config to NULL here since we only have a local copy of the pointer */
}

/**
 * @brief Create a generic bus configuration with common parameters.
 *
 * @param[in] name Name of the bus.
 * @param[in] type Type of the bus.
 * @param[in] mode Operation mode.
 * @return bus_config_t* Pointer to the created configuration, or NULL on failure.
 */
static pstar_bus_config_t* priv_pstar_bus_config_create_common(const char*         name,
                                                               pstar_bus_type_t    type,
                                                               pstar_common_mode_t mode)
{
  if (name == NULL) {
    log_error(TAG, "Create Error", "Bus name cannot be NULL");
    return NULL;
  }

  /* Allocate memory for the bus configuration */
  pstar_bus_config_t* config = (pstar_bus_config_t*)malloc(sizeof(pstar_bus_config_t));
  if (config == NULL) {
    log_error(TAG, "Memory Error", "Failed to allocate memory for bus configuration");
    return NULL;
  }

  /* Initialize the configuration to zeros */
  memset(config, 0, sizeof(pstar_bus_config_t));

  /* Make a copy of the bus name
   * Note: We use strdup to allocate memory for the name string.
   * We store it as const char* to prevent accidental modification,
   * but we're responsible for freeing it when no longer needed. */
  char* name_copy = strdup(name);
  if (name_copy == NULL) {
    log_error(TAG, "Memory Error", "Failed to allocate memory for bus name");
    free(config);
    config = NULL;
    return NULL;
  }

  /* Store the name copy in the config - we'll free it in cleanup */
  config->name = name_copy;

  /* Set common parameters */
  config->type        = type;
  config->mode        = mode;
  config->initialized = false;
  config->handle      = NULL;
  config->user_ctx    = NULL;
  config->next        = NULL;

  return config;
}
