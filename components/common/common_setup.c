/* components/common/common_setup.c */

#include "common/common_setup.h"
#include "common/bus_manager.h"
#include "log_handler.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"
#include "driver/uart.h"

/* Constants ******************************************************************/

const char* const common_setup_tag = "Common Setup";

/* Public Functions ***********************************************************/

esp_err_t common_setup_gpio(uint64_t pin_bit_mask, 
                           gpio_mode_t mode,
                           gpio_pullup_t pull_up_en,
                           gpio_pulldown_t pull_down_en,
                           gpio_int_type_t intr_type,
                           const char* const tag)
{
  if (pin_bit_mask == 0) {
    log_warn(tag ? tag : common_setup_tag, 
             "GPIO Warning", 
             "No GPIO pins specified for setup");
    return ESP_OK;
  }

  log_info(tag ? tag : common_setup_tag, 
           "GPIO Setup", 
           "Configuring GPIO pins");

  /* Configure GPIO pins with the specified settings */
  gpio_config_t io_conf = {
    .pin_bit_mask = pin_bit_mask,
    .mode         = mode,
    .pull_up_en   = pull_up_en,
    .pull_down_en = pull_down_en,
    .intr_type    = intr_type
  };

  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    log_error(tag ? tag : common_setup_tag, 
              "GPIO Error", 
              "Failed to configure GPIO pins: %s", 
              esp_err_to_name(ret));
  } else {
    log_info(tag ? tag : common_setup_tag, 
             "GPIO Success", 
             "GPIO pins configured successfully");
  }

  return ret;
}

esp_err_t common_setup_mutex(SemaphoreHandle_t* mutex, const char* const tag)
{
  if (mutex == NULL) {
    log_error(tag ? tag : common_setup_tag, 
              "Mutex Error", 
              "Mutex pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  if (*mutex != NULL) {
    log_warn(tag ? tag : common_setup_tag, 
             "Mutex Warning", 
             "Mutex is already initialized");
    return ESP_OK;
  }

  log_info(tag ? tag : common_setup_tag, 
           "Mutex Setup", 
           "Creating mutex");

  /* Create mutex */
  *mutex = xSemaphoreCreateMutex();
  if (*mutex == NULL) {
    log_error(tag ? tag : common_setup_tag, 
              "Mutex Error", 
              "Failed to create mutex");
    return ESP_ERR_NO_MEM;
  }

  log_info(tag ? tag : common_setup_tag, 
           "Mutex Success", 
           "Mutex created successfully");

  return ESP_OK;
}

esp_err_t common_setup_i2c(int i2c_bus, int scl_io, int sda_io, uint32_t freq_hz, const char* const tag)
{
  esp_err_t overall_status = ESP_OK;

  log_info(tag ? tag : common_setup_tag, 
           "I2C Setup", 
           "Setting up I2C bus %d (SCL: %d, SDA: %d, Freq: %lu Hz)", 
           i2c_bus, 
           scl_io, 
           sda_io, 
           freq_hz);

  /* Initialize I2C bus */
  esp_err_t ret = bus_manager_i2c_init(scl_io, sda_io, freq_hz, i2c_bus);
  if (ret != ESP_OK) {
    log_error(tag ? tag : common_setup_tag, 
              "I2C Error", 
              "Failed to initialize I2C bus %d: %s", 
              i2c_bus, 
              esp_err_to_name(ret));
    overall_status = ret;
  } else {
    log_info(tag ? tag : common_setup_tag, 
             "I2C Success", 
             "I2C bus %d set up successfully", 
             i2c_bus);
  }

  return overall_status;
}

esp_err_t common_setup_spi(int host, int mosi_io, int miso_io, int sclk_io, int dma_chan, const char* const tag)
{
  esp_err_t overall_status = ESP_OK;

  log_info(tag ? tag : common_setup_tag, 
           "SPI Setup", 
           "Setting up SPI host %d (MOSI: %d, MISO: %d, SCLK: %d, DMA: %d)", 
           host, 
           mosi_io, 
           miso_io, 
           sclk_io, 
           dma_chan);

  /* Initialize SPI bus */
  esp_err_t ret = bus_manager_spi_init(mosi_io, miso_io, sclk_io, host, dma_chan);
  if (ret != ESP_OK) {
    log_error(tag ? tag : common_setup_tag, 
              "SPI Error", 
              "Failed to initialize SPI host %d: %s", 
              host, 
              esp_err_to_name(ret));
    overall_status = ret;
  } else {
    log_info(tag ? tag : common_setup_tag, 
             "SPI Success", 
             "SPI host %d set up successfully", 
             host);
  }

  return overall_status;
}

esp_err_t common_setup_uart(int uart_num, int tx_io, int rx_io, uint32_t baud_rate, 
                           size_t rx_buffer_size, size_t tx_buffer_size, const char* const tag)
{
  esp_err_t overall_status = ESP_OK;

  log_info(tag ? tag : common_setup_tag, 
           "UART Setup", 
           "Setting up UART port %d (TX: %d, RX: %d, Baud: %lu)", 
           uart_num, 
           tx_io, 
           rx_io, 
           baud_rate);

  /* Initialize UART */
  esp_err_t ret = bus_manager_uart_init(tx_io, rx_io, baud_rate, uart_num, rx_buffer_size, tx_buffer_size);
  if (ret != ESP_OK) {
    log_error(tag ? tag : common_setup_tag, 
              "UART Error", 
              "Failed to initialize UART port %d: %s", 
              uart_num, 
              esp_err_to_name(ret));
    overall_status = ret;
  } else {
    log_info(tag ? tag : common_setup_tag, 
             "UART Success", 
             "UART port %d set up successfully", 
             uart_num);
  }

  return overall_status;
}

esp_err_t common_setup_multiple(const char* const tag, 
                               const char* const setup_name,
                               esp_err_t (*setup_funcs[])(void),
                               int num_funcs)
{
  if (setup_funcs == NULL || num_funcs <= 0) {
    log_warn(tag ? tag : common_setup_tag, 
             "Multiple Warning", 
             "No setup functions specified");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(tag ? tag : common_setup_tag, 
           "Multiple Setup", 
           "Beginning %s setup with %d functions", 
           setup_name ? setup_name : "multiple", 
           num_funcs);

  esp_err_t overall_status = ESP_OK;

  /* Execute all setup functions, even if some fail */
  for (int i = 0; i < num_funcs; i++) {
    if (setup_funcs[i] == NULL) {
      log_warn(tag ? tag : common_setup_tag, 
               "Function Warning", 
               "Setup function %d is NULL, skipping", 
               i);
      continue;
    }

    esp_err_t ret = setup_funcs[i]();
    if (ret != ESP_OK) {
      log_error(tag ? tag : common_setup_tag, 
                "Function Error", 
                "Setup function %d failed with error: %s", 
                i, 
                esp_err_to_name(ret));
      overall_status = ESP_FAIL;
    } else {
      log_info(tag ? tag : common_setup_tag, 
               "Function Success", 
               "Setup function %d completed successfully", 
               i);
    }
  }

  if (overall_status == ESP_OK) {
    log_info(tag ? tag : common_setup_tag, 
             "Multiple Success", 
             "%s setup completed successfully", 
             setup_name ? setup_name : "Multiple");
  } else {
    log_warn(tag ? tag : common_setup_tag, 
             "Multiple Warning", 
             "%s setup completed with some errors", 
             setup_name ? setup_name : "Multiple");
  }

  return overall_status;
} 