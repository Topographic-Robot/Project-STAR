/* components/common/common_cleanup.c */

#include "common/common_cleanup.h"
#include "common/bus_manager.h"
#include "log_handler.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"
#include "driver/uart.h"

/* Constants ******************************************************************/

const char* const common_cleanup_tag = "Common Cleanup";

/* Public Functions ***********************************************************/

esp_err_t common_cleanup_gpio(uint64_t pin_bit_mask, const char* const tag)
{
  if (pin_bit_mask == 0) {
    log_warn(tag ? tag : common_cleanup_tag, 
             "GPIO Warning", 
             "No GPIO pins specified for cleanup");
    return ESP_OK;
  }

  log_info(tag ? tag : common_cleanup_tag, 
           "GPIO Cleanup", 
           "Resetting GPIO pins");

  /* Reset GPIO pins to input mode with no pull-up/down */
  gpio_config_t io_conf = {
    .pin_bit_mask = pin_bit_mask,
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE
  };

  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    log_warn(tag ? tag : common_cleanup_tag, 
             "GPIO Warning", 
             "Failed to reset GPIO pin configuration: %s", 
             esp_err_to_name(ret));
  } else {
    log_info(tag ? tag : common_cleanup_tag, 
             "GPIO Success", 
             "GPIO pins reset successfully");
  }

  return ret;
}

esp_err_t common_cleanup_mutex(SemaphoreHandle_t* mutex, const char* const tag)
{
  if (mutex == NULL) {
    log_warn(tag ? tag : common_cleanup_tag, 
             "Mutex Warning", 
             "Mutex pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  if (*mutex == NULL) {
    log_info(tag ? tag : common_cleanup_tag, 
             "Mutex Info", 
             "Mutex is already NULL, nothing to clean up");
    return ESP_OK;
  }

  log_info(tag ? tag : common_cleanup_tag, 
           "Mutex Cleanup", 
           "Cleaning up mutex");

  /* Take mutex to ensure no one else is using it */
  if (xSemaphoreTake(*mutex, portMAX_DELAY) != pdTRUE) {
    log_error(tag ? tag : common_cleanup_tag, 
              "Mutex Error", 
              "Failed to take mutex during cleanup");
    return ESP_ERR_TIMEOUT;
  }

  /* Give mutex back before deleting it */
  xSemaphoreGive(*mutex);

  /* Delete mutex */
  vSemaphoreDelete(*mutex);
  *mutex = NULL;

  log_info(tag ? tag : common_cleanup_tag, 
           "Mutex Success", 
           "Mutex cleaned up successfully");

  return ESP_OK;
}

esp_err_t common_cleanup_i2c(int i2c_bus, int scl_io, int sda_io, const char* const tag)
{
  esp_err_t overall_status = ESP_OK;
  esp_err_t temp_ret;

  log_info(tag ? tag : common_cleanup_tag, 
           "I2C Cleanup", 
           "Cleaning up I2C bus %d", 
           i2c_bus);

  /* Clean up I2C bus */
  temp_ret = bus_manager_i2c_cleanup(i2c_bus);
  if (temp_ret != ESP_OK) {
    log_warn(tag ? tag : common_cleanup_tag, 
             "I2C Warning", 
             "Failed to clean up I2C bus %d: %s", 
             i2c_bus, 
             esp_err_to_name(temp_ret));
    overall_status = temp_ret;
  }

  /* Reset GPIO pins */
  if (scl_io >= 0 && sda_io >= 0) {
    uint64_t pin_mask = (1ULL << scl_io) | (1ULL << sda_io);
    temp_ret = common_cleanup_gpio(pin_mask, tag ? tag : common_cleanup_tag);
    if (temp_ret != ESP_OK) {
      overall_status = temp_ret;
    }
  }

  if (overall_status == ESP_OK) {
    log_info(tag ? tag : common_cleanup_tag, 
             "I2C Success", 
             "I2C bus %d cleaned up successfully", 
             i2c_bus);
  }

  return overall_status;
}

esp_err_t common_cleanup_spi(int host, int mosi_io, int miso_io, int sclk_io, const char* const tag)
{
  esp_err_t overall_status = ESP_OK;
  esp_err_t temp_ret;

  log_info(tag ? tag : common_cleanup_tag, 
           "SPI Cleanup", 
           "Cleaning up SPI host %d", 
           host);

  /* Clean up SPI bus */
  temp_ret = bus_manager_spi_cleanup(host);
  if (temp_ret != ESP_OK) {
    log_warn(tag ? tag : common_cleanup_tag, 
             "SPI Warning", 
             "Failed to clean up SPI host %d: %s", 
             host, 
             esp_err_to_name(temp_ret));
    overall_status = temp_ret;
  }

  /* Reset GPIO pins */
  uint64_t pin_mask = 0;
  if (mosi_io >= 0) pin_mask |= (1ULL << mosi_io);
  if (miso_io >= 0) pin_mask |= (1ULL << miso_io);
  if (sclk_io >= 0) pin_mask |= (1ULL << sclk_io);

  if (pin_mask != 0) {
    temp_ret = common_cleanup_gpio(pin_mask, tag ? tag : common_cleanup_tag);
    if (temp_ret != ESP_OK) {
      overall_status = temp_ret;
    }
  }

  if (overall_status == ESP_OK) {
    log_info(tag ? tag : common_cleanup_tag, 
             "SPI Success", 
             "SPI host %d cleaned up successfully", 
             host);
  }

  return overall_status;
}

esp_err_t common_cleanup_uart(int uart_num, int tx_io, int rx_io, const char* const tag)
{
  esp_err_t overall_status = ESP_OK;
  esp_err_t temp_ret;

  log_info(tag ? tag : common_cleanup_tag, 
           "UART Cleanup", 
           "Cleaning up UART port %d", 
           uart_num);

  /* Clean up UART port */
  temp_ret = bus_manager_uart_cleanup(uart_num);
  if (temp_ret != ESP_OK) {
    log_warn(tag ? tag : common_cleanup_tag, 
             "UART Warning", 
             "Failed to clean up UART port %d: %s", 
             uart_num, 
             esp_err_to_name(temp_ret));
    overall_status = temp_ret;
  }

  /* Reset GPIO pins */
  uint64_t pin_mask = 0;
  if (tx_io >= 0) pin_mask |= (1ULL << tx_io);
  if (rx_io >= 0) pin_mask |= (1ULL << rx_io);

  if (pin_mask != 0) {
    temp_ret = common_cleanup_gpio(pin_mask, tag ? tag : common_cleanup_tag);
    if (temp_ret != ESP_OK) {
      overall_status = temp_ret;
    }
  }

  if (overall_status == ESP_OK) {
    log_info(tag ? tag : common_cleanup_tag, 
             "UART Success", 
             "UART port %d cleaned up successfully", 
             uart_num);
  }

  return overall_status;
}

esp_err_t common_cleanup_multiple(const char* const tag, 
                                 const char* const cleanup_name,
                                 esp_err_t (*cleanup_funcs[])(void),
                                 int num_funcs)
{
  if (cleanup_funcs == NULL || num_funcs <= 0) {
    log_warn(tag ? tag : common_cleanup_tag, 
             "Multiple Warning", 
             "No cleanup functions specified");
    return ESP_ERR_INVALID_ARG;
  }

  log_info(tag ? tag : common_cleanup_tag, 
           "Multiple Cleanup", 
           "Beginning %s cleanup with %d functions", 
           cleanup_name ? cleanup_name : "multiple", 
           num_funcs);

  esp_err_t overall_status = ESP_OK;

  /* Execute all cleanup functions, even if some fail */
  for (int i = 0; i < num_funcs; i++) {
    if (cleanup_funcs[i] == NULL) {
      log_warn(tag ? tag : common_cleanup_tag, 
               "Function Warning", 
               "Cleanup function %d is NULL, skipping", 
               i);
      continue;
    }

    esp_err_t temp_ret = cleanup_funcs[i]();
    if (temp_ret != ESP_OK) {
      log_warn(tag ? tag : common_cleanup_tag, 
               "Function Warning", 
               "Cleanup function %d failed: %s", 
               i, 
               esp_err_to_name(temp_ret));
      overall_status = ESP_FAIL;
    }
  }

  if (overall_status == ESP_OK) {
    log_info(tag ? tag : common_cleanup_tag, 
             "Multiple Success", 
             "%s cleanup completed successfully", 
             cleanup_name ? cleanup_name : "Multiple");
  } else {
    log_warn(tag ? tag : common_cleanup_tag, 
             "Multiple Warning", 
             "%s cleanup completed with some failures", 
             cleanup_name ? cleanup_name : "Multiple");
  }

  return overall_status;
} 