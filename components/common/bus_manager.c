/* components/common/bus_manager.c */

#include "common/bus_manager.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"
#include "driver/uart.h"
#include "log_handler.h"

/* Constants ******************************************************************/

const char* const bus_manager_tag = "Bus Manager";

/* Private Variables **********************************************************/

static bool s_i2c_initialized[I2C_NUM_MAX]   = { false };
static bool s_spi_initialized[SPI_HOST_MAX]  = { false };
static bool s_uart_initialized[UART_NUM_MAX] = { false };

/* Public Functions ***********************************************************/

esp_err_t bus_manager_i2c_init(uint8_t    scl_io, 
                               uint8_t    sda_io, 
                               uint32_t   freq_hz, 
                               i2c_port_t i2c_bus)
{
  if (i2c_bus >= I2C_NUM_MAX) {
    log_error(bus_manager_tag, 
              "I2C Error", 
              "Invalid I2C bus number: %d", 
              i2c_bus);
    return ESP_ERR_INVALID_ARG;
  }

  if (s_i2c_initialized[i2c_bus]) {
    log_warn(bus_manager_tag, 
             "I2C Warning", 
             "I2C bus %d is already initialized", 
             i2c_bus);
    return ESP_OK;
  }

  /* The I2C configuration structure */
  i2c_config_t conf = {
    .mode             = I2C_MODE_MASTER,    /* Set the I2C mode to controller */
    .sda_io_num       = sda_io,             /* Set the GPIO number for SDA (data line) */
    .sda_pullup_en    = GPIO_PULLUP_ENABLE, /* Enable internal pull-up for SDA line */
    .scl_io_num       = scl_io,             /* Set the GPIO number for SCL (clock line) */
    .scl_pullup_en    = GPIO_PULLUP_ENABLE, /* Enable internal pull-up for SCL line */
    .master.clk_speed = freq_hz,            /* Set the I2C controller clock frequency */
  };

  /* Configure the I2C bus with the settings specified in 'conf' */
  esp_err_t err = i2c_param_config(i2c_bus, &conf);
  if (err != ESP_OK) {
    log_error(bus_manager_tag, 
              "I2C Config Error", 
              "Failed to configure I2C parameters: %s", 
              esp_err_to_name(err));
    return err;
  }

  /* Install the I2C driver for the controller mode; no RX/TX buffers are required */
  err = i2c_driver_install(i2c_bus, conf.mode, 0, 0, 0);
  if (err != ESP_OK) {
    log_error(bus_manager_tag, 
              "I2C Install Error", 
              "Failed to install I2C driver: %s", 
              esp_err_to_name(err));
    return err;
  }

  s_i2c_initialized[i2c_bus] = true;
  log_info(bus_manager_tag, 
           "I2C Init", 
           "I2C bus %d initialized successfully (SCL: %d, SDA: %d, Freq: %lu Hz)", 
           i2c_bus, 
           scl_io, 
           sda_io, 
           freq_hz);

  return ESP_OK;
}

esp_err_t bus_manager_i2c_cleanup(i2c_port_t i2c_bus)
{
  esp_err_t ret = ESP_OK;
  
  /* Check if the I2C bus is valid */
  if (i2c_bus < 0 || i2c_bus >= I2C_NUM_MAX) {
    log_error(bus_manager_tag, 
              "I2C Parameter Error", 
              "Invalid I2C bus number: %d", 
              i2c_bus);
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the I2C bus is initialized */
  if (!s_i2c_initialized[i2c_bus]) {
    /* Bus not initialized, nothing to do */
    return ESP_OK;
  }
  
  /* Driver deinitialize */
  ret = i2c_driver_delete(i2c_bus);
  if (ret != ESP_OK) {
    log_error(bus_manager_tag, 
              "I2C Cleanup Error", 
              "Failed to delete I2C driver for bus %d: %s", 
              i2c_bus, 
              esp_err_to_name(ret));
    return ret;
  }
  
  /* Mark the bus as uninitialized */
  s_i2c_initialized[i2c_bus] = false;
  
  log_info(bus_manager_tag, 
           "I2C Cleanup", 
           "I2C bus %d cleaned up successfully", 
           i2c_bus);
  
  return ESP_OK;
}

esp_err_t bus_manager_spi_init(int mosi_io, int miso_io, int sclk_io, spi_host_device_t host, int dma_chan)
{
  if (host >= SPI_HOST_MAX) {
    log_error(bus_manager_tag, 
              "SPI Error", 
              "Invalid SPI host: %d", 
              host);
    return ESP_ERR_INVALID_ARG;
  }

  if (s_spi_initialized[host]) {
    log_warn(bus_manager_tag, 
             "SPI Warning", 
             "SPI host %d is already initialized", 
             host);
    return ESP_OK;
  }

  spi_bus_config_t bus_cfg = {
    .mosi_io_num     = mosi_io,
    .miso_io_num     = miso_io,
    .sclk_io_num     = sclk_io,
    .quadwp_io_num   = -1, /* Not used */
    .quadhd_io_num   = -1, /* Not used */
    .max_transfer_sz = 0   /* Use default value */
  };

  esp_err_t err = spi_bus_initialize(host, &bus_cfg, dma_chan);
  if (err != ESP_OK) {
    log_error(bus_manager_tag, 
              "SPI Init Error", 
              "Failed to initialize SPI bus: %s", 
              esp_err_to_name(err));
    return err;
  }

  s_spi_initialized[host] = true;
  log_info(bus_manager_tag, 
           "SPI Init", 
           "SPI host %d initialized successfully (MOSI: %d, MISO: %d, SCLK: %d, DMA: %d)", 
           host, 
           mosi_io, 
           miso_io, 
           sclk_io, 
           dma_chan);

  return ESP_OK;
}

esp_err_t bus_manager_spi_cleanup(spi_host_device_t host)
{
  esp_err_t ret = ESP_OK;
  
  /* Check if the SPI host is valid */
  if (host < 0 || host >= SPI_HOST_MAX) {
    log_error(bus_manager_tag, 
              "SPI Parameter Error", 
              "Invalid SPI host number: %d", 
              host);
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the SPI host is initialized */
  if (!s_spi_initialized[host]) {
    /* Host not initialized, nothing to do */
    return ESP_OK;
  }
  
  /* Driver deinitialize */
  ret = spi_bus_free(host);
  if (ret != ESP_OK) {
    log_error(bus_manager_tag, 
              "SPI Cleanup Error", 
              "Failed to free SPI bus for host %d: %s", 
              host, 
              esp_err_to_name(ret));
    return ret;
  }
  
  /* Mark the host as uninitialized */
  s_spi_initialized[host] = false;
  
  log_info(bus_manager_tag, 
           "SPI Cleanup", 
           "SPI host %d cleaned up successfully", 
           host);
  
  return ESP_OK;
}

esp_err_t bus_manager_uart_init(uint8_t     tx_io, 
                                uint8_t     rx_io, 
                                uint32_t    baud_rate,
                                uart_port_t uart_num,
                                size_t      rx_buffer_size,
                                size_t      tx_buffer_size)
{
  if (uart_num >= UART_NUM_MAX) {
    log_error(bus_manager_tag, 
              "UART Error", 
              "Invalid UART port number: %d", 
              uart_num);
    return ESP_ERR_INVALID_ARG;
  }

  if (s_uart_initialized[uart_num]) {
    log_warn(bus_manager_tag, 
             "UART Warning", 
             "UART port %d is already initialized", 
             uart_num);
    return ESP_OK;
  }

  uart_config_t uart_config = {
    .baud_rate = baud_rate,                /* Set the baud rate (communication speed) */
    .data_bits = UART_DATA_8_BITS,         /* Set data bits (8 bits per word) */
    .parity    = UART_PARITY_DISABLE,      /* No parity check */
    .stop_bits = UART_STOP_BITS_1,         /* Set 1 stop bit */
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, /* Disable hardware flow control */
  };

  /* Configure the UART driver with the specified settings */
  esp_err_t ret = uart_param_config(uart_num, &uart_config);
  if (ret != ESP_OK) {
    log_error(bus_manager_tag, 
              "UART Config Error", 
              "Failed to configure UART parameters: %s", 
              esp_err_to_name(ret));
    return ret;
  }

  /* Set the TX and RX pin numbers for the UART interface */
  ret = uart_set_pin(uart_num, tx_io, rx_io, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    log_error(bus_manager_tag, 
              "UART Pin Error", 
              "Failed to configure UART pins TX:%u RX:%u: %s", 
              tx_io, 
              rx_io, 
              esp_err_to_name(ret));
    return ret;
  }

  /* Install the UART driver with RX and TX buffers */
  ret = uart_driver_install(uart_num, rx_buffer_size, tx_buffer_size, 0, NULL, 0);
  if (ret != ESP_OK) {
    log_error(bus_manager_tag, 
              "UART Driver Error", 
              "Failed to install UART driver: %s", 
              esp_err_to_name(ret));
    return ret;
  }

  s_uart_initialized[uart_num] = true;
  log_info(bus_manager_tag, 
           "UART Init", 
           "UART port %d initialized successfully (TX: %d, RX: %d, Baud: %lu)", 
           uart_num, 
           tx_io, 
           rx_io, 
           baud_rate);

  return ESP_OK;
}

esp_err_t bus_manager_uart_cleanup(uart_port_t uart_num)
{
  esp_err_t ret = ESP_OK;
  
  /* Check if the UART port is valid */
  if (uart_num < 0 || uart_num >= UART_NUM_MAX) {
    log_error(bus_manager_tag, 
              "UART Parameter Error", 
              "Invalid UART port number: %d", 
              uart_num);
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the UART port is initialized */
  if (!s_uart_initialized[uart_num]) {
    /* Port not initialized, nothing to do */
    return ESP_OK;
  }
  
  /* Driver deinitialize */
  ret = uart_driver_delete(uart_num);
  if (ret != ESP_OK) {
    log_error(bus_manager_tag, 
              "UART Cleanup Error", 
              "Failed to delete UART driver for port %d: %s", 
              uart_num, 
              esp_err_to_name(ret));
    return ret;
  }
  
  /* Mark the port as uninitialized */
  s_uart_initialized[uart_num] = false;
  
  log_info(bus_manager_tag, 
           "UART Cleanup", 
           "UART port %d cleaned up successfully", 
           uart_num);
  
  return ESP_OK;
}

esp_err_t bus_manager_cleanup_all(void)
{
  esp_err_t ret = ESP_OK;
  esp_err_t temp_ret = ESP_OK;
  
  log_info(bus_manager_tag, 
           "Cleanup Start", 
           "Beginning cleanup of all communication buses");
  
  /* Clean up all I2C buses */
  for (int i = 0; i < I2C_NUM_MAX; i++) {
    if (s_i2c_initialized[i]) {
      temp_ret = bus_manager_i2c_cleanup(i);
      if (temp_ret != ESP_OK) {
        ret = ESP_FAIL;
      }
    }
  }
  
  /* Clean up all SPI buses */
  for (int i = 0; i < SPI_HOST_MAX; i++) {
    if (s_spi_initialized[i]) {
      temp_ret = bus_manager_spi_cleanup(i);
      if (temp_ret != ESP_OK) {
        ret = ESP_FAIL;
      }
    }
  }
  
  /* Clean up all UART ports */
  for (int i = 0; i < UART_NUM_MAX; i++) {
    if (s_uart_initialized[i]) {
      temp_ret = bus_manager_uart_cleanup(i);
      if (temp_ret != ESP_OK) {
        ret = ESP_FAIL;
      }
    }
  }
  
  if (ret == ESP_OK) {
    log_info(bus_manager_tag, 
             "Cleanup Complete", 
             "All communication buses cleaned up successfully");
  } else {
    log_warn(bus_manager_tag, 
             "Cleanup Warning", 
             "Some communication buses failed to clean up");
  }
  
  return ret;
} 