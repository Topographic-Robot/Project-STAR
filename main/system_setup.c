/* main/system_setup.c */

#include "system_setup.h"
#include "log_handler.h"
#include "common/common_setup.h"
#include "common/common_cleanup.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"
#include "driver/uart.h"

/* Constants ******************************************************************/

const char* const system_setup_tag = "System Setup";

/* I2C Bus Configuration */
#define SENSOR_I2C_BUS       I2C_NUM_0
#define SENSOR_I2C_SCL_PIN   22
#define SENSOR_I2C_SDA_PIN   21
#define SENSOR_I2C_FREQ_HZ   400000  /* 400 KHz */

#define MOTOR_I2C_BUS        I2C_NUM_1
#define MOTOR_I2C_SCL_PIN    32
#define MOTOR_I2C_SDA_PIN    33
#define MOTOR_I2C_FREQ_HZ    100000  /* 100 KHz */

/* SPI Bus Configuration */
#define SD_CARD_SPI_HOST     SPI2_HOST
#define SD_CARD_SPI_MOSI     23
#define SD_CARD_SPI_MISO     19
#define SD_CARD_SPI_SCLK     18
#define SD_CARD_SPI_CS       5
#define SD_CARD_SPI_DMA_CHAN 1

/* UART Configuration */
#define DEBUG_UART_NUM       UART_NUM_0
#define DEBUG_UART_TX        1
#define DEBUG_UART_RX        3
#define DEBUG_UART_BAUD      115200
#define DEBUG_UART_RX_BUF    1024
#define DEBUG_UART_TX_BUF    1024

/* Private Function Prototypes ************************************************/

static esp_err_t priv_setup_i2c_buses(void);
static esp_err_t priv_setup_spi_bus(void);
static esp_err_t priv_setup_uart(void);
static esp_err_t priv_setup_gpio(void);

static esp_err_t priv_cleanup_i2c_buses(void);
static esp_err_t priv_cleanup_spi_bus(void);
static esp_err_t priv_cleanup_uart(void);
static esp_err_t priv_cleanup_gpio(void);

/* Public Functions ***********************************************************/

esp_err_t system_setup_init(void)
{
  log_info(system_setup_tag, "Setup Start", "Beginning system hardware setup");
  
  esp_err_t (*setup_funcs[])(void) = {
    priv_setup_i2c_buses,
    priv_setup_spi_bus,
    priv_setup_uart,
    priv_setup_gpio
  };
  
  esp_err_t ret = common_setup_multiple(system_setup_tag, 
                                       "system hardware", 
                                       setup_funcs, 
                                       sizeof(setup_funcs) / sizeof(setup_funcs[0]));
  
  if (ret == ESP_OK) {
    log_info(system_setup_tag, "Setup Complete", "System hardware setup completed successfully");
  } else {
    log_warn(system_setup_tag, "Setup Warning", "System hardware setup completed with some errors");
  }
  
  return ret;
}

esp_err_t system_setup_cleanup(void)
{
  log_info(system_setup_tag, "Cleanup Start", "Beginning system hardware cleanup");
  
  esp_err_t (*cleanup_funcs[])(void) = {
    priv_cleanup_gpio,
    priv_cleanup_uart,
    priv_cleanup_spi_bus,
    priv_cleanup_i2c_buses
  };
  
  esp_err_t ret = common_cleanup_multiple(system_setup_tag, 
                                         "system hardware", 
                                         cleanup_funcs, 
                                         sizeof(cleanup_funcs) / sizeof(cleanup_funcs[0]));
  
  if (ret == ESP_OK) {
    log_info(system_setup_tag, "Cleanup Complete", "System hardware cleanup completed successfully");
  } else {
    log_warn(system_setup_tag, "Cleanup Warning", "System hardware cleanup completed with some errors");
  }
  
  return ret;
}

/* Private Functions **********************************************************/

static esp_err_t priv_setup_i2c_buses(void)
{
  log_info(system_setup_tag, "I2C Setup", "Setting up I2C buses");
  
  /* Set up sensor I2C bus */
  esp_err_t ret = common_setup_i2c(SENSOR_I2C_BUS, 
                                  SENSOR_I2C_SCL_PIN, 
                                  SENSOR_I2C_SDA_PIN, 
                                  SENSOR_I2C_FREQ_HZ, 
                                  system_setup_tag);
  if (ret != ESP_OK) {
    return ret;
  }
  
  /* Set up motor I2C bus */
  ret = common_setup_i2c(MOTOR_I2C_BUS, 
                        MOTOR_I2C_SCL_PIN, 
                        MOTOR_I2C_SDA_PIN, 
                        MOTOR_I2C_FREQ_HZ, 
                        system_setup_tag);
  
  return ret;
}

static esp_err_t priv_setup_spi_bus(void)
{
  log_info(system_setup_tag, "SPI Setup", "Setting up SPI bus for SD card");
  
  return common_setup_spi(SD_CARD_SPI_HOST, 
                         SD_CARD_SPI_MOSI, 
                         SD_CARD_SPI_MISO, 
                         SD_CARD_SPI_SCLK, 
                         SD_CARD_SPI_DMA_CHAN, 
                         system_setup_tag);
}

static esp_err_t priv_setup_uart(void)
{
  log_info(system_setup_tag, "UART Setup", "Setting up UART for debugging");
  
  return common_setup_uart(DEBUG_UART_NUM, 
                          DEBUG_UART_TX, 
                          DEBUG_UART_RX, 
                          DEBUG_UART_BAUD, 
                          DEBUG_UART_RX_BUF, 
                          DEBUG_UART_TX_BUF, 
                          system_setup_tag);
}

static esp_err_t priv_setup_gpio(void)
{
  log_info(system_setup_tag, "GPIO Setup", "Setting up GPIO pins");
  
  /* Set up SD card CS pin as output */
  uint64_t pin_bit_mask = (1ULL << SD_CARD_SPI_CS);
  
  return common_setup_gpio(pin_bit_mask, 
                          GPIO_MODE_OUTPUT, 
                          GPIO_PULLUP_DISABLE, 
                          GPIO_PULLDOWN_DISABLE, 
                          GPIO_INTR_DISABLE, 
                          system_setup_tag);
}

static esp_err_t priv_cleanup_i2c_buses(void)
{
  log_info(system_setup_tag, "I2C Cleanup", "Cleaning up I2C buses");
  
  /* Clean up sensor I2C bus */
  esp_err_t ret = common_cleanup_i2c(SENSOR_I2C_BUS, 
                                    SENSOR_I2C_SCL_PIN, 
                                    SENSOR_I2C_SDA_PIN, 
                                    system_setup_tag);
  if (ret != ESP_OK) {
    return ret;
  }
  
  /* Clean up motor I2C bus */
  ret = common_cleanup_i2c(MOTOR_I2C_BUS, 
                          MOTOR_I2C_SCL_PIN, 
                          MOTOR_I2C_SDA_PIN, 
                          system_setup_tag);
  
  return ret;
}

static esp_err_t priv_cleanup_spi_bus(void)
{
  log_info(system_setup_tag, "SPI Cleanup", "Cleaning up SPI bus");
  
  return common_cleanup_spi(SD_CARD_SPI_HOST, 
                           SD_CARD_SPI_MOSI, 
                           SD_CARD_SPI_MISO, 
                           SD_CARD_SPI_SCLK, 
                           system_setup_tag);
}

static esp_err_t priv_cleanup_uart(void)
{
  log_info(system_setup_tag, "UART Cleanup", "Cleaning up UART");
  
  return common_cleanup_uart(DEBUG_UART_NUM, 
                            DEBUG_UART_TX, 
                            DEBUG_UART_RX, 
                            system_setup_tag);
}

static esp_err_t priv_cleanup_gpio(void)
{
  log_info(system_setup_tag, "GPIO Cleanup", "Cleaning up GPIO pins");
  
  /* Clean up SD card CS pin */
  uint64_t pin_bit_mask = (1ULL << SD_CARD_SPI_CS);
  
  return common_cleanup_gpio(pin_bit_mask, system_setup_tag);
} 