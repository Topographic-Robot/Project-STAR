/* components/sensors/common_sensor_setup.c */

#include "common/common_setup.h"
#include "common/common_cleanup.h"
#include "common/i2c.h"
#include "log_handler.h"
#include "driver/gpio.h"

/* Constants ******************************************************************/

static const char* const sensor_setup_tag = "Sensor Setup";
static const char* const sensor_cleanup_tag = "Sensor Cleanup";

/* I2C Bus Configuration for Sensors */
#define SENSOR_I2C_BUS       I2C_NUM_0
#define SENSOR_I2C_SCL_PIN   22
#define SENSOR_I2C_SDA_PIN   21
#define SENSOR_I2C_FREQ_HZ   400000  /* 400 KHz */

/* GPIO Configuration for Sensors */
#define DHT22_DATA_PIN       4
#define MQ135_ANALOG_PIN     36

/* Private Function Prototypes ************************************************/

static esp_err_t priv_setup_i2c_sensors(void);
static esp_err_t priv_setup_gpio_sensors(void);
static esp_err_t priv_cleanup_i2c_sensors(void);
static esp_err_t priv_cleanup_gpio_sensors(void);

/* Public Functions ***********************************************************/

/**
 * @brief Sets up all sensors
 * 
 * This function initializes all the necessary hardware resources for sensors,
 * including I2C bus and GPIO pins.
 * 
 * @return ESP_OK if all setup operations succeeded, ESP_FAIL otherwise
 */
esp_err_t common_sensor_setup(void)
{
  log_info(sensor_setup_tag, "Setup Start", "Beginning sensor hardware setup");
  
  esp_err_t (*setup_funcs[])(void) = {
    priv_setup_i2c_sensors,
    priv_setup_gpio_sensors
  };
  
  esp_err_t ret = common_setup_multiple(sensor_setup_tag, 
                                       "sensor hardware", 
                                       setup_funcs, 
                                       sizeof(setup_funcs) / sizeof(setup_funcs[0]));
  
  if (ret == ESP_OK) {
    log_info(sensor_setup_tag, "Setup Complete", "Sensor hardware setup completed successfully");
  } else {
    log_warn(sensor_setup_tag, "Setup Warning", "Sensor hardware setup completed with some errors");
  }
  
  return ret;
}

/**
 * @brief Cleans up all sensors
 * 
 * This function releases all the hardware resources used by sensors,
 * including I2C bus and GPIO pins.
 * 
 * @return ESP_OK if all cleanup operations succeeded, ESP_FAIL otherwise
 */
esp_err_t common_sensor_cleanup(void)
{
  log_info(sensor_cleanup_tag, "Cleanup Start", "Beginning sensor hardware cleanup");
  
  esp_err_t (*cleanup_funcs[])(void) = {
    priv_cleanup_i2c_sensors,
    priv_cleanup_gpio_sensors
  };
  
  esp_err_t ret = common_cleanup_multiple(sensor_cleanup_tag, 
                                         "sensor hardware", 
                                         cleanup_funcs, 
                                         sizeof(cleanup_funcs) / sizeof(cleanup_funcs[0]));
  
  if (ret == ESP_OK) {
    log_info(sensor_cleanup_tag, "Cleanup Complete", "Sensor hardware cleanup completed successfully");
  } else {
    log_warn(sensor_cleanup_tag, "Cleanup Warning", "Sensor hardware cleanup completed with some errors");
  }
  
  return ret;
}

/* Private Functions **********************************************************/

/**
 * @brief Sets up I2C bus for sensors
 * 
 * This function initializes the I2C bus used by sensors like BH1750, QMC5883L, MPU6050, and CCS811.
 * 
 * @return ESP_OK if setup succeeded, error code otherwise
 */
static esp_err_t priv_setup_i2c_sensors(void)
{
  log_info(sensor_setup_tag, "I2C Setup", "Setting up I2C bus for sensors");
  
  return common_setup_i2c(SENSOR_I2C_BUS, 
                         SENSOR_I2C_SCL_PIN, 
                         SENSOR_I2C_SDA_PIN, 
                         SENSOR_I2C_FREQ_HZ, 
                         sensor_setup_tag);
}

/**
 * @brief Sets up GPIO pins for sensors
 * 
 * This function initializes the GPIO pins used by sensors like DHT22 and MQ135.
 * 
 * @return ESP_OK if setup succeeded, error code otherwise
 */
static esp_err_t priv_setup_gpio_sensors(void)
{
  log_info(sensor_setup_tag, "GPIO Setup", "Setting up GPIO pins for sensors");
  
  /* Configure DHT22 data pin as input with pull-up */
  esp_err_t ret = common_setup_gpio((1ULL << DHT22_DATA_PIN), 
                                   GPIO_MODE_INPUT, 
                                   GPIO_PULLUP_ENABLE, 
                                   GPIO_PULLDOWN_DISABLE, 
                                   GPIO_INTR_DISABLE, 
                                   sensor_setup_tag);
  if (ret != ESP_OK) {
    return ret;
  }
  
  /* Configure MQ135 analog pin as input */
  ret = common_setup_gpio((1ULL << MQ135_ANALOG_PIN), 
                         GPIO_MODE_INPUT, 
                         GPIO_PULLUP_DISABLE, 
                         GPIO_PULLDOWN_DISABLE, 
                         GPIO_INTR_DISABLE, 
                         sensor_setup_tag);
  
  return ret;
}

/**
 * @brief Cleans up I2C bus for sensors
 * 
 * This function releases the I2C bus used by sensors.
 * 
 * @return ESP_OK if cleanup succeeded, error code otherwise
 */
static esp_err_t priv_cleanup_i2c_sensors(void)
{
  log_info(sensor_cleanup_tag, "I2C Cleanup", "Cleaning up I2C bus for sensors");
  
  return common_cleanup_i2c(SENSOR_I2C_BUS, 
                           SENSOR_I2C_SCL_PIN, 
                           SENSOR_I2C_SDA_PIN, 
                           sensor_cleanup_tag);
}

/**
 * @brief Cleans up GPIO pins for sensors
 * 
 * This function releases the GPIO pins used by sensors.
 * 
 * @return ESP_OK if cleanup succeeded, error code otherwise
 */
static esp_err_t priv_cleanup_gpio_sensors(void)
{
  log_info(sensor_cleanup_tag, "GPIO Cleanup", "Cleaning up GPIO pins for sensors");
  
  /* Release sensor GPIO pins */
  uint64_t pin_mask = (1ULL << DHT22_DATA_PIN) | (1ULL << MQ135_ANALOG_PIN);
  
  return common_cleanup_gpio(pin_mask, sensor_cleanup_tag);
} 