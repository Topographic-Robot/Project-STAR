/* components/controllers/common_motor_setup.c */

#include "common/common_setup.h"
#include "common/common_cleanup.h"
#include "common/i2c.h"
#include "log_handler.h"
#include "driver/gpio.h"

/* Constants ******************************************************************/

static const char* const motor_setup_tag = "Motor Setup";
static const char* const motor_cleanup_tag = "Motor Cleanup";

/* I2C Bus Configuration for Motors */
#define MOTOR_I2C_BUS       I2C_NUM_1
#define MOTOR_I2C_SCL_PIN   32
#define MOTOR_I2C_SDA_PIN   33
#define MOTOR_I2C_FREQ_HZ   100000  /* 100 KHz */

/* GPIO Configuration for Encoders */
#define ENCODER_A_PIN       25
#define ENCODER_B_PIN       26

/* Private Function Prototypes ************************************************/

static esp_err_t priv_setup_i2c_motors(void);
static esp_err_t priv_setup_gpio_encoders(void);
static esp_err_t priv_cleanup_i2c_motors(void);
static esp_err_t priv_cleanup_gpio_encoders(void);

/* Public Functions ***********************************************************/

/**
 * @brief Sets up all motor controllers
 * 
 * This function initializes all the necessary hardware resources for motor controllers,
 * including I2C bus for PCA9685 and GPIO pins for encoders.
 * 
 * @return ESP_OK if all setup operations succeeded, ESP_FAIL otherwise
 */
esp_err_t common_motor_setup(void)
{
  log_info(motor_setup_tag, "Setup Start", "Beginning motor hardware setup");
  
  esp_err_t (*setup_funcs[])(void) = {
    priv_setup_i2c_motors,
    priv_setup_gpio_encoders
  };
  
  esp_err_t ret = common_setup_multiple(motor_setup_tag, 
                                       "motor hardware", 
                                       setup_funcs, 
                                       sizeof(setup_funcs) / sizeof(setup_funcs[0]));
  
  if (ret == ESP_OK) {
    log_info(motor_setup_tag, "Setup Complete", "Motor hardware setup completed successfully");
  } else {
    log_warn(motor_setup_tag, "Setup Warning", "Motor hardware setup completed with some errors");
  }
  
  return ret;
}

/**
 * @brief Cleans up all motor controllers
 * 
 * This function releases all the hardware resources used by motor controllers,
 * including I2C bus for PCA9685 and GPIO pins for encoders.
 * 
 * @return ESP_OK if all cleanup operations succeeded, ESP_FAIL otherwise
 */
esp_err_t common_motor_cleanup(void)
{
  log_info(motor_cleanup_tag, "Cleanup Start", "Beginning motor hardware cleanup");
  
  esp_err_t (*cleanup_funcs[])(void) = {
    priv_cleanup_i2c_motors,
    priv_cleanup_gpio_encoders
  };
  
  esp_err_t ret = common_cleanup_multiple(motor_cleanup_tag, 
                                         "motor hardware", 
                                         cleanup_funcs, 
                                         sizeof(cleanup_funcs) / sizeof(cleanup_funcs[0]));
  
  if (ret == ESP_OK) {
    log_info(motor_cleanup_tag, "Cleanup Complete", "Motor hardware cleanup completed successfully");
  } else {
    log_warn(motor_cleanup_tag, "Cleanup Warning", "Motor hardware cleanup completed with some errors");
  }
  
  return ret;
}

/* Private Functions **********************************************************/

/**
 * @brief Sets up I2C bus for motor controllers
 * 
 * This function initializes the I2C bus used by PCA9685 PWM controllers.
 * 
 * @return ESP_OK if setup succeeded, error code otherwise
 */
static esp_err_t priv_setup_i2c_motors(void)
{
  log_info(motor_setup_tag, "I2C Setup", "Setting up I2C bus for motor controllers");
  
  return common_setup_i2c(MOTOR_I2C_BUS, 
                         MOTOR_I2C_SCL_PIN, 
                         MOTOR_I2C_SDA_PIN, 
                         MOTOR_I2C_FREQ_HZ, 
                         motor_setup_tag);
}

/**
 * @brief Sets up GPIO pins for encoders
 * 
 * This function initializes the GPIO pins used by encoders.
 * 
 * @return ESP_OK if setup succeeded, error code otherwise
 */
static esp_err_t priv_setup_gpio_encoders(void)
{
  log_info(motor_setup_tag, "GPIO Setup", "Setting up GPIO pins for encoders");
  
  /* Configure encoder pins as inputs with pull-up */
  uint64_t pin_mask = (1ULL << ENCODER_A_PIN) | (1ULL << ENCODER_B_PIN);
  
  return common_setup_gpio(pin_mask, 
                          GPIO_MODE_INPUT, 
                          GPIO_PULLUP_ENABLE, 
                          GPIO_PULLDOWN_DISABLE, 
                          GPIO_INTR_ANYEDGE, 
                          motor_setup_tag);
}

/**
 * @brief Cleans up I2C bus for motor controllers
 * 
 * This function releases the I2C bus used by PCA9685 PWM controllers.
 * 
 * @return ESP_OK if cleanup succeeded, error code otherwise
 */
static esp_err_t priv_cleanup_i2c_motors(void)
{
  log_info(motor_cleanup_tag, "I2C Cleanup", "Cleaning up I2C bus for motor controllers");
  
  return common_cleanup_i2c(MOTOR_I2C_BUS, 
                           MOTOR_I2C_SCL_PIN, 
                           MOTOR_I2C_SDA_PIN, 
                           motor_cleanup_tag);
}

/**
 * @brief Cleans up GPIO pins for encoders
 * 
 * This function releases the GPIO pins used by encoders.
 * 
 * @return ESP_OK if cleanup succeeded, error code otherwise
 */
static esp_err_t priv_cleanup_gpio_encoders(void)
{
  log_info(motor_cleanup_tag, "GPIO Cleanup", "Cleaning up GPIO pins for encoders");
  
  /* Release encoder pins */
  uint64_t pin_mask = (1ULL << ENCODER_A_PIN) | (1ULL << ENCODER_B_PIN);
  
  return common_cleanup_gpio(pin_mask, motor_cleanup_tag);
} 