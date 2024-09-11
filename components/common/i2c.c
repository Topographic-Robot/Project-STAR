#include <driver/i2c.h>
#include <esp_log.h>

/* Private Functions **********************************************************/

/**
 * @brief Initialize the I2C interface with the specified parameters.
 *
 * This function configures the I2C bus with the provided SCL and SDA pins, 
 * pull-up configuration, and frequency. It sets the I2C mode to master 
 * and initializes the driver without requiring a buffer for the master mode.
 *
 * @param scl_io Pin number for the I2C clock line (SCL).
 * @param sda_io Pin number for the I2C data line (SDA).
 * @param freq_hz Clock frequency in Hertz for the I2C bus.
 * @param i2c_bus I2C bus number to use for communication.
 * @param tag The tag for logging errors.
 *
 * @return 
 *      - ESP_OK on successful initialization.
 *      - An error code from the esp_err_t enumeration on failure.
 *
 * @note The function enables internal pull-ups for both SDA and SCL pins.
 *       Make sure the I2C bus is free before calling this function.
 */
esp_err_t priv_i2c_init(uint8_t scl_io, uint8_t sda_io, uint32_t freq_hz, 
                        uint8_t i2c_bus, const char* tag)
{
  /* The I2C configuration structure */
  i2c_config_t conf = {
    .mode             = I2C_MODE_MASTER,    /* Set the I2C mode to master */
    .sda_io_num       = sda_io,             /* Set the GPIO number for SDA (data line) */
    .sda_pullup_en    = GPIO_PULLUP_ENABLE, /* Enable internal pull-up for SDA line */
    .scl_io_num       = scl_io,             /* Set the GPIO number for SCL (clock line) */
    .scl_pullup_en    = GPIO_PULLUP_ENABLE, /* Enable internal pull-up for SCL line */
    .master.clk_speed = freq_hz,            /* Set the I2C master clock frequency */
  };

  /* Configure the I2C bus with the settings specified in 'conf' */
  esp_err_t err = i2c_param_config(i2c_bus, &conf);
  if (err != ESP_OK) {
    ESP_LOGE(tag, "I2C param config failed: %s", esp_err_to_name(err));
    return err;  /* Return the error code if configuration fails */
  }

  /* Install the I2C driver for the master mode; no RX/TX buffers are required */
  return i2c_driver_install(i2c_bus, conf.mode, 0, 0, 0);
}

