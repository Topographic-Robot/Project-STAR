#include "bh1750.h"
#include "common/i2c.h"
#include <driver/i2c.h>
#include <esp_log.h>


/* Constants ******************************************************************/

const uint8_t bh1750_power_on_cmd      = 0x01;     /* Power on command */
const uint8_t bh1750_i2c_address       = 0x23;     /* I2C address for BH1750 */
const uint8_t bh1750_cont_low_res_mode = 0x13;     /* Continuously measure low resolution (4 lux) */
const uint8_t bh1750_reset_cmd         = 0x07;     /* Reset Command */
const char   *bh1750_tag               = "BH1750"; /* Tag for logs */

/* Private (Static) Functions *************************************************/

/**
 * @brief Write a byte to the BH1750 sensor over I2C.
 *
 * This function sends a single byte of data to the BH1750 sensor using the
 * specified I2C bus. The command link is created, and the data is written
 * using the I2C protocol.
 *
 * @param[in] data The byte of data to be written to the BH1750 sensor.
 * @param[in] i2c_bus The I2C bus number to communicate over.
 *
 * @return 
 *      - ESP_OK on success.
 *      - Error code on failure.
 */
static esp_err_t priv_bh1750_write_byte(uint8_t data, uint8_t i2c_bus)
{
  /* Create an I2C command link handle */
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  /* Start I2C communication */
  i2c_master_start(cmd);

  /* Send the BH1750 I2C address with the write flag */
  i2c_master_write_byte(cmd, bh1750_i2c_address << 1 | I2C_MASTER_WRITE, true);

  /* Write the data byte to the sensor */
  i2c_master_write_byte(cmd, data, true);

  /* Stop I2C communication */
  i2c_master_stop(cmd);

  /* Execute the I2C command */
  esp_err_t err = i2c_master_cmd_begin(i2c_bus, cmd, 1000 / portTICK_PERIOD_MS);

  /* Delete the command link after execution */
  i2c_cmd_link_delete(cmd);

  /* Check for errors in the I2C command */
  if (err != ESP_OK) {
    ESP_LOGE(bh1750_tag, "I2C write failed: %s", esp_err_to_name(err));
  }

  return err; /* Return the error status or ESP_OK */
}

/**
 * @brief Read multiple bytes from the BH1750 sensor over I2C.
 *
 * This function reads a specified number of bytes from the BH1750 sensor using
 * the I2C bus. It handles reading a single byte or multiple bytes with ACK/NACK 
 * handling for I2C communication.
 *
 * @param[out] data Pointer to the buffer where read data will be stored.
 * @param[in] len The number of bytes to read.
 * @param[in] i2c_bus The I2C bus number to communicate over.
 *
 * @return 
 *      - ESP_OK on success.
 *      - Error code on failure.
 */
static esp_err_t priv_bh1750_read(uint8_t *data, size_t len, uint8_t i2c_bus)
{
  /* Create an I2C command link handle */
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  /* Start I2C communication */
  i2c_master_start(cmd);

  /* Send the BH1750 I2C address with the read flag */
  i2c_master_write_byte(cmd, bh1750_i2c_address << 1 | I2C_MASTER_READ, true);

  /* Read multiple bytes if length is greater than 1 */
  if (len > 1) {
    /* Read len-1 bytes with an ACK after each byte */
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  }

  /* Read the last byte with a NACK to signal the end of the transmission */
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);

  /* Stop I2C communication */
  i2c_master_stop(cmd);

  /* Execute the I2C command */
  esp_err_t err = i2c_master_cmd_begin(i2c_bus, cmd, 1000 / portTICK_PERIOD_MS);

  /* Delete the command link after execution */
  i2c_cmd_link_delete(cmd);

  /* Check for errors in the I2C command */
  if (err != ESP_OK) {
    ESP_LOGE(bh1750_tag, "I2C read failed: %s", esp_err_to_name(err));
  }

  return err; /* Return the error status or ESP_OK */
}

/* Public Functions ***********************************************************/

esp_err_t bh1750_init(uint8_t scl_io, uint8_t sda_io, uint32_t freq_hz,
                      uint8_t i2c_bus)
{
  /* Initialize the I2C bus with specified SCL, SDA pins, frequency, and bus number */
  esp_err_t err = priv_i2c_init(scl_io, sda_io, freq_hz, i2c_bus, bh1750_tag);
  if (err != ESP_OK) {
    /* Log an error if the I2C driver installation fails */
    ESP_LOGE(bh1750_tag, "I2C driver install failed: %s", esp_err_to_name(err));
    return err; /* Return the error code if initialization fails */
  }
  
  /* Power on the BH1750 sensor */
  err = priv_bh1750_write_byte(bh1750_power_on_cmd, i2c_bus);
  if (err != ESP_OK) {
    /* Log an error if powering on the BH1750 fails */
    ESP_LOGE(bh1750_tag, "BH1750 power on failed");
    return err; /* Return the error code if power on fails */
  }
  
  /* Delay for 10ms to allow the sensor to power on */
  vTaskDelay(10 / portTICK_PERIOD_MS);
  
  /* Reset the BH1750 sensor */
  err = priv_bh1750_write_byte(bh1750_reset_cmd, i2c_bus);
  if (err != ESP_OK) {
    /* Log an error if resetting the BH1750 fails */
    ESP_LOGE(bh1750_tag, "BH1750 reset failed");
    return err; /* Return the error code if reset fails */
  }
  
  /* Delay for 10ms to allow the reset to take effect */
  vTaskDelay(10 / portTICK_PERIOD_MS);
  
  /* Set the BH1750 sensor to continuous high-resolution mode */
  err = priv_bh1750_write_byte(bh1750_cont_low_res_mode, i2c_bus);
  if (err != ESP_OK) {
    /* Log an error if setting the resolution mode fails */
    ESP_LOGE(bh1750_tag, "BH1750 setting resolution mode failed");
    return err; /* Return the error code if setting the mode fails */
  }
  
  /* Delay for 10ms to allow the resolution mode to be set */
  vTaskDelay(10 / portTICK_PERIOD_MS);
  
  return err; /* Return the final error status or ESP_OK if successful */
}

float bh1750_read_lux(uint8_t i2c_bus)
{
  /* Array to store the two bytes of data read from the BH1750 sensor */
  uint8_t data[2];
  
  /* Read 2 bytes of data from the BH1750 sensor over I2C */
  esp_err_t err = priv_bh1750_read(data, 2, i2c_bus);
  if (err == ESP_OK) {
    /* Combine the two bytes into a 16-bit raw light intensity value */
    uint16_t raw_light_intensity = (data[0] << 8) | data[1];
  
    /* Convert the raw light intensity to lux as per the BH1750 datasheet */
    return raw_light_intensity / 1.2; /* Lux = raw value / 1.2 */
  } 

  /* Log an error if reading data from the BH1750 sensor fails */
  ESP_LOGE(bh1750_tag, "Failed to read data from BH1750");
  return -1.0; /* Return -1.0 to indicate an error */
}

