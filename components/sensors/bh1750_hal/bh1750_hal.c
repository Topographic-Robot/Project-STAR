#include "bh1750_hal.h"
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
 * @note this functions don't check the semaphore, it is an internal function.
 *
 * @param[in] data The byte of data to be written to the BH1750 sensor.
 * @param[in] i2c_bus The I2C bus number to communicate over.
 *
 * @return 
 *   - ESP_OK on success.
 *   - Error code on failure.
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
 * @note this functions don't check the semaphore, it is an internal function.
 *
 * @param[out] data Pointer to the buffer where read data will be stored.
 * @param[in] len The number of bytes to read.
 * @param[in] i2c_bus The I2C bus number to communicate over.
 *
 * @return 
 *   - ESP_OK on success.
 *   - Error code on failure.
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

esp_err_t bh1750_init(bh1750_data_t* bh1750_data)
{
  /* Initialize the struct, but not the semaphore until the state is 0x00.
   * This is to prevent memory allocation that might not be used. If we call
   * this function and it creates a new mutex very time then that is very bad.
   */
  bh1750_data->sensor_mutex = NULL; /* Set NULL, and change it later when its ready */
  bh1750_data->lux          = -1.0; /* Start with an invalid value since 
                                     * it hasnet been read yet */

  bh1750_data->state = k_bh1750_uninitialized; /* Start in uninitialized */

  /* Initialize the I2C bus with specified SCL, SDA pins, frequency, and bus number */
  esp_err_t err = priv_i2c_init(
      bh1750_scl_io, bh1750_sda_io, bh1750_freq_hz, bh1750_data->i2c_bus, 
      bh1750_tag);

  if (err != ESP_OK) {
    /* Log an error if the I2C driver installation fails */
    ESP_LOGE(bh1750_tag, "I2C driver install failed: %s", esp_err_to_name(err));
    return err; /* Return the error code if initialization fails */
  }
  
  /* Power on the BH1750 sensor */
  err = priv_bh1750_write_byte(bh1750_power_on_cmd, bh1750_data->i2c_bus);
  if (err != ESP_OK) {
    /* Log an error if powering on the BH1750 fails */
    ESP_LOGE(bh1750_tag, "BH1750 power on failed");

    /* Update state */
    bh1750_data->state = k_bh1750_power_on_error;

    return err; /* Return the error code if power on fails */
  }
  
  /* Delay for 10ms to allow the sensor to power on */
  vTaskDelay(10 / portTICK_PERIOD_MS);
  
  /* Reset the BH1750 sensor */
  err = priv_bh1750_write_byte(bh1750_reset_cmd, bh1750_data->i2c_bus);
  if (err != ESP_OK) {
    /* Log an error if resetting the BH1750 fails */
    ESP_LOGE(bh1750_tag, "BH1750 reset failed");

    /* Update the state */
    bh1750_data->state = k_bh1750_reset_error;

    return err; /* Return the error code if reset fails */
  }
  
  /* Delay for 10ms to allow the reset to take effect */
  vTaskDelay(10 / portTICK_PERIOD_MS);
  
  /* Set the BH1750 sensor to continuous high-resolution mode */
  err = priv_bh1750_write_byte(bh1750_cont_low_res_mode, bh1750_data->i2c_bus);
  if (err != ESP_OK) {
    /* Log an error if setting the resolution mode fails */
    ESP_LOGE(bh1750_tag, "BH1750 setting resolution mode failed");

    /* Update the state */
    bh1750_data->state = k_bh1759_cont_low_res_error;

    return err; /* Return the error code if setting the mode fails */
  }
  
  /* Delay for 10ms to allow the resolution mode to be set */
  vTaskDelay(10 / portTICK_PERIOD_MS);

  /* At this point no errors happened and the sensor is Initialized */
  /* Verify that this sensor didnt already have its mutex set */
  if (bh1750_data->sensor_mutex == NULL) {
    /* Now we set the mutex value only once */
    bh1750_data->sensor_mutex = xSemaphoreCreateMutex();
    if (bh1750_data->sensor_mutex == NULL) {
      ESP_LOGE(bh1750_tag, "ESP32 ran out of memory");
      return ESP_ERR_NO_MEM;
    }
  }
  
  return ESP_OK;
}

void bh1750_read(bh1750_data_t *bh1750_data) 
{
  /* Try to take the semaphore before accessing shared data */
  if (xSemaphoreTake(bh1750_data->sensor_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGW(bh1750_tag, "Failed to take sensor mutex");
    return;
  }

  /* Array to store the two bytes of data read from the BH1750 sensor */
  uint8_t data[2];

  /* Read 2 bytes of data from the BH1750 sensor over I2C */
  esp_err_t err = priv_bh1750_read(data, 2, bh1750_data->i2c_bus);
  if (err != ESP_OK) {
    bh1750_data->lux   = -1.0; /* Set lux to -1.0 to indicate an error */
    bh1750_data->state = k_bh1750_error;
    ESP_LOGE(bh1750_tag, "Failed to read data from BH1750");

    xSemaphoreGive(bh1750_data->sensor_mutex); /* Ensure the semaphore is released */
    return;
  }

  /* Combine the two bytes into a 16-bit raw light intensity value */
  uint16_t raw_light_intensity = (data[0] << 8) | data[1];

  /* Convert the raw light intensity to lux as per the BH1750 datasheet */
  bh1750_data->lux = raw_light_intensity / 1.2; /* Lux = raw value / 1.2 */
  ESP_LOGI(bh1750_tag, "The measured light intensity was %f lux", bh1750_data->lux);

  /* Give the semaphore back after accessing the shared data */
  xSemaphoreGive(bh1750_data->sensor_mutex);
}

void bh1750_reset_on_error(bh1750_data_t *bh1750_data) 
{
  /* Check if the state indicates any error using a bitwise AND with k_bh1750_error */
  if (bh1750_data->state & k_bh1750_error) {
    ESP_LOGI(bh1750_tag, "Error detected. Attempting to reset the BH1750 sensor.");

    /* Attempt to initialize/reset the sensor */
    if (bh1750_init(bh1750_data) == ESP_OK) {
      /* If successful, set the state to ready */
      bh1750_data->state = k_bh1750_ready;
      ESP_LOGI(bh1750_tag, "BH1750 sensor reset successfully. State is now ready.");
    } else {
      /* If reset fails, set the state to reset error */
      bh1750_data->state = k_bh1750_reset_error;
      ESP_LOGE(bh1750_tag, "Failed to reset the BH1750 sensor. State set to reset error.");
    }
  }
}
