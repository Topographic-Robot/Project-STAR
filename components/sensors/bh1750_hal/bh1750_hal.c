#include "bh1750_hal.h"
#include "common/i2c.h"
#include <esp_log.h>

/* Constants ******************************************************************/

const uint8_t  bh1750_i2c_address        = 0x23;
const char    *bh1750_tag                = "BH1750";
const uint8_t  bh1750_scl_io             = GPIO_NUM_22;
const uint8_t  bh1750_sda_io             = GPIO_NUM_21;
const uint32_t bh1750_freq_hz            = 100000;
const uint32_t bh1750_polling_rate_ticks = pdMS_TO_TICKS(5 * 1000);

/* Public Functions ***********************************************************/

esp_err_t bh1750_init(bh1750_data_t *sensor_data, bool first_time)
{
  /* Initialize the struct, but not the semaphore until the state is 0x00.
   * This is to prevent memory allocation that might not be used. If we call
   * this function and it creates a new mutex very time then that is very bad.
   */
  if (first_time) {
    sensor_data->sensor_mutex = NULL; /* Set NULL, and change it later when its ready */
  }
  sensor_data->lux = -1.0; /* Start with an invalid value since 
                            * it hasnet been read yet */

  sensor_data->state = k_bh1750_uninitialized; /* Start in uninitialized */

  /* Initialize the I2C bus with specified SCL, SDA pins, frequency, and bus number */
  esp_err_t ret = priv_i2c_init(
      bh1750_scl_io, bh1750_sda_io, bh1750_freq_hz, sensor_data->i2c_bus, 
      bh1750_tag);

  if (ret != ESP_OK) {
    /* Log an error if the I2C driver installation fails */
    ESP_LOGE(bh1750_tag, "I2C driver install failed: %s", esp_err_to_name(ret));
    return ret; /* Return the error code if initialization fails */
  }
  
  /* Power on the BH1750 sensor */
  ret = priv_i2c_write_byte(k_bh1750_power_on_cmd, sensor_data->i2c_bus, bh1750_tag);
  if (ret != ESP_OK) {
    /* Log an error if powering on the BH1750 fails */
    ESP_LOGE(bh1750_tag, "BH1750 power on failed");

    /* Update state */
    sensor_data->state = k_bh1750_power_on_error;

    return ret; /* Return the error code if power on fails */
  }
  
  /* Delay for 10ms to allow the sensor to power on */
  vTaskDelay(10 / portTICK_PERIOD_MS);
  
  /* Reset the BH1750 sensor */
  ret = priv_i2c_write_byte(k_bh1750_reset_cmd, sensor_data->i2c_bus, bh1750_tag);
  if (ret != ESP_OK) {
    /* Log an error if resetting the BH1750 fails */
    ESP_LOGE(bh1750_tag, "BH1750 reset failed");

    /* Try a power cycle */
    ret = priv_i2c_write_byte(k_bh1750_power_down_cmd, 
                              sensor_data->i2c_bus, bh1750_tag);
    if (ret != ESP_OK) {
      /* Set and return the error */
      sensor_data->state = k_bh1750_power_cycle_error;
      return ret;
    } else {
      /* Call this function again to turn it back on */
      bh1750_init(sensor_data, false);
    }

    /* Update the state */
    sensor_data->state = k_bh1750_reset_error;

    return ret; /* Return the error code if reset fails */
  }
  
  /* Delay for 10ms to allow the reset to take effect */
  vTaskDelay(10 / portTICK_PERIOD_MS);
  
  /* Set the BH1750 sensor to continuous high-resolution mode */
  ret = priv_i2c_write_byte(k_bh1750_cont_low_res_mode_cmd, 
                            sensor_data->i2c_bus, bh1750_tag);
  if (ret != ESP_OK) {
    /* Log an error if setting the resolution mode fails */
    ESP_LOGE(bh1750_tag, "BH1750 setting resolution mode failed");

    /* Update the state */
    sensor_data->state = k_bh1750_cont_low_res_error;

    return ret; /* Return the error code if setting the mode fails */
  }
  
  /* Delay for 10ms to allow the resolution mode to be set */
  vTaskDelay(10 / portTICK_PERIOD_MS);

  /* At this point no errors happened and the sensor is Initialized */
  /* Verify that this sensor didnt already have its mutex set */
  if (sensor_data->sensor_mutex == NULL) {
    /* Now we set the mutex value only once */
    sensor_data->sensor_mutex = xSemaphoreCreateMutex();
    if (sensor_data->sensor_mutex == NULL) {
      ESP_LOGE(bh1750_tag, "ESP32 ran out of memory");

      sensor_data->state = k_bh1750_error;
      return ESP_ERR_NO_MEM;
    }
  }

  sensor_data->state = k_bh1750_ready; /* it is initialized */
  return ESP_OK;
}

void bh1750_read(bh1750_data_t *sensor_data) 
{
  /* Check if the sensor data is NULL */
  if (sensor_data == NULL) {
    ESP_LOGE(bh1750_tag, "Sensor data pointer is NULL");
    return;
  }


  /* Try to take the semaphore before accessing shared data */
  if (xSemaphoreTake(sensor_data->sensor_mutex, 2 * bh1750_polling_rate_ticks) != pdTRUE) {
    ESP_LOGW(bh1750_tag, "Failed to take sensor mutex");
    return;
  }

  /* Array to store the two bytes of data read from the BH1750 sensor */
  uint8_t data[2];

  /* Read 2 bytes of data from the BH1750 sensor over I2C */
  esp_err_t ret = priv_i2c_read_bytes(data, 2, sensor_data->i2c_bus, bh1750_tag);
  if (ret != ESP_OK) {
    sensor_data->lux   = -1.0; /* Set lux to -1.0 to indicate an error */
    sensor_data->state = k_bh1750_error;
    ESP_LOGE(bh1750_tag, "Failed to read data from BH1750");

    xSemaphoreGive(sensor_data->sensor_mutex); /* Ensure the semaphore is released */
    return;
  }

  /* Combine the two bytes into a 16-bit raw light intensity value */
  uint16_t raw_light_intensity = (data[0] << 8) | data[1];

  /* Convert the raw light intensity to lux as per the BH1750 datasheet */
  sensor_data->lux = raw_light_intensity / 1.2; /* Lux = raw value / 1.2 */
  ESP_LOGI(bh1750_tag, "The measured light intensity was %f lux", sensor_data->lux);

  /* Give the semaphore back after accessing the shared data */
  xSemaphoreGive(sensor_data->sensor_mutex);
}

void bh1750_reset_on_error(bh1750_data_t *sensor_data) 
{
  /* Check if the state indicates any error using a bitwise AND with k_bh1750_error */
  if (sensor_data->state & k_bh1750_error) {
    ESP_LOGI(bh1750_tag, "Error detected. Attempting to reset the BH1750 sensor.");

    /* Attempt to initialize/reset the sensor */
    if (bh1750_init(sensor_data, false) == ESP_OK) {
      /* If successful, set the state to ready */
      sensor_data->state = k_bh1750_ready;
      ESP_LOGI(bh1750_tag, "BH1750 sensor reset successfully. State is now ready.");
    } else {
      /* If reset fails, set the state to reset error */
      sensor_data->state = k_bh1750_reset_error;
      ESP_LOGE(bh1750_tag, "Failed to reset the BH1750 sensor. State set to reset error.");
    }
  }
}

void bh1750_tasks(void *sensor_data)
{
  bh1750_data_t *bh1750_data = (bh1750_data_t *)sensor_data;
  bh1750_read(bh1750_data);
  bh1750_reset_on_error(bh1750_data);

  vTaskDelay(bh1750_polling_rate_ticks);
}
