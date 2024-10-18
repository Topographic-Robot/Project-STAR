#include "dht22_hal.h"
#include "common/gpio.h"
#include <esp_log.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* Constants *******************************************************************/

const char   *dht22_tag            = "DHT22";    /* Tag for logging */
const uint8_t dht22_data_io        = GPIO_NUM_4; /* GPIO pin for DHT22 data line */
const uint8_t dht22_polling_rate_s = 5;          /* Polling rate in seconds */
const uint8_t dht22_bit_count      = 40;         /* Total number of bits from DHT22 */

/**
 * Increasing this value:
 *   - Extends the time the GPIO pin is held low before sending the start signal.
 *   - A small increase might still work, but holding the line too long could 
 *     cause the DHT22 to ignore the start signal.
 * Decreasing this value:
 *   - Reduces the time the pin is held low. If decreased too much, 
 *   the DHT22 may not detect the start signal and fail to respond.
 * 
 * Typical value: 20ms is used to meet the DHT22 timing requirements.
 */
static const uint32_t dht22_start_delay_ms = 20; /* Start signal delay for DHT22 in ms */

/**
 * Increasing this value:
 *   - Provides more time for the DHT22 to respond, which could be useful if 
 *     the sensor has a delay.
 *   - However, increasing this too much might introduce unnecessary delays, 
 *     especially if the sensor is already responding quickly.
 * Decreasing this value:
 *   - Reduces the waiting time for a response from the DHT22. If reduced too 
 *     much, the program may timeout before the sensor can respond, leading to errors.
 * 
 * Typical value: 80us ensures a reasonable waiting time for the DHT22's initial response.
 */
static const uint32_t dht22_response_timeout_us = 80; /* Timeout for response in microseconds */

/**
 * Increasing this value:
 *   - Increases the time threshold for distinguishing between a '1' and a '0' 
 *     bit. A longer pulse will be interpreted as a '1'.
 *   - If increased too much, shorter pulses that represent '1' could be 
 *     interpreted as '0'.
 * Decreasing this value:
 *   - Shortens the time threshold for interpreting a '1' bit. If reduced too 
 *     much, longer pulses representing '0' might be interpreted as '1'.
 * 
 * Typical value: 40us ensures accurate differentiation between '1' and '0' 
 *                bits based on the DHT22's timing.
 */
static const uint32_t dht22_bit_threshold_us = 40; /* Threshold for distinguishing '1' and '0' bits */

/* Static (Private) Functions **************************************************/

/**
 * @brief Wait for a specific GPIO level within the given timeout.
 * 
 * @param level Desired GPIO level (0 or 1).
 * @param timeout Maximum time to wait for the level, in microseconds.
 * @return 0 if level reached successfully, 1 if timeout occurred.
 */
static int priv_dht22_wait_for_level(int level, uint32_t timeout) 
{
  int elapsed_time = 0;
  while (gpio_get_level(dht22_data_io) != level) {
    if (elapsed_time >= timeout) {
      return 1; /* Timeout occurred */
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); /* Delay 1 ms */
    elapsed_time++;
  }
  return 0; /* Success */
}

/**
 * @brief Sends the start signal to the DHT22 sensor.
 * 
 * Sets the GPIO direction to output, holds the line low for a set time, then releases it.
 */
static void priv_dht22_send_start_signal(void)
{
  gpio_set_direction(dht22_data_io, GPIO_MODE_OUTPUT);
  gpio_set_level(dht22_data_io, 0);                /* Pull line low */
  vTaskDelay(pdMS_TO_TICKS(dht22_start_delay_ms)); /* Wait for start signal duration */

  gpio_set_level(dht22_data_io, 1); /* Release line */
  vTaskDelay(pdMS_TO_TICKS(1000));  /* Short delay before switching to input (1ms) */
}

/**
 * @brief Waits for the DHT22 to respond after sending the start signal.
 * 
 * Checks for the response sequence from the sensor (low-high-low transitions).
 * 
 * @return ESP_OK if the sensor responds, ESP_FAIL on timeout.
 */
static esp_err_t priv_dht22_wait_for_response(void)
{
  if (priv_dht22_wait_for_level(0, dht22_response_timeout_us) == 1 ||
      priv_dht22_wait_for_level(1, dht22_response_timeout_us) == 1 ||
      priv_dht22_wait_for_level(0, dht22_response_timeout_us) == 1) {
    ESP_LOGE(dht22_tag, "Sensor not responding");
    return ESP_FAIL;
  }
  return ESP_OK;
}

/**
 * @brief Reads one bit of data from the DHT22 sensor.
 * 
 * Waits for the bit start and end signals, and interprets the bit based on timing.
 * 
 * @return 0 for a '0' bit, 1 for a '1' bit, ESP_FAIL on timeout.
 */
static int priv_dht22_read_bit(void) 
{
  if (priv_dht22_wait_for_level(1, 50) == 1) {
    ESP_LOGE(dht22_tag, "Timeout waiting for bit start");
    return ESP_FAIL;
  }
  int duration = priv_dht22_wait_for_level(0, 70);
  if (duration == 1) {
    ESP_LOGE(dht22_tag, "Timeout waiting for bit end");
    return ESP_FAIL;
  }

  return duration > dht22_bit_threshold_us;
}

/**
 * @brief Reads the full 40 bits (5 bytes) of data from the sensor.
 * 
 * Data is read bit by bit and stored in the provided data buffer.
 * 
 * @param[out] data_buffer Array to store the 40 bits (5 bytes) of data.
 * @return ESP_OK on success, ESP_FAIL on timeout or other errors.
 */
static esp_err_t priv_dht22_read_data_bits(uint8_t *data_buffer) 
{
  uint8_t byte_index = 0, bit_index = 7;
  for (int i = 0; i < dht22_bit_count; i++) {
    int bit = priv_dht22_read_bit();
    if (bit == ESP_FAIL) {
      return ESP_FAIL;
    }
    if (bit == 1) {
      data_buffer[byte_index] |= 1 << bit_index; /* Set bit to '1' */
    }

    /* Move to the next bit */
    if (bit_index == 0) {
      bit_index = 7;
      byte_index++;
    } else {
      bit_index--;
    }
  }

  return ESP_OK;
}

/**
 * @brief Verifies the checksum of the data received from the DHT22.
 * 
 * @param data_buffer Buffer containing the 40 bits (5 bytes) of data.
 * @return ESP_OK if checksum matches, ESP_FAIL otherwise.
 */
static esp_err_t priv_dht22_verify_checksum(uint8_t *data_buffer)
{
  uint8_t checksum = data_buffer[0] + data_buffer[1] + data_buffer[2] + data_buffer[3];
  if ((checksum & 0xFF) != data_buffer[4]) {
    ESP_LOGE(dht22_tag, "Checksum failed");
    return ESP_FAIL;
  }
  return ESP_OK;
}

/**
 * @brief Reads temperature and humidity from the DHT22 sensor.
 * 
 * Sends a start signal, waits for the sensor's response, and reads the 40 bits of data.
 * Temperature is converted to Fahrenheit and stored in the dht22_data_t structure.
 * 
 * @param[out] sensor_data Pointer to dht22_data_t where the sensor data will be stored.
 * @return ESP_OK on success, ESP_FAIL on timeout or checksum failure.
 */
static esp_err_t priv_dht22_read_data(dht22_data_t *sensor_data) 
{
  /* Buffer to store 5 bytes (40 bits) of data */
  uint8_t data_buffer[5] = {0};

  /* Send start signal and wait for response */
  priv_dht22_send_start_signal();
  gpio_set_direction(dht22_data_io, GPIO_MODE_INPUT);
  if (priv_dht22_wait_for_response() != ESP_OK) {
    return ESP_FAIL;
  }

  /* Read 40 bits of data */
  if (priv_dht22_read_data_bits(data_buffer) != ESP_OK) {
    return ESP_FAIL;
  }

  /* Verify checksum */
  if (priv_dht22_verify_checksum(data_buffer) != ESP_OK) {
    return ESP_FAIL;
  }

  /* Convert raw data to humidity and temperature */
  sensor_data->humidity      = ((data_buffer[0] << 8) + data_buffer[1]) * 0.1f;
  sensor_data->temperature_c = (((data_buffer[2] & 0x7F) << 8) + data_buffer[3]) * 0.1f;
  sensor_data->temperature_f = sensor_data->temperature_c * 9.0 / 5.0 + 32; /* Convert Celsius to Fahrenheit */

  /* Handle negative temperatures */
  if (data_buffer[2] & 0x80) {
    sensor_data->temperature_c = -(sensor_data->temperature_c);
    sensor_data->temperature_f = -(sensor_data->temperature_f);
  }

  return ESP_OK;
}

/* Public Functions ************************************************************/

esp_err_t dht22_init(dht22_data_t *sensor_data, bool first_time) 
{
  /* Initialize the struct, but not the semaphore until the state is 0x00.
   * This is to prevent memory allocation that might not be used. If we call
   * this function and it creates a new mutex very time then that is very bad.
   */
  if (first_time) {
    sensor_data->sensor_mutex = NULL; /* Set NULL, and change it later when its ready */
  }

  sensor_data->humidity      = -1.0;
  sensor_data->temperature_f = -1.0;
  sensor_data->temperature_c = -1.0;
  sensor_data->state         = k_dht22_uninitialized; /* start uninitialized */

  esp_err_t ret = priv_gpio_init(dht22_data_io);
  if (ret != ESP_OK) {
    ESP_LOGE(dht22_tag, "Failed to configure GPIO: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Pull the line high (logic 1) before starting communication.
   * Explanation:
   * - The DHT22 expects the line to be high during idle states.
   * - After pulling the line low to send a start signal, the line must be released 
   *   (set high) so the DHT22 can pull the line low in response.
   * - Keeping the line high ensures proper communication between the sensor and MCU. */
  gpio_set_level(dht22_data_io, 1);

  if (sensor_data->sensor_mutex == NULL) {
    sensor_data->sensor_mutex = xSemaphoreCreateMutex();
    if (sensor_data->sensor_mutex == NULL) {
      ESP_LOGE(dht22_tag, "Failed to create sensor mutex");

      sensor_data->state = k_dht22_error;
      return ESP_ERR_NO_MEM;
    }
  }

  sensor_data->state = k_dht22_ready;
  return ESP_OK;
}

void dht22_read(dht22_data_t *sensor_data) 
{
  /* Check if the sensor data is NULL */
  if (sensor_data == NULL) {
    ESP_LOGE(dht22_tag, "Sensor data pointer is NULL");
    return;
  }

  /* Try to take the mutex to ensure exclusive access to the sensor */
  if (xSemaphoreTake(sensor_data->sensor_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(dht22_tag, "Failed to acquire mutex to access sensor");
    return;
  }

  /* Attempt to read data from the DHT22 sensor */
  esp_err_t ret = priv_dht22_read_data(sensor_data);
  if (ret != ESP_OK) {
    sensor_data->humidity      = -1.0;
    sensor_data->temperature_f = -1.0;
    sensor_data->temperature_c = -1.0;
    sensor_data->state         = k_dht22_error;

    ESP_LOGE(dht22_tag, "Failed to read data from DHT22");
    xSemaphoreGive(sensor_data->sensor_mutex); /* Ensure the semaphore is released */
    return;
  }

  ESP_LOGI(dht22_tag, "Temperature: %.1f F (%1.f C), Humidity: %.1f %%", sensor_data->temperature_f, sensor_data->temperature_c, sensor_data->humidity);

  /* Release the mutex after accessing the sensor */
  xSemaphoreGive(sensor_data->sensor_mutex);
}

void dht22_tasks(void *sensor_data) 
{
  dht22_data_t *dht22_data = (dht22_data_t *)sensor_data;
  dht22_read(dht22_data);

  vTaskDelay(pdMS_TO_TICKS(dht22_polling_rate_s * 1000)); /* convert s to ms */
}
