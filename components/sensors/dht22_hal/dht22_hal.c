/* components/sensors/dht22_hal/dht22_hal.c */

#include "dht22_hal.h"
#include <stdio.h>
#include <string.h>
#include "webserver_tasks.h"
#include "cJSON.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Constants *******************************************************************/

const char    *dht22_tag                    = "DHT22";
const uint8_t  dht22_data_io                = GPIO_NUM_4;
const uint32_t dht22_polling_rate_ticks     = pdMS_TO_TICKS(5 * 1000);
const uint8_t  dht22_bit_count              = 40;
const uint8_t  dht22_max_retries            = 4;
const uint32_t dht22_initial_retry_interval = pdMS_TO_TICKS(15 * 1000);
const uint32_t dht22_max_backoff_interval   = pdMS_TO_TICKS(480 * 1000);

/* Timing Constants for DHT22 communication */
static const uint32_t dht22_start_delay_ms      = 20; /**< Start signal delay for DHT22 in milliseconds */
static const uint32_t dht22_response_timeout_us = 80; /**< Timeout for response from DHT22 in microseconds */
static const uint32_t dht22_bit_threshold_us    = 40; /**< Threshold for distinguishing between '1' and '0' bits */

/* Static (Private) Functions **************************************************/

/**
 * @brief Initializes the GPIO pin connected to the DHT22 data line.
 *
 * This function configures the GPIO pin connected to the DHT22 sensor as an output with the necessary pull-up resistor enabled. 
 * It sets the pin to an initial high state to ensure the sensor is ready for communication.
 * The GPIO configuration includes setting the mode to output, enabling pull-up, and disabling pull-down and interrupts.
 * 
 * **Logic and Flow:**
 * - Set the GPIO pin as output with pull-up enabled.
 * - The pull-up ensures that the pin is kept high when not actively pulled low by the sensor or microcontroller.
 * - This function is intended to be called during sensor initialization.
 *
 * @param[in] data_io GPIO pin number connected to the DHT22 data line.
 * 
 * @return 
 * - `ESP_OK` if the GPIO was configured successfully.
 * - An `esp_err_t` code if an error occurred during configuration.
 */
static esp_err_t priv_gpio_init(uint8_t data_io)
{
  gpio_config_t io_conf;
  io_conf.pin_bit_mask = (1ULL << data_io);
  io_conf.mode         = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type    = GPIO_INTR_DISABLE;

  return gpio_config(&io_conf);
}

/**
 * @brief Wait for a specific GPIO level within the given timeout and measure the duration.
 *
 * This function monitors the GPIO line connected to the DHT22 sensor to determine when it reaches 
 * the desired level (`0` or `1`). The function measures the time taken to reach this level and 
 * stores the duration in the provided variable `duration_us`. If the desired level is not reached 
 * within the specified `timeout_us`, the function returns false, indicating a timeout.
 *
 * **Logic and Flow:**
 * - Start by recording the current time using `esp_timer_get_time()`.
 * - Continuously check the level of the specified GPIO pin until it matches the desired level.
 * - If the GPIO level is not reached within the given `timeout_us`, return `false`.
 * - Otherwise, store the duration taken to reach the level in `duration_us` and return `true`.
 *
 * @param[in] level Desired GPIO level (0 or 1).
 * @param[in] timeout_us Maximum time to wait for the level, in microseconds.
 * @param[out] duration_us Pointer to store the time taken to reach the level, in microseconds.
 *
 * @return
 * - `true` if the desired level was reached successfully within the given timeout.
 * - `false` if the timeout occurred without reaching the desired level.
 */
static bool priv_dht22_wait_for_level_with_duration(int level, uint32_t timeout_us, 
                                                    uint32_t *duration_us)
{
  uint64_t start_time = esp_timer_get_time();
  while (gpio_get_level(dht22_data_io) != level) {
    if ((esp_timer_get_time() - start_time) >= timeout_us) {
      return false; /* Timeout occurred */
    }
  }
  *duration_us = (uint32_t)(esp_timer_get_time() - start_time);
  return true; /* Success */
}

/**
 * @brief Sends the start signal to the DHT22 sensor.
 *
 * The start signal is sent by pulling the data line low for a specified duration (`dht22_start_delay_ms`), 
 * indicating to the DHT22 that the ESP32 is ready to communicate. After holding the line low, the GPIO line 
 * is released by setting it high, and a short delay is added before switching the GPIO mode back to input. 
 * This transition from low to high initiates the communication between the ESP32 and the sensor.
 *
 * **Logic and Flow:**
 * - Set the GPIO direction to output.
 * - Pull the data line low for the start signal duration.
 * - Set the data line high and add a short delay.
 * - The sensor will then start sending a response indicating it is ready for data transmission.
 *
 * @note The GPIO pin must be configured properly before calling this function.
 */
static void priv_dht22_send_start_signal(void)
{
  gpio_set_direction(dht22_data_io, GPIO_MODE_OUTPUT);
  gpio_set_level(dht22_data_io, 0); /* Pull line low */
  esp_rom_delay_us(dht22_start_delay_ms * 1000); /* Wait for start signal duration */

  gpio_set_level(dht22_data_io, 1); /* Release line */
  esp_rom_delay_us(30); /* Wait for 30 microseconds before switching to input */
}

/**
 * @brief Waits for the DHT22 to respond after sending the start signal.
 *
 * After the start signal is sent, the DHT22 sensor responds with a specific sequence of GPIO transitions: 
 * it pulls the line low for approximately 80 microseconds, then high for another 80 microseconds, and finally 
 * pulls it low again. This function waits for these transitions to occur within the specified time limit.
 *
 * **Logic and Flow:**
 * - Wait for the line to go low (indicating the start of the sensor response).
 * - Wait for the line to go high (indicating the continuation of the response).
 * - Wait for the line to go low again, signaling readiness to transmit data.
 * - If the expected transitions do not occur within the allotted time, return `ESP_FAIL`.
 *
 * @return 
 * - `ESP_OK` if the sensor responds correctly.
 * - `ESP_FAIL` if the sensor fails to respond within the specified timeout.
 */
static esp_err_t priv_dht22_wait_for_response(void)
{
  uint32_t duration = 0;

  /* DHT22 pulls the line low for 80us, then high for 80us, as a response signal */
  if (!priv_dht22_wait_for_level_with_duration(0, dht22_response_timeout_us, &duration) ||
      !priv_dht22_wait_for_level_with_duration(1, dht22_response_timeout_us, &duration) ||
      !priv_dht22_wait_for_level_with_duration(0, dht22_response_timeout_us, &duration)) {
    ESP_LOGE(dht22_tag, "Sensor not responding");
    return ESP_FAIL;
  }
  return ESP_OK;
}

/**
 * @brief Reads one bit of data from the DHT22 sensor.
 *
 * This function reads a single bit from the DHT22 sensor by measuring the duration of a high-level pulse. 
 * The DHT22 represents a `0` bit with a short pulse, and a `1` bit with a longer pulse. The function waits 
 * for the bit transmission to start, measures the duration of the high-level signal, and determines whether 
 * it represents a `0` or `1` based on a predefined threshold.
 *
 * **Logic and Flow:**
 * - Wait for the line to go high, signaling the start of a bit transmission.
 * - Measure the time taken for the line to go low, indicating the end of the bit.
 * - If the duration exceeds the threshold (`dht22_bit_threshold_us`), the bit is interpreted as `1`, otherwise as `0`.
 *
 * @return
 * - `0` for a `0` bit.
 * - `1` for a `1` bit.
 * - `ESP_FAIL` if the bit could not be read within the expected timing constraints.
 */
static int priv_dht22_read_bit(void)
{
  uint32_t duration = 0;

  /* Wait for the line to go high (start of bit transmission) */
  if (!priv_dht22_wait_for_level_with_duration(1, 50, &duration)) {
    ESP_LOGE(dht22_tag, "Timeout waiting for bit start");
    return ESP_FAIL;
  }

  /* Wait for the line to go low again, measuring the duration of the high level */
  if (!priv_dht22_wait_for_level_with_duration(0, 70, &duration)) {
    ESP_LOGE(dht22_tag, "Timeout waiting for bit end");
    return ESP_FAIL;
  }

  /* Determine if the bit is '0' or '1' based on the duration */
  if (duration > dht22_bit_threshold_us) {
    return 1; /* Bit is '1' */
  } else {
    return 0; /* Bit is '0' */
  }
}

/**
 * @brief Reads the full 40 bits (5 bytes) of data from the sensor.
 *
 * The DHT22 sensor transmits 40 bits of data, which include humidity, temperature, and a checksum. 
 * This function reads each bit from the sensor and assembles them into 5 bytes stored in the provided 
 * data buffer. If any bit cannot be read within the expected timing constraints, the function will 
 * return an error.
 *
 * **Logic and Flow:**
 * - Initialize the data buffer to zero.
 * - Read 40 bits from the sensor, assembling each bit into the appropriate byte.
 * - Set the corresponding bit in the data buffer if the bit value is `1`.
 *
 * @param[out] data_buffer Array to store the 40 bits (5 bytes) of data.
 *
 * @return
 * - `ESP_OK` on successful data read.
 * - `ESP_FAIL` on timeout or other errors while reading bits.
 */
static esp_err_t priv_dht22_read_data_bits(uint8_t *data_buffer)
{
  uint8_t byte_index = 0, bit_index = 7;
  memset(data_buffer, 0, 5);
  for (int i = 0; i < dht22_bit_count; i++) {
    int bit = priv_dht22_read_bit();
    if (bit == ESP_FAIL) {
      return ESP_FAIL;
    }
    if (bit == 1) {
      data_buffer[byte_index] |= (1 << bit_index); /* Set bit to '1' */
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
 * The DHT22 sensor sends 5 bytes of data, with the first four bytes representing humidity and temperature, 
 * and the fifth byte being a checksum. This function calculates the checksum from the first four bytes 
 * and compares it to the received checksum byte to ensure data integrity. If the checksum matches, the 
 * function returns `ESP_OK`; otherwise, it returns an error.
 *
 * **Logic and Flow:**
 * - Calculate the checksum by adding the first four bytes of data.
 * - Compare the calculated checksum with the fifth byte in the data buffer.
 *
 * @param[in] data_buffer Buffer containing the 40 bits (5 bytes) of data received from the sensor.
 *
 * @return
 * - `ESP_OK` if the calculated checksum matches the received checksum.
 * - `ESP_FAIL` if the checksum verification fails.
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

/* Public Functions ************************************************************/

char *dht22_data_to_json(const dht22_data_t *data) 
{
  cJSON *json = cJSON_CreateObject();
  cJSON_AddStringToObject(json, "sensor_type", "temperature_humidity");
  cJSON_AddNumberToObject(json, "temperature_c", data->temperature_c);
  cJSON_AddNumberToObject(json, "humidity", data->humidity);
  char *json_string = cJSON_PrintUnformatted(json);
  cJSON_Delete(json);
  return json_string;
}

esp_err_t dht22_init(void *sensor_data)
{
  dht22_data_t *dht22_data = (dht22_data_t *)sensor_data;
  ESP_LOGI(dht22_tag, "Starting Configuration");

  dht22_data->humidity           = -1.0;
  dht22_data->temperature_f      = -1.0;
  dht22_data->temperature_c      = -1.0;
  dht22_data->state              = k_dht22_uninitialized; /* Start uninitialized */
  dht22_data->retry_count        = 0;
  dht22_data->retry_interval     = dht22_initial_retry_interval;
  dht22_data->last_attempt_ticks = 0;

  esp_err_t ret = priv_gpio_init(dht22_data_io);
  if (ret != ESP_OK) {
    ESP_LOGE(dht22_tag, "Failed to configure GPIO: %s", esp_err_to_name(ret));
    return ret;
  }

  gpio_set_level(dht22_data_io, 1);
  dht22_data->state = k_dht22_ready;
  ESP_LOGI(dht22_tag, "Sensor Configuration Complete");
  return ESP_OK;
}

esp_err_t dht22_read(dht22_data_t *sensor_data)
{
  if (sensor_data == NULL) {
    ESP_LOGE(dht22_tag, "Sensor data pointer is NULL");
    return ESP_FAIL;
  }

  uint8_t   data_buffer[5] = {0};
  esp_err_t ret;

  /* Send start signal and wait for response */
  priv_dht22_send_start_signal();
  gpio_set_direction(dht22_data_io, GPIO_MODE_INPUT);

  ret = priv_dht22_wait_for_response();
  if (ret != ESP_OK) {
    sensor_data->state = k_dht22_error;
    ESP_LOGE(dht22_tag, "Failed to receive response from DHT22");
    return ESP_FAIL;
  }

  /* Read data bits */
  ret = priv_dht22_read_data_bits(data_buffer);
  if (ret != ESP_OK) {
    sensor_data->state = k_dht22_error;
    ESP_LOGE(dht22_tag, "Failed to read data bits from DHT22");
    return ESP_FAIL;
  }

  /* Verify checksum */
  ret = priv_dht22_verify_checksum(data_buffer);
  if (ret != ESP_OK) {
    sensor_data->state = k_dht22_error;
    ESP_LOGE(dht22_tag, "Checksum verification failed");
    return ESP_FAIL;
  }

  /* Convert raw data to meaningful values */
  sensor_data->humidity      = ((data_buffer[0] << 8) + data_buffer[1]) * 0.1f;
  sensor_data->temperature_c = (((data_buffer[2] & 0x7F) << 8) + data_buffer[3]) * 0.1f;
  sensor_data->temperature_f = sensor_data->temperature_c * 9.0 / 5.0 + 32; /* Convert Celsius to Fahrenheit */

  /* Handle negative temperatures */
  if (data_buffer[2] & 0x80) {
    sensor_data->temperature_c = -sensor_data->temperature_c;
    sensor_data->temperature_f = -sensor_data->temperature_f;
  }

  sensor_data->state = k_dht22_data_updated;

  ESP_LOGI(dht22_tag,
           "Temperature: %.1f F (%.1f C), Humidity: %.1f %%",
           sensor_data->temperature_f,
           sensor_data->temperature_c,
           sensor_data->humidity);
  return ESP_OK;
}

void dht22_reset_on_error(dht22_data_t *sensor_data)
{
  if (sensor_data->state & k_dht22_error) {
    TickType_t current_ticks = xTaskGetTickCount();

    if ((current_ticks - sensor_data->last_attempt_ticks) > sensor_data->retry_interval) {
      ESP_LOGI(dht22_tag, "Attempting to reset DHT22 sensor");

      esp_err_t ret = dht22_init(sensor_data);
      if (ret == ESP_OK) {
        sensor_data->state          = k_dht22_ready;
        sensor_data->retry_count    = 0;
        sensor_data->retry_interval = dht22_initial_retry_interval;
        ESP_LOGI(dht22_tag, "DHT22 sensor reset successfully.");
      } else {
        sensor_data->retry_count++;
        if (sensor_data->retry_count >= dht22_max_retries) {
          sensor_data->retry_count    = 0;
          sensor_data->retry_interval = (sensor_data->retry_interval * 2 > dht22_max_backoff_interval) ? 
                                        dht22_max_backoff_interval : 
                                        sensor_data->retry_interval * 2;
        }
      }

      sensor_data->last_attempt_ticks = current_ticks;
    }
  }
}

void dht22_tasks(void *sensor_data)
{
  dht22_data_t *dht22_data = (dht22_data_t *)sensor_data;
  while (1) {
    if (dht22_read(dht22_data) == ESP_OK) {
      char *json = dht22_data_to_json(dht22_data);
      send_sensor_data_to_webserver(json);
      free(json);
    } else {
      dht22_reset_on_error(dht22_data);
    }
    vTaskDelay(dht22_polling_rate_ticks);
  }
}

