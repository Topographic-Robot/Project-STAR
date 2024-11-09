#include "sensor_hal.h"
#include "common/gpio.h"
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* Constants *******************************************************************/

const char    *dht22_tag                = "DHT22";
const uint8_t  dht22_data_io            = GPIO_NUM_4;
const uint32_t dht22_polling_rate_ticks = pdMS_TO_TICKS(5 * 1000);
const uint8_t  dht22_bit_count          = 40;

/**
 * Increasing this value:
 *   - Extends the time the GPIO pin is held low before sending the start signal.
 *   - A small increase might still work, but holding the line too long could 
 *     cause the DHT22 to ignore the start signal.
 * Decreasing this value:
 *   - Reduces the time the pin is held low. If decreased too much, 
 *     the DHT22 may not detect the start signal and fail to respond.
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
 * @brief Wait for a specific GPIO level within the given timeout and measure the duration.
 * 
 * @param level Desired GPIO level (0 or 1).
 * @param timeout_us Maximum time to wait for the level, in microseconds.
 * @param duration_us Pointer to store the time taken to reach the level, in microseconds.
 * @return true if level reached successfully within timeout, false if timeout occurred.
 */
static bool priv_dht22_wait_for_level_with_duration(int level, uint32_t timeout_us, uint32_t *duration_us) 
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
 * Sets the GPIO direction to output, holds the line low for a set time, then releases it.
 */
static void priv_dht22_send_start_signal(void)
{
    gpio_set_direction(dht22_data_io, GPIO_MODE_OUTPUT);
    gpio_set_level(dht22_data_io, 0); /* Pull line low */
    esp_rom_delay_us(dht22_start_delay_ms * 1000); /* Wait for start signal duration in microseconds */

    gpio_set_level(dht22_data_io, 1); /* Release line */
    esp_rom_delay_us(30); /* Wait for 30 microseconds before switching to input */
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
    uint32_t duration;

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
 * Waits for the bit start and end signals, and interprets the bit based on timing.
 * 
 * @return 0 for a '0' bit, 1 for a '1' bit, ESP_FAIL on timeout.
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
 * Data is read bit by bit and stored in the provided data buffer.
 * 
 * @param[out] data_buffer Array to store the 40 bits (5 bytes) of data.
 * @return ESP_OK on success, ESP_FAIL on timeout or other errors.
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
        sensor_data->temperature_c = -sensor_data->temperature_c;
        sensor_data->temperature_f = -sensor_data->temperature_f;
    }

    return ESP_OK;
}

/* Public Functions ************************************************************/

esp_err_t dht22_init(void *sensor_data) 
{
    sensor_data_t *all_sensor_data = (sensor_data_t *)sensor_data;
    dht22_data_t  *dht22_data      = &all_sensor_data->dht22_data;
    ESP_LOGI(dht22_tag, "Starting Configuration");

    dht22_data->humidity      = -1.0;
    dht22_data->temperature_f = -1.0;
    dht22_data->temperature_c = -1.0;
    dht22_data->state         = k_dht22_uninitialized; /* Start uninitialized */

    esp_err_t ret = priv_gpio_init(dht22_data_io);
    if (ret != ESP_OK) {
        ESP_LOGE(dht22_tag, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Pull the line high (logic 1) before starting communication */
    gpio_set_level(dht22_data_io, 1);

    dht22_data->state = k_dht22_ready;
    ESP_LOGI(dht22_tag, "Sensor Configuration Complete");
    return ESP_OK;
}

void dht22_read(dht22_data_t *sensor_data) 
{
    /* Check if the sensor data is NULL */
    if (sensor_data == NULL) {
        ESP_LOGE(dht22_tag, "Sensor data pointer is NULL");
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
        return;
    }

    sensor_data->state = k_dht22_data_updated;

    ESP_LOGI(dht22_tag, 
        "Temperature: %.1f F (%.1f C), Humidity: %.1f %%", 
        sensor_data->temperature_f, 
        sensor_data->temperature_c, 
        sensor_data->humidity);
}

void dht22_tasks(void *sensor_data) 
{
    sensor_data_t *all_sensor_data = (sensor_data_t *)sensor_data;
    dht22_data_t  *dht22_data      = &all_sensor_data->dht22_data;
    while (1) {
        dht22_read(dht22_data);
        vTaskDelay(dht22_polling_rate_ticks);
    }
}

