/* components/sensors/gy_neo6mv2_hal/gy_neo6mv2_hal.c */

/* TODO: Test this */

#include "gy_neo6mv2_hal.h"
#include <string.h>
#include <stdlib.h>
#include "esp_err.h"
#include "webserver_tasks.h"
#include "cJSON.h"
#include "common/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

/* Constants *******************************************************************/

const char    *gy_neo6mv2_tag                    = "GY-NEO6MV2";
const uint8_t  gy_neo6mv2_tx_io                  = GPIO_NUM_17;
const uint8_t  gy_neo6mv2_rx_io                  = GPIO_NUM_16;
const uint8_t  gy_neo6mv2_uart_num               = UART_NUM_2;
const uint32_t gy_neo6mv2_uart_baudrate          = 9600;
const uint32_t gy_neo6mv2_polling_rate_ticks     = pdMS_TO_TICKS(5 * 1000);
const uint8_t  gy_neo6mv2_max_retries            = 4;
const uint32_t gy_neo6mv2_initial_retry_interval = pdMS_TO_TICKS(15 * 1000);
const uint32_t gy_neo6mv2_max_backoff_interval   = pdMS_TO_TICKS(480 * 1000);

/* Globals (Static) ***********************************************************/

/**
 * @brief Buffer for assembling fragmented NMEA sentences.
 *
 * This static buffer stores parts of NMEA sentences received from the GPS module.
 * It accumulates characters until a complete sentence is formed, identified by a newline character.
 * The buffer helps handle cases where sentences are split across multiple UART reads.
 */
static char sentence_buffer[gy_neo6mv2_sentence_buffer_size];

/**
 * @brief Index to track the current position in the sentence buffer.
 *
 * This static variable keeps track of the current insertion point in the `sentence_buffer`.
 * It increments with each character read and resets when a complete sentence is processed.
 */
static uint32_t sentence_index = 0;

/* Static (Private) Functions **************************************************/

/**
 * @brief Parses a GPS coordinate from NMEA format to decimal degrees.
 *
 * This function converts a coordinate from the NMEA format used by the GY-NEO6MV2 GPS module into
 * decimal degrees. NMEA coordinates are in the format DDMM.MMMM, where DD is degrees and MM.MMMM is minutes.
 * The function also adjusts the coordinate based on the hemisphere.
 *
 * **Logic and Flow:**
 * - Parses the coordinate string into degrees and minutes.
 * - Converts minutes into decimal degrees.
 * - Adjusts the sign based on the hemisphere.
 *
 * @param[in] coord_str Pointer to a string containing the coordinate in NMEA format.
 * @param[in] hemisphere Pointer to a character indicating the hemisphere ('N', 'S', 'E', 'W').
 *
 * @return The coordinate in decimal degrees.
 */
static float priv_parse_coordinate(const char *coord_str, const char *hemisphere)
{
  float    coord           = atof(coord_str);
  uint32_t degrees         = (uint32_t)(coord / 100);
  float    minutes         = coord - (degrees * 100);
  float    decimal_degrees = degrees + (minutes / 60.0);

  if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
    decimal_degrees = -decimal_degrees;
  }

  return decimal_degrees;
}

/**
 * @brief Parses the GPRMC NMEA sentence to extract GPS information.
 *
 * This function parses a GPRMC (Recommended Minimum Specific GPS/Transit Data) sentence from the GPS module
 * and extracts the time, latitude, longitude, speed, and fix status. It updates the provided
 * `gy_neo6mv2_data_t` structure with the extracted gy_neo6mv2_data.
 *
 * **Logic and Flow:**
 * - Copies the sentence to a local buffer to avoid modifying the original.
 * - Tokenizes the sentence using commas.
 * - Validates the sentence structure and fix status.
 * - Parses and converts latitude and longitude to decimal degrees.
 * - Parses speed over ground in knots and converts it to meters per second.
 * - Updates the `sensor_data` structure with the parsed values.
 *
 * @param[in] sentence Pointer to the GPRMC NMEA sentence string.
 * @param[out] sensor_data Pointer to `gy_neo6mv2_data_t` structure to store the extracted GPS gy_neo6mv2_data.
 */
static void priv_parse_gprmc(const char *sentence, gy_neo6mv2_data_t *sensor_data)
{
  char    *tokens[20];
  uint32_t token_index = 0;

  /* Use stack-allocated buffer to avoid heap allocation */
  char sentence_copy[gy_neo6mv2_sentence_buffer_size];
  strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
  sentence_copy[sizeof(sentence_copy) - 1] = '\0';

  char *token = strtok(sentence_copy, ",");
  while (token != NULL && token_index < 20) {
    tokens[token_index++] = token;
    token                 = strtok(NULL, ",");
  }

  if (token_index < 12) {
    ESP_LOGE(gy_neo6mv2_tag, "Incomplete GPRMC sentence");
    sensor_data->state = k_gy_neo6mv2_error;
    return;
  }

  /* Check status */
  if (tokens[2][0] != 'A') { /* Status: V = invalid, A = valid */
    sensor_data->fix_status = 0;
    sensor_data->state      = k_gy_neo6mv2_error;
    ESP_LOGW(gy_neo6mv2_tag, "No GPS fix");
    return;
  } else {
    sensor_data->fix_status = 1;
  }

  /* Parse time */
  strncpy(sensor_data->time, tokens[1], sizeof(sensor_data->time));
  sensor_data->time[sizeof(sensor_data->time) - 1] = '\0';

  /* Parse latitude */
  sensor_data->latitude = priv_parse_coordinate(tokens[3], tokens[4]);

  /* Parse longitude */
  sensor_data->longitude = priv_parse_coordinate(tokens[5], tokens[6]);

  /* Parse speed over ground in knots, convert to meters per second */
  float speed_knots  = atof(tokens[7]);
  sensor_data->speed = speed_knots * 0.514444; /* 1 knot = 0.514444 m/s */

  sensor_data->state = k_gy_neo6mv2_data_updated;
  ESP_LOGI(gy_neo6mv2_tag, "Parsed GPRMC: Lat=%.6f, Lon=%.6f, Speed=%.2f m/s",
      sensor_data->latitude, sensor_data->longitude, sensor_data->speed);
}

/* Public Functions ***********************************************************/

char *gy_neo6mv2_data_to_json(const gy_neo6mv2_data_t *gy_neo6mv2_data)
{
  cJSON *json = cJSON_CreateObject();
  cJSON_AddStringToObject(json, "sensor_type", "gps");
  cJSON_AddNumberToObject(json, "latitude", gy_neo6mv2_data->latitude);
  cJSON_AddNumberToObject(json, "longitude", gy_neo6mv2_data->longitude);
  cJSON_AddNumberToObject(json, "speed", gy_neo6mv2_data->speed);
  cJSON_AddStringToObject(json, "time", gy_neo6mv2_data->time);
  char *json_string = cJSON_PrintUnformatted(json);
  cJSON_Delete(json);
  return json_string;
}

esp_err_t gy_neo6mv2_init(void *sensor_data)
{
  ESP_LOGI(gy_neo6mv2_tag, "Initializing GPS module");

  /* Initialize UART using the common UART function */
  esp_err_t ret = priv_uart_init(gy_neo6mv2_tx_io, gy_neo6mv2_rx_io, gy_neo6mv2_uart_baudrate,
                                 gy_neo6mv2_uart_num, gy_neo6mv2_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(gy_neo6mv2_tag, "UART initialization failed");
    return ret;
  }

  /* Allow time for the GPS module to warm up */
  vTaskDelay(pdMS_TO_TICKS(5000));

  /* Initialize GPS gy_neo6mv2_data and state */
  gy_neo6mv2_data_t *gy_neo6mv2_data = (gy_neo6mv2_data_t *)sensor_data;
  gy_neo6mv2_data->latitude          = 0.0;
  gy_neo6mv2_data->longitude         = 0.0;
  gy_neo6mv2_data->speed             = 0.0;
  memset(gy_neo6mv2_data->time, 0, sizeof(gy_neo6mv2_data->time));
  gy_neo6mv2_data->fix_status = 0; /* Initially no fix */
  gy_neo6mv2_data->state      = k_gy_neo6mv2_uninitialized;

  ESP_LOGI(gy_neo6mv2_tag, "GPS module initialized successfully");
  return ESP_OK;
}

esp_err_t gy_neo6mv2_read(gy_neo6mv2_data_t *sensor_data)
{
  uint8_t gy_neo6mv2_data[gy_neo6mv2_sentence_buffer_size];
  int32_t length = 0; /* Variable to hold the length of gy_neo6mv2_data read */

  /* Read gy_neo6mv2_data from UART */
  esp_err_t ret = priv_uart_read(gy_neo6mv2_data, sizeof(gy_neo6mv2_data), &length, gy_neo6mv2_uart_num, gy_neo6mv2_tag);

  if (ret == ESP_OK && length > 0) {
    ESP_LOGI(gy_neo6mv2_tag, "Raw GPS gy_neo6mv2_data: %.*s", (int)length, (char *)gy_neo6mv2_data);

    for (int i = 0; i < length; i++) {
      if (gy_neo6mv2_data[i] == '\n' || gy_neo6mv2_data[i] == '\r') { /* End of NMEA sentence */
        if (sentence_index > 0) {
          sentence_buffer[sentence_index] = '\0';
          if (strncmp(sentence_buffer, "$GPRMC", 6) == 0) {
            priv_parse_gprmc(sentence_buffer, sensor_data);
          }
          sentence_index = 0; /* Reset buffer */
        }
      } else if (sentence_index < sizeof(sentence_buffer) - 1) {
        sentence_buffer[sentence_index++] = gy_neo6mv2_data[i];
      }
    }
    return ESP_OK;
  } else {
    ESP_LOGE(gy_neo6mv2_tag, "Failed to read GPS gy_neo6mv2_data");
    sensor_data->state = k_gy_neo6mv2_error;
    return ESP_FAIL;
  }
}

void gy_neo6mv2_reset_on_error(gy_neo6mv2_data_t *sensor_data)
{
  if (sensor_data->state == k_gy_neo6mv2_error) {
    TickType_t current_ticks = xTaskGetTickCount();

    if ((current_ticks - sensor_data->last_attempt_ticks) > sensor_data->retry_interval) {
      ESP_LOGI(gy_neo6mv2_tag, "Attempting to reset GY-NEO6MV2 GPS module");

      esp_err_t ret = gy_neo6mv2_init(sensor_data);
      if (ret == ESP_OK) {
        sensor_data->state          = k_gy_neo6mv2_ready;
        sensor_data->retry_count    = 0;
        sensor_data->retry_interval = gy_neo6mv2_initial_retry_interval;
        ESP_LOGI(gy_neo6mv2_tag, "GY-NEO6MV2 GPS module reset successfully.");
      } else {
        sensor_data->retry_count++;
        if (sensor_data->retry_count >= gy_neo6mv2_max_retries) {
          sensor_data->retry_count    = 0;
          sensor_data->retry_interval = (sensor_data->retry_interval * 2 > gy_neo6mv2_max_backoff_interval) ?
            gy_neo6mv2_max_backoff_interval : sensor_data->retry_interval * 2;
        }
      }

      sensor_data->last_attempt_ticks = current_ticks;
    }
  }
}

void gy_neo6mv2_tasks(void *sensor_data)
{
  gy_neo6mv2_data_t *gy_neo6mv2_data = (gy_neo6mv2_data_t *)sensor_data;
  while (1) {
    if (gy_neo6mv2_read(gy_neo6mv2_data) == ESP_OK) {
      char *json = gy_neo6mv2_data_to_json(gy_neo6mv2_data);
      send_sensor_data_to_webserver(json);
      free(json);
    } else {
      ESP_LOGW(gy_neo6mv2_tag, "Error reading GPS gy_neo6mv2_data, resetting...");
      gy_neo6mv2_reset_on_error(gy_neo6mv2_data);
    }
    vTaskDelay(gy_neo6mv2_polling_rate_ticks);
  }
}

