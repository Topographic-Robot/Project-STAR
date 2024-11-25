/* components/sensors/gy_neo6mv2_hal/gy_neo6mv2_hal.c */

#include "gy_neo6mv2_hal.h"
#include <string.h>
#include <stdlib.h>
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
const uint8_t  gy_neo6mv2_sentence_buffer_size   = 128;

/* Static (Private) Functions **************************************************/

/**
 * @brief Parses a GPS coordinate from NMEA format to decimal degrees.
 *
 * The GY-NEO6MV2 GPS module outputs coordinates in the NMEA format, which represents
 * latitude and longitude in degrees and minutes. This function converts the NMEA string
 * representation to decimal degrees, adjusting for the hemisphere if necessary.
 *
 * **Logic and Flow:**
 * - Extract the degrees from the NMEA coordinate string.
 * - Calculate the decimal degrees using the minutes part.
 * - Adjust the final value based on the hemisphere (negative for South or West).
 *
 * @param[in] coord_str Pointer to a string containing the coordinate in NMEA format.
 * @param[in] hemisphere Pointer to a string containing the hemisphere character ('N', 'S', 'E', 'W').
 *
 * @return The parsed coordinate in decimal degrees.
 */
static float parse_coordinate(const char *coord_str, const char *hemisphere)
{
  float coord           = atof(coord_str);
  int   degrees         = (int)(coord / 100);
  float minutes         = coord - (degrees * 100);
  float decimal_degrees = degrees + (minutes / 60.0);

  if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
    decimal_degrees = -decimal_degrees;
  }

  return decimal_degrees;
}

/**
 * @brief Parses the GPRMC NMEA sentence to extract GPS information.
 *
 * The GPRMC sentence contains essential information such as time, latitude, longitude,
 * speed, and GPS fix status. This function extracts these fields and updates the
 * `gy_neo6mv2_data_t` structure. It uses a stack-allocated buffer to avoid dynamic 
 * memory allocation within the task, preventing potential stack overflows that could
 * occur with heap allocation using functions like `strdup`.
 *
 * **Logic and Flow:**
 * - Creates a copy of the input sentence in a local buffer on the stack to avoid 
 *   modifying the original string.
 * - Tokenizes the GPRMC sentence using commas as delimiters.
 * - Checks if the sentence is valid (enough tokens) and the GPS fix status.
 * - If the fix is valid ('A' status), extracts latitude, longitude, time, and speed.
 * - Updates the GPS data structure with the extracted information and sets the state
 *   to `k_gy_neo6mv2_data_updated`.
 * - If the fix status is invalid ('V' status) or the sentence is incomplete, updates
 *   the state to `k_gy_neo6mv2_error`.
 *
 * @param[in] sentence Pointer to the GPRMC NMEA sentence string.
 * @param[out] sensor_data Pointer to `gy_neo6mv2_data_t` structure to store extracted GPS data.
 */
static void parse_gprmc(const char *sentence, gy_neo6mv2_data_t *sensor_data)
{
  char *tokens[20];
  int   token_index = 0;

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
  if (tokens[2][0] != 'A') {
    /* Status V means invalid data */
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
  sensor_data->latitude = parse_coordinate(tokens[3], tokens[4]);

  /* Parse longitude */
  sensor_data->longitude = parse_coordinate(tokens[5], tokens[6]);

  /* Parse speed over ground in knots, convert to meters per second */
  float speed_knots  = atof(tokens[7]);
  sensor_data->speed = speed_knots * 0.514444; /* 1 knot = 0.514444 m/s */

  sensor_data->state = k_gy_neo6mv2_data_updated;
}

/* Public Functions ***********************************************************/

char *gy_neo6mv2_data_to_json(const gy_neo6mv2_data_t *data) 
{
  cJSON *json = cJSON_CreateObject();
  cJSON_AddStringToObject(json, "sensor_type", "gps");
  cJSON_AddNumberToObject(json, "latitude", data->latitude);
  cJSON_AddNumberToObject(json, "longitude", data->longitude);
  cJSON_AddNumberToObject(json, "speed", data->speed);
  cJSON_AddStringToObject(json, "time", data->time);
  char *json_string = cJSON_PrintUnformatted(json);
  cJSON_Delete(json);
  return json_string;
}

esp_err_t gy_neo6mv2_init(void *sensor_data)
{
  ESP_LOGI(gy_neo6mv2_tag, "Starting Configuration");

  /* Initialize UART using the common UART function */
  esp_err_t ret = priv_uart_init(gy_neo6mv2_tx_io, gy_neo6mv2_rx_io,
                                 gy_neo6mv2_uart_baudrate, 
                                 gy_neo6mv2_uart_num, 
                                 gy_neo6mv2_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(gy_neo6mv2_tag, "GY-NEO6MV2 UART initialization failed");
    return ret;
  }

  /* Initialize GPS data and state */
  gy_neo6mv2_data_t *gy_neo6mv2_data  = (gy_neo6mv2_data_t *)sensor_data;
  gy_neo6mv2_data->latitude           = 0.0;
  gy_neo6mv2_data->longitude          = 0.0;
  gy_neo6mv2_data->speed              = 0.0;
  memset(gy_neo6mv2_data->time, 0, sizeof(gy_neo6mv2_data->time));
  gy_neo6mv2_data->fix_status         = 0; // Initially no fix
  gy_neo6mv2_data->state              = k_gy_neo6mv2_uninitialized;
  gy_neo6mv2_data->retry_count        = 0;
  gy_neo6mv2_data->retry_interval     = gy_neo6mv2_initial_retry_interval;
  gy_neo6mv2_data->last_attempt_ticks = 0;

  ESP_LOGI(gy_neo6mv2_tag, "Sensor Configuration Complete");
  return ESP_OK;
}

esp_err_t gy_neo6mv2_read(gy_neo6mv2_data_t *sensor_data)
{
  if (sensor_data == NULL) {
    ESP_LOGE(gy_neo6mv2_tag, "GPS data pointer is NULL");
    return ESP_FAIL;
  }

  uint8_t data[gy_neo6mv2_sentence_buffer_size]; /* Buffer to hold NMEA data */

  esp_err_t ret = priv_uart_read(data, sizeof(data) - 1, gy_neo6mv2_uart_num, 
                                 gy_neo6mv2_tag);
  if (ret == ESP_OK) {
    data[sizeof(data) - 1] = '\0'; /* Null-terminate the data */
    ESP_LOGI(gy_neo6mv2_tag, "Received NMEA: %s", data);

    /* Process NMEA sentences */
    char *line = strtok((char *)data, "\n");
    while (line != NULL) {
      if (strncmp(line, "$GPRMC", 6) == 0) {
        parse_gprmc(line, sensor_data);
        break;
      }
      line = strtok(NULL, "\n");
    }
  } else {
    ESP_LOGW(gy_neo6mv2_tag, "No data read from UART");
    sensor_data->state = k_gy_neo6mv2_error; /* Set error state if read fails */
    return ESP_FAIL;
  }


  return ESP_OK;
}

void gy_neo6mv2_reset_on_error(gy_neo6mv2_data_t *sensor_data)
{
  if (sensor_data->state == k_gy_neo6mv2_error) {
    TickType_t current_ticks = xTaskGetTickCount();

    if ((current_ticks - sensor_data->last_attempt_ticks) > sensor_data->retry_interval) {
      ESP_LOGI(gy_neo6mv2_tag, "Attempting to reset GY-NEO6MV2 GPS module");

      esp_err_t ret = gy_neo6mv2_init(sensor_data);
      if (ret == ESP_OK) {
        sensor_data->state = k_gy_neo6mv2_ready;
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
      gy_neo6mv2_reset_on_error(gy_neo6mv2_data);
    }
    vTaskDelay(gy_neo6mv2_polling_rate_ticks);
  }
}
