/* components/sensors/gy_neo6mv2_hal/gy_neo6mv2_hal.c */

#include "gy_neo6mv2_hal.h"
#include "common/uart.h"
#include <esp_log.h>
#include <string.h>
#include <stdlib.h>

const char    *gy_neo6mv2_tag                = "GY-NEO6MV2";
const uint8_t  gy_neo6mv2_tx_io              = GPIO_NUM_17; /* TX pin */
const uint8_t  gy_neo6mv2_rx_io              = GPIO_NUM_16; /* RX pin */
const uint8_t  gy_neo6mv2_uart_num           = UART_NUM_2;
const uint32_t gy_neo6mv2_uart_baudrate      = 9600;        /* Baud rate */
const uint32_t gy_neo6mv2_polling_rate_ticks = pdMS_TO_TICKS(5 * 1000);

/* Function to parse coordinates from NMEA format to decimal degrees */
static float parse_coordinate(const char *coord_str, const char *hemisphere)
{
  float coord           = atof(coord_str);
  int   degrees         = (int)(coord / 100);
  float minutes         = coord - (degrees * 100);
  float decimal_degrees = degrees + minutes / 60.0;

  if (hemisphere[0] == 'S' || hemisphere[0] == 'W') {
    decimal_degrees = -decimal_degrees;
  }

  return decimal_degrees;
}

/* Function to parse GPRMC sentence */
static void parse_gprmc(const char *sentence, gy_neo6mv2_data_t *sensor_data)
{
  char *tokens[20];
  int   token_index   = 0;
  char *sentence_copy = strdup(sentence); /* Make a copy since strtok modifies the string */
  char *token         = strtok(sentence_copy, ",");

  while (token != NULL && token_index < 20) {
    tokens[token_index++] = token;
    token                 = strtok(NULL, ",");
  }

  if (token_index < 12) {
    ESP_LOGE(gy_neo6mv2_tag, "Incomplete GPRMC sentence");
    free(sentence_copy);
    return;
  }

  /* Check status */
  if (tokens[2][0] != 'A') {
    /* Status V means invalid data */
    sensor_data->fix_status = 0;
    ESP_LOGW(gy_neo6mv2_tag, "No GPS fix");
    free(sentence_copy);
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
  float speed_knots = atof(tokens[7]);
  sensor_data->speed = speed_knots * 0.514444; /* 1 knot = 0.514444 m/s */

  free(sentence_copy);
}

/* Public Functions ***********************************************************/

esp_err_t gy_neo6mv2_init(void *sensor_data)
{
  ESP_LOGI(gy_neo6mv2_tag, "Starting Configuration");

  /* Initialize UART using the common UART function */
  esp_err_t ret = priv_uart_init(gy_neo6mv2_tx_io, gy_neo6mv2_rx_io,
      gy_neo6mv2_uart_baudrate, gy_neo6mv2_uart_num, 
      gy_neo6mv2_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(gy_neo6mv2_tag, "GY-NEO6MV2 UART initialization failed");
    return ret;
  }

  ESP_LOGI(gy_neo6mv2_tag, "Sensor Configuration Complete");
  return ESP_OK;
}

void gy_neo6mv2_read(gy_neo6mv2_data_t *sensor_data)
{
  if (sensor_data == NULL) {
    ESP_LOGE(gy_neo6mv2_tag, "GPS data pointer is NULL");
    return;
  }

  uint8_t data[128]; /* Buffer to hold NMEA data */

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
  }
}

void gy_neo6mv2_tasks(void *sensor_data)
{
  gy_neo6mv2_data_t *gy_neo6mv2_data = (gy_neo6mv2_data_t *)sensor_data;
  while (1) {
    gy_neo6mv2_read(gy_neo6mv2_data);
    vTaskDelay(gy_neo6mv2_polling_rate_ticks);
  }
}

