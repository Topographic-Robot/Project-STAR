#include "sensor_hal.h"
#include "common/uart.h"
#include <esp_log.h>

const char    *gy_neo6mv2_tag                = "GY-NEO6MV2";
const uint8_t  gy_neo6mv2_tx_io              = GPIO_NUM_17; /* TX pin */
const uint8_t  gy_neo6mv2_rx_io              = GPIO_NUM_16; /* RX pin */
const uint32_t gy_neo6mv2_uart_baudrate      = 9600;        /* Baud rate */
const uint32_t gy_neo6mv2_polling_rate_ticks = pdMS_TO_TICKS(5 * 1000);

/* Public Functions ***********************************************************/

esp_err_t gy_neo6mv2_init(void *sensor_data)
{
  ESP_LOGI(gy_neo6mv2_tag, "Starting Configuration");

  /* Initialize UART using the common UART function */
  esp_err_t ret = priv_uart_init(gy_neo6mv2_tx_io, gy_neo6mv2_rx_io, 
                                 gy_neo6mv2_uart_baudrate, UART_NUM_1, gy_neo6mv2_tag);
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

  /* Read UART data using the common function */
  esp_err_t ret = priv_uart_read(data, sizeof(data), UART_NUM_1, gy_neo6mv2_tag);
  if (ret == ESP_OK) {
    /* TODO: Process the NMEA data (for example, extract GPRMC sentence) */
    ESP_LOGI(gy_neo6mv2_tag, "Received NMEA: %s", data);
  }
}

void gy_neo6mv2_tasks(void *sensor_data)
{
  sensor_data_t     *all_sensor_data = (sensor_data_t *)sensor_data;
  gy_neo6mv2_data_t *gy_neo6mv2_data = &all_sensor_data->gy_neo6mv2_data;
  while (1) {
    gy_neo6mv2_read(gy_neo6mv2_data);
    vTaskDelay(gy_neo6mv2_polling_rate_ticks);
  }
}
