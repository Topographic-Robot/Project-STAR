#include "gy_neo6mv2_hal.h"
#include "common/uart.h"
#include <esp_log.h>

const char    *gy_neo6mv2_tag                = "GY-NEO6MV2";
const uint8_t  gy_neo6mv2_tx_io              = GPIO_NUM_17; /* TX pin */
const uint8_t  gy_neo6mv2_rx_io              = GPIO_NUM_16; /* RX pin */
const uint32_t gy_neo6mv2_uart_baudrate      = 9600;        /* Baud rate */
const uint32_t gy_neo6mv2_polling_rate_ticks = pdMS_TO_TICKS(5 * 1000);

/* Public Functions ***********************************************************/

esp_err_t gy_neo6mv2_init(gy_neo6mv2_data_t *sensor_data, bool first_time)
{
  if (first_time) {
    sensor_data->data_mutex = NULL; /* Set NULL and initialize later */
  }

  /* Initialize UART using the common UART function */
  esp_err_t ret = priv_uart_init(gy_neo6mv2_tx_io, gy_neo6mv2_rx_io, 
                                 gy_neo6mv2_uart_baudrate, UART_NUM_1, gy_neo6mv2_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(gy_neo6mv2_tag, "GY-NEO6MV2 UART initialization failed");
    return ret;
  }

  if (sensor_data->data_mutex == NULL) {
    sensor_data->data_mutex = xSemaphoreCreateMutex();
    if (sensor_data->data_mutex == NULL) {
      ESP_LOGE(gy_neo6mv2_tag, "Memory allocation failed");
      return ESP_ERR_NO_MEM;
    }
  }

  ESP_LOGI(gy_neo6mv2_tag, "GY-NEO6MV2 initialized");
  return ESP_OK;
}

void gy_neo6mv2_read(gy_neo6mv2_data_t *sensor_data)
{
  if (sensor_data == NULL) {
    ESP_LOGE(gy_neo6mv2_tag, "GPS data pointer is NULL");
    return;
  }

  if (xSemaphoreTake(sensor_data->data_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(gy_neo6mv2_tag, "Failed to take data mutex");
    return;
  }

  uint8_t data[128]; /* Buffer to hold NMEA data */

  /* Read UART data using the common function */
  esp_err_t ret = priv_uart_read(data, sizeof(data), UART_NUM_1, gy_neo6mv2_tag);
  if (ret == ESP_OK) {
    /* TODO: Process the NMEA data (for example, extract GPRMC sentence) */
    ESP_LOGI(gy_neo6mv2_tag, "Received NMEA: %s", data);
  }

  xSemaphoreGive(sensor_data->data_mutex);
}

void gy_neo6mv2_tasks(void *sensor_data)
{
  gy_neo6mv2_data_t *gy_neo6mv2_data = (gy_neo6mv2_data_t *)sensor_data;
  gy_neo6mv2_read(gy_neo6mv2_data);
  vTaskDelay(gy_neo6mv2_polling_rate_ticks);
}
