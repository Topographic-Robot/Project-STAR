#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

/* Define pin */
#define DHT_PIN GPIO_NUM_4

static const char *TAG = "DHT22";

typedef struct {
  float temperature;
  float humidity;
} dht_data_t;

static void set_gpio_input() { gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT); }

static void set_gpio_output() { gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT); }

static int wait_for_level(int level, uint32_t timeout_us) {
  int elapsed_time = 0;
  while (gpio_get_level(DHT_PIN) != level) {
    if (elapsed_time > timeout_us) {
      return -1; /* Timeout */
    }
    esp_rom_delay_us(1);
    elapsed_time++;
  }
  return elapsed_time;
}

static int read_dht_data(dht_data_t *data) {
  uint8_t bits[5]    = {0};
  uint8_t byte_index = 0, bit_index = 7;
  set_gpio_output();
  gpio_set_level(DHT_PIN, 0);
  vTaskDelay(20 / portTICK_PERIOD_MS); /* 20ms delay */
  gpio_set_level(DHT_PIN, 1);
  esp_rom_delay_us(40);
  set_gpio_input();

  if (wait_for_level(0, 80) == -1 || wait_for_level(1, 80) == -1 ||
      wait_for_level(0, 80) == -1) {
    return -1; /* DHT22 not responding */
  }

  for (int i = 0; i < 40; i++) {
    if (wait_for_level(1, 50) == -1) {
      return -1; /* Timeout waiting for bit start */
    }
    int duration = wait_for_level(0, 70);
    if (duration == -1) {
      return -1; /* Timeout waiting for bit end */
    }
    if (duration > 40) {
      bits[byte_index] |= (1 << bit_index);
    }
    if (bit_index == 0) {
      bit_index = 7;
      byte_index++;
    } else {
      bit_index--;
    }
  }

  if (bits[4] != ((bits[0] + bits[1] + bits[2] + bits[3]) & 0xFF)) {
    return -1; /* Checksum failed */
  }

  data->humidity    = ((bits[0] << 8) + bits[1]) * 0.1;
  data->temperature = (((bits[2] & 0x7F) << 8) + bits[3]) * 0.1;
  if (bits[2] & 0x80) {
    data->temperature = -data->temperature;
  }

  return 0;
}

void app_main() {
  dht_data_t dht_data;

  while (1) {
    if (read_dht_data(&dht_data) == 0) {
      ESP_LOGI(TAG, "Humidity: %.1f%% Temperature: %.1fC", dht_data.humidity,
               dht_data.temperature);
    } else {
      ESP_LOGE(TAG, "Failed to read from DHT22 sensor");
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
