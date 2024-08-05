#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DHT_PIN GPIO_NUM_4

static const char *TAG = "DHT22";

typedef struct {
  float temperature;
  float humidity;
} dht_data_t; /* XXX: This should be dht22_t */

/* XXX: Why is this needed? */
static int wait_for_level(int level, uint32_t timeout_us) {
  int elapsed_time = 0;
  while (gpio_get_level(DHT_PIN) != level) {
    if (elapsed_time > timeout_us) {
      return -1; /* Timeout */
    }
    esp_rom_delay_us(1); /* XXX: Is using a timer better? */
    elapsed_time++;
  }
  return elapsed_time;
}

static int read_dht_data(dht_data_t *data) {
  uint8_t bits[5]    = {0};
  uint8_t byte_index = 0, bit_index = 7;
  /* XXX: Why do we set OUTPUT then INPUT
   * Is this setting this to be both input and output? */
  gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(DHT_PIN, 0);

  vTaskDelay(20 / portTICK_PERIOD_MS); /* 20ms delay */
  gpio_set_level(DHT_PIN, 1);
  esp_rom_delay_us(40);
  gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);

  if (wait_for_level(0, 80) == -1 || wait_for_level(1, 80) == -1 ||
      wait_for_level(0, 80) == -1) {
    return -1; /* DHT22 not responding */
  }

  /* XXX: Where is 40 from */
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

  /* XXX: Add something to explain this checksum */
  if (bits[4] != ((bits[0] + bits[1] + bits[2] + bits[3]) & 0xFF)) {
    return -1; /* Checksum failed */
  }

  /* XXX: Move these into constants or macros */
  data->humidity    = ((bits[0] << 8) + bits[1]) * 0.1;
  data->temperature = (((bits[2] & 0x7F) << 8) + bits[3]) * 0.1;
  /* XXX: Explain 0x80 */
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
