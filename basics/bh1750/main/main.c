#include "bh1750.h"
#include "driver/i2c.h"
#include "esp_log.h"

/*
 * BH1750 Connections:
 * VCC -> 3.3V
 * GND -> GND
 * SCL -> GPIO 22
 * SDA -> GPIO 21
 * ADDR -> GND
 */

#define BH1750_SCL    GPIO_NUM_22
#define BH1750_SDA    GPIO_NUM_21
#define BH1750_MASTER I2C_NUM_1
#define BH1750_FREQ   50000 /* in Hz */

static const char *TAG = "Light Sensor";

void app_main(void) {
  /* could handle the error better */
  ESP_ERROR_CHECK(
      bh1750_init(BH1750_SCL, BH1750_SDA, BH1750_FREQ, BH1750_MASTER));

  while (1) {
    float light_intensity = bh1750_read_lux(BH1750_MASTER);
    if (light_intensity >= 0) {
      ESP_LOGI(TAG, "Light Intensity: %0.2f lux", light_intensity);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
