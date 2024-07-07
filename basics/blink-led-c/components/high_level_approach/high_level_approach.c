#include "high_level_approach.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

// Define the GPIO pin where the LED is connected
#define BLINK_GPIO GPIO_NUM_2

void high_level_approach() {
  // Configure the GPIO pin as output
  esp_rom_gpio_pad_select_gpio(BLINK_GPIO);
  gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

  while (1) {
    // Turn the LED on
    gpio_set_level(BLINK_GPIO, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second

    // Turn the LED off
    gpio_set_level(BLINK_GPIO, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
  }
}
