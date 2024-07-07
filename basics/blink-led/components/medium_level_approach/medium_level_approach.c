#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "medium_level_approach.h"
#include "soc/gpio_struct.h"

#define LED_PIN GPIO_NUM_2

void medium_level_approach(void) {
  // Set pin as output
  GPIO.enable_w1ts             = (1 << LED_PIN);
  GPIO.pin[LED_PIN].pad_driver = 0; // Set push-pull mode
  GPIO.func_out_sel_cfg[LED_PIN].func_sel =
      256; // GPIO_FUNC_OUT_SEL (FUNC_SEL = 256)

  while (1) {
    // Set pin high
    GPIO.out_w1ts = (1 << LED_PIN);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // Set pin low
    GPIO.out_w1tc = (1 << LED_PIN);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
