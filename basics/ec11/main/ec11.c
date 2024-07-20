#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <inttypes.h>

#define EC11_BUTTON      GPIO_NUM_18
#define EC11_OUT_A       GPIO_NUM_19
#define EC11_OUT_B       GPIO_NUM_21
#define DEBOUNCE_TIME_MS 50 /* debounce time in milliseconds */

static const char *TAG = "EC11";

/* Define a structure for GPIO events */
typedef struct {
  uint32_t gpio_num;
  int      level;
} gpio_event_t;

/* Declare a queue handle */
static QueueHandle_t gpio_evt_queue         = NULL;
static int64_t       last_gpio_event_time_a = 0;
static int64_t       last_gpio_event_time_b = 0;

static int prev_state = 0;

/* Interrupt service routine for EC11_OUT_A and EC11_OUT_B */
static void IRAM_ATTR gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  int64_t  now = esp_timer_get_time() / 1000; /* current time in milliseconds */

  if (gpio_num == EC11_OUT_A) {
    if (now - last_gpio_event_time_a > DEBOUNCE_TIME_MS) {
      last_gpio_event_time_a = now;
      gpio_event_t evt;
      evt.gpio_num = gpio_num;
      evt.level    = gpio_get_level(gpio_num);
      xQueueSendFromISR(gpio_evt_queue, &evt, NULL);
    }
  } else if (gpio_num == EC11_OUT_B) {
    if (now - last_gpio_event_time_b > DEBOUNCE_TIME_MS) {
      last_gpio_event_time_b = now;
      gpio_event_t evt;
      evt.gpio_num = gpio_num;
      evt.level    = gpio_get_level(gpio_num);
      xQueueSendFromISR(gpio_evt_queue, &evt, NULL);
    }
  }
}

void gpio_task(void *arg) {
  gpio_event_t evt;
  while (1) {
    if (xQueueReceive(gpio_evt_queue, &evt, portMAX_DELAY)) {
      int current_state =
          (gpio_get_level(EC11_OUT_A) << 1) | gpio_get_level(EC11_OUT_B);

      const char *state_str[] = {"00", "01", "11", "10"};
      ESP_LOGI(TAG, "State: %s", state_str[current_state]);

      if (evt.gpio_num == EC11_OUT_A || evt.gpio_num == EC11_OUT_B) {
        switch (prev_state) {
        case 0:
          if (current_state == 1) {
            ESP_LOGI(TAG, "Clockwise");
          } else if (current_state == 2) {
            ESP_LOGI(TAG, "Counterclockwise");
          }
          break;
        case 1:
          if (current_state == 3) {
            ESP_LOGI(TAG, "Clockwise");
          } else if (current_state == 0) {
            ESP_LOGI(TAG, "Counterclockwise");
          }
          break;
        case 3:
          if (current_state == 2) {
            ESP_LOGI(TAG, "Clockwise");
          } else if (current_state == 1) {
            ESP_LOGI(TAG, "Counterclockwise");
          }
          break;
        case 2:
          if (current_state == 0) {
            ESP_LOGI(TAG, "Clockwise");
          } else if (current_state == 3) {
            ESP_LOGI(TAG, "Counterclockwise");
          }
          break;
        }
        prev_state = current_state;
      }

      ESP_LOGI(TAG, "GPIO[%" PRIu32 "] state changed to: %d", evt.gpio_num,
               evt.level);
    }
  }
}

void app_main(void) {
  /* Configure the button GPIO as input */
  gpio_config_t io_conf_button = {.pin_bit_mask = (1ULL << EC11_BUTTON),
                                  .mode         = GPIO_MODE_INPUT,
                                  .pull_up_en   = GPIO_PULLUP_ENABLE,
                                  .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                  .intr_type    = GPIO_INTR_DISABLE};
  gpio_config(&io_conf_button);

  /* Configure the out_a GPIO as input with interrupt on both edges */
  gpio_config_t io_conf_out_a = {.pin_bit_mask = (1ULL << EC11_OUT_A),
                                 .mode         = GPIO_MODE_INPUT,
                                 .pull_up_en   = GPIO_PULLUP_ENABLE,
                                 .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                 .intr_type    = GPIO_INTR_ANYEDGE};
  gpio_config(&io_conf_out_a);

  /* Configure the out_b GPIO as input with interrupt on both edges */
  gpio_config_t io_conf_out_b = {.pin_bit_mask = (1ULL << EC11_OUT_B),
                                 .mode         = GPIO_MODE_INPUT,
                                 .pull_up_en   = GPIO_PULLUP_ENABLE,
                                 .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                 .intr_type    = GPIO_INTR_ANYEDGE};
  gpio_config(&io_conf_out_b);

  /* Create a queue to handle GPIO events */
  gpio_evt_queue = xQueueCreate(10, sizeof(gpio_event_t));

  /* Start a task to handle GPIO events */
  xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

  /* Install GPIO ISR service */
  gpio_install_isr_service(0);

  /* Attach the interrupt service routine to the GPIOs */
  gpio_isr_handler_add(EC11_OUT_A, gpio_isr_handler, (void *)EC11_OUT_A);
  gpio_isr_handler_add(EC11_OUT_B, gpio_isr_handler, (void *)EC11_OUT_B);

  uint8_t last_button_state = 1;

  while (1) {
    int button_state = gpio_get_level(EC11_BUTTON);
    if (button_state == 0 && last_button_state == 1) {
      ESP_LOGI("Button", "Button pressed!");
    }
    last_button_state = button_state;

    /* Add a delay to avoid bouncing issues */
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
