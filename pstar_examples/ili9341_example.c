/* pstar_examples/ili9341_example.c */

#include "sdkconfig.h"

/* Only compile if ILI9341 component is enabled */
#ifdef CONFIG_PSTAR_KCONFIG_ILI9341_ENABLED

#include "pstar_bus_manager.h"
#include "pstar_ili9341_hal.h"
#include "pstar_pin_validator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdlib.h> /* For rand() */

#include "esp_check.h" /* For ESP_ERROR_CHECK */
#include "esp_log.h"
#include "ili9341_example.h"

static const char* TAG_EX = "ILI9341_Example";

/* Simple RGB565 Color Definitions */
#define COLOR_BLACK 0x0000
#define COLOR_BLUE 0x001F
#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_CYAN 0x07FF
#define COLOR_MAGENTA 0xF81F
#define COLOR_YELLOW 0xFFE0
#define COLOR_WHITE 0xFFFF

void example_ili9341_app_main(void)
{
  ESP_LOGI(TAG_EX, "Running: ILI9341 Example");
  esp_err_t                  ret = ESP_OK;
  pstar_bus_manager_t        example_bus_manager;
  pstar_ili9341_hal_handle_t lcd_handle          = NULL;
  bool                       manager_initialized = false;
  bool                       validator_used      = false; /* Track if validator was active */

/* --- Setup Step 1: Register Example Pins (if validator enabled) --- */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  validator_used = true;
  ESP_LOGI(TAG_EX, "Registering ILI9341 example pins...");
  /* Use Kconfig pins for the example */
  ret = pstar_ili9341_register_kconfig_pins();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to register ILI9341 pins: %s", esp_err_to_name(ret));
    goto example_fail; /* Treat as fatal */
  }
  /* --- Setup Step 2: Validate Registered Pins --- */
  ESP_LOGI(TAG_EX, "Validating example pins...");
  ret = pstar_validate_pins();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Pin validation failed for example: %s", esp_err_to_name(ret));
    goto example_fail; /* Treat as fatal */
  }
  ESP_LOGI(TAG_EX, "Example Pin validation successful.");
#else
  ESP_LOGW(TAG_EX, "Pin Validator disabled. Skipping pin checks for example.");
  validator_used = false;
#endif

  /* --- Setup Step 3: Init Bus Manager (local to this example) --- */
  ESP_LOGI(TAG_EX, "Initializing Example Bus Manager...");
  ret = pstar_bus_manager_init(&example_bus_manager, "ILI9341_Example_BusMgr");
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to init example bus manager: %s", esp_err_to_name(ret));
    goto example_fail; /* Cannot proceed */
  }
  manager_initialized = true;

  /* --- Setup Step 4: Create ILI9341 device --- */
  ESP_LOGI(TAG_EX, "Initializing ILI9341 HAL...");
  /* This function is already guarded internally by the HAL C file */
  ret = pstar_ili9341_hal_create_kconfig_default(&example_bus_manager, &lcd_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to create ILI9341 default: %s", esp_err_to_name(ret));
    goto example_fail;
  }
  ESP_LOGI(TAG_EX, "ILI9341 Initialized Successfully.");

  /* --- Setup Step 5: Example Loop --- */
  ESP_LOGI(TAG_EX, "Starting example loop (drawing random rectangles)...");
  uint16_t colors[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW, COLOR_CYAN, COLOR_MAGENTA};
  int      num_colors = sizeof(colors) / sizeof(colors[0]);

  while (1) {
    /* Get current screen dimensions (might change with rotation) */
    uint16_t width  = 0;
    uint16_t height = 0;
    /* Need a way to get width/height from handle, assuming fixed for example */
    /* Or add getter functions to HAL */
#if CONFIG_PSTAR_KCONFIG_ILI9341_ROTATION == 1 || CONFIG_PSTAR_KCONFIG_ILI9341_ROTATION == 3
    width  = ILI9341_HEIGHT;
    height = ILI9341_WIDTH;
#else
    width  = ILI9341_WIDTH;
    height = ILI9341_HEIGHT;
#endif

    /* Generate random rectangle parameters */
    int16_t  x     = rand() % width;
    int16_t  y     = rand() % height;
    int16_t  w     = (rand() % (width - x)) + 1;
    int16_t  h     = (rand() % (height - y)) + 1;
    uint16_t color = colors[rand() % num_colors];

    ESP_LOGD(TAG_EX, "Drawing rect: (%d, %d) %dx%d Color: 0x%04X", x, y, w, h, color);

    /* Draw the rectangle */
    ret = pstar_ili9341_hal_fill_rect(lcd_handle, x, y, w, h, color);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "Failed to draw rectangle: %s", esp_err_to_name(ret));
      /* Optional: Add error handling or break */
    }

    vTaskDelay(pdMS_TO_TICKS(100)); /* Delay between drawing rectangles */
  }

example_fail:
  /* Cleanup resources if setup failed or loop exited unexpectedly */
  ESP_LOGE(TAG_EX, "Example failed or ending. Cleaning up and halting.");
  if (lcd_handle) {
    esp_err_t deinit_ret = pstar_ili9341_hal_deinit(lcd_handle, true); /* Turn off backlight */
    if (deinit_ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "ILI9341 HAL deinit failed: %s", esp_err_to_name(deinit_ret));
    }
  }
  if (manager_initialized) {
    esp_err_t bm_deinit_ret = pstar_bus_manager_deinit(&example_bus_manager);
    if (bm_deinit_ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "Bus Manager deinit reported errors: %s", esp_err_to_name(bm_deinit_ret));
    }
  }
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  if (validator_used) {
    ESP_LOGI(TAG_EX, "Freeing pin validator resources...");
    esp_err_t pin_val_ret = pstar_free_pin_validator();
    if (pin_val_ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "Pin validator free failed: %s", esp_err_to_name(pin_val_ret));
    }
  }
#endif
  /* Halt */
  ESP_LOGI(TAG_EX, "Example finished cleanup. Halting task.");
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}

#else /* CONFIG_PSTAR_KCONFIG_ILI9341_ENABLED */

/* Provide a stub function if the component is disabled */
#include "esp_log.h"
/* Include FreeRTOS headers needed ONLY for the halt loop in the stub */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG_EX_STUB = "ILI9341_Example";
void               example_ili9341_app_main(void)
{
  ESP_LOGE(TAG_EX_STUB,
           "ILI9341 Example is selected in Kconfig, but the ILI9341 component is disabled!");
  ESP_LOGE(
    TAG_EX_STUB,
    "Please enable 'PSTAR_KCONFIG_ILI9341_ENABLED' or select a different example/Main Application.");
  while (1) {
    vTaskDelay(portMAX_DELAY); /* Halt */
  }
}

#endif /* CONFIG_PSTAR_KCONFIG_ILI9341_ENABLED */