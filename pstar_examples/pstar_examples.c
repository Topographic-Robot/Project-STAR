/* pstar_examples/pstar_examples.c */

#include "pstar_examples.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

/* --- Include headers for ALL possible examples --- */
#ifdef CONFIG_PSTAR_EXAMPLE_BH1750
#include "bh1750_example.h"
#endif
#ifdef CONFIG_PSTAR_EXAMPLE_PCA9685
#include "pca9685_example.h"
#endif
#ifdef CONFIG_PSTAR_EXAMPLE_DHT22
#include "dht22_example.h"
#endif
#ifdef CONFIG_PSTAR_EXAMPLE_MQ135
#include "mq135_example.h"
#endif
#ifdef CONFIG_PSTAR_EXAMPLE_HD44780
#include "hd44780_example.h"
#endif
#ifdef CONFIG_PSTAR_EXAMPLE_PIN_VALIDATOR
#include "pin_validator_example.h"
#endif
#ifdef CONFIG_PSTAR_EXAMPLE_ERROR_HANDLER
#include "error_handler_example.h"
#endif
#ifdef CONFIG_PSTAR_EXAMPLE_JTAG
#include "jtag_example.h"
#endif
#ifdef CONFIG_PSTAR_EXAMPLE_MPU6050
#include "mpu6050_example.h"
#endif
#ifdef CONFIG_PSTAR_EXAMPLE_QMC5883
#include "qmc5883_example.h"
#endif
#ifdef CONFIG_PSTAR_EXAMPLE_ILI9341
#include "ili9341_example.h"
#endif
#ifdef CONFIG_PSTAR_EXAMPLE_SDMMC /* Added */
#include "sdmmc_example.h"
#endif

static const char* TAG_DISPATCH = "ExampleDispatch";

void run_selected_example(void)
{
#if CONFIG_PSTAR_EXAMPLE_BH1750
  ESP_LOGI(TAG_DISPATCH, "Dispatching to BH1750 Example");
  example_bh1750_app_main();
#elif CONFIG_PSTAR_EXAMPLE_PCA9685
  ESP_LOGI(TAG_DISPATCH, "Dispatching to PCA9685 Example");
  example_pca9685_app_main();
#elif CONFIG_PSTAR_EXAMPLE_DHT22
  ESP_LOGI(TAG_DISPATCH, "Dispatching to DHT22 Example");
  example_dht22_app_main();
#elif CONFIG_PSTAR_EXAMPLE_MQ135
  ESP_LOGI(TAG_DISPATCH, "Dispatching to MQ135 Example");
  example_mq135_app_main();
#elif CONFIG_PSTAR_EXAMPLE_HD44780
  ESP_LOGI(TAG_DISPATCH, "Dispatching to HD44780 Example");
  example_hd44780_app_main();
#elif CONFIG_PSTAR_EXAMPLE_PIN_VALIDATOR
  ESP_LOGI(TAG_DISPATCH, "Dispatching to Pin Validator Example");
  example_pin_validator_app_main();
#elif CONFIG_PSTAR_EXAMPLE_ERROR_HANDLER
  ESP_LOGI(TAG_DISPATCH, "Dispatching to Error Handler Example");
  example_error_handler_app_main();
#elif CONFIG_PSTAR_EXAMPLE_JTAG
  ESP_LOGI(TAG_DISPATCH, "Dispatching to JTAG Example");
  example_jtag_app_main();
#elif CONFIG_PSTAR_EXAMPLE_MPU6050
  ESP_LOGI(TAG_DISPATCH, "Dispatching to MPU6050 Example");
  example_mpu6050_app_main();
#elif CONFIG_PSTAR_EXAMPLE_QMC5883
  ESP_LOGI(TAG_DISPATCH, "Dispatching to QMC5883 Example");
  example_qmc5883_app_main();
#elif CONFIG_PSTAR_EXAMPLE_ILI9341
  ESP_LOGI(TAG_DISPATCH, "Dispatching to ILI9341 Example");
  example_ili9341_app_main();
#elif CONFIG_PSTAR_EXAMPLE_SDMMC /* Added */
  ESP_LOGI(TAG_DISPATCH, "Dispatching to SD Card (SDMMC/SPI) Example");
  example_sdmmc_app_main();
#elif CONFIG_PSTAR_EXAMPLE_NONE
  ESP_LOGE(TAG_DISPATCH, "Error: run_selected_example called when Main Application selected!");
#else
#error "An example is selected in Kconfig but not handled in pstar_examples.c!"
  ESP_LOGE(TAG_DISPATCH, "Error: Unknown example selected in Kconfig!");
#endif

  ESP_LOGW(TAG_DISPATCH, "Warning: Selected example function returned unexpectedly.");
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}