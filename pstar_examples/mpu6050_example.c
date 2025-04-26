/* pstar_examples/mpu6050_example.c */

#include "sdkconfig.h"

/* Only compile if MPU6050 component is enabled */
#ifdef CONFIG_PSTAR_KCONFIG_MPU6050_ENABLED

#include "pstar_bus_manager.h"
#include "pstar_mpu6050_hal.h"
#include "pstar_pin_validator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h" /* For ESP_ERROR_CHECK */
#include "esp_log.h"
#include "mpu6050_example.h"

static const char* TAG_EX = "MPU6050_Example";

void example_mpu6050_app_main(void)
{
  ESP_LOGI(TAG_EX, "Running: MPU6050 Example");
  esp_err_t                  ret = ESP_OK;
  pstar_bus_manager_t        example_bus_manager;
  pstar_mpu6050_hal_handle_t mpu6050_handle      = NULL;
  bool                       manager_initialized = false;
  bool                       validator_used      = false; /* Track if validator was active */

/* --- Setup Step 1: Register Example Pins (if validator enabled) --- */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  validator_used = true;
  ESP_LOGI(TAG_EX, "Registering MPU6050 example pins...");
  /* Use Kconfig pins for the example */
  ret = pstar_mpu6050_register_kconfig_pins();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to register MPU6050 pins: %s", esp_err_to_name(ret));
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
  ret = pstar_bus_manager_init(&example_bus_manager, "MPU6050_Example_BusMgr");
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to init example bus manager: %s", esp_err_to_name(ret));
    goto example_fail; /* Cannot proceed */
  }
  manager_initialized = true;

  /* --- Setup Step 4: Create MPU6050 device --- */
  ESP_LOGI(TAG_EX, "Initializing MPU6050 HAL...");
  /* This function is already guarded internally by the HAL C file */
  ret = pstar_mpu6050_hal_create_kconfig_default(&example_bus_manager, &mpu6050_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to create MPU6050 default: %s", esp_err_to_name(ret));
    goto example_fail;
  }
  ESP_LOGI(TAG_EX, "MPU6050 Initialized Successfully.");

  /* --- Setup Step 5: Example Loop --- */
  ESP_LOGI(TAG_EX, "Starting example loop...");
  mpu6050_accel_t accel_data;
  mpu6050_gyro_t  gyro_data;
  float           temp_c;

  while (1) {
    /* Read Accelerometer */
    ret = pstar_mpu6050_hal_read_accel(mpu6050_handle, &accel_data);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "Failed to read accelerometer: %s", esp_err_to_name(ret));
    }

    /* Read Gyroscope */
    ret = pstar_mpu6050_hal_read_gyro(mpu6050_handle, &gyro_data);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "Failed to read gyroscope: %s", esp_err_to_name(ret));
    }

    /* Read Temperature */
    ret = pstar_mpu6050_hal_read_temp(mpu6050_handle, &temp_c);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "Failed to read temperature: %s", esp_err_to_name(ret));
    }

    /* Print results */
    if (ret == ESP_OK) { /* Only print if last read (temp) was okay */
      ESP_LOGI(TAG_EX,
               "Accel[g]: X=%.2f Y=%.2f Z=%.2f | Gyro[dps]: X=%.2f Y=%.2f Z=%.2f | Temp[C]: %.2f",
               accel_data.x,
               accel_data.y,
               accel_data.z,
               gyro_data.x,
               gyro_data.y,
               gyro_data.z,
               temp_c);
    }

    vTaskDelay(pdMS_TO_TICKS(500)); /* Delay between readings */
  }

example_fail:
  /* Cleanup resources if setup failed or loop exited unexpectedly */
  ESP_LOGE(TAG_EX, "Example failed or ending. Cleaning up and halting.");
  if (mpu6050_handle) {
    esp_err_t deinit_ret = pstar_mpu6050_hal_deinit(mpu6050_handle, true); /* Put to sleep */
    if (deinit_ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "MPU6050 HAL deinit failed: %s", esp_err_to_name(deinit_ret));
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

#else /* CONFIG_PSTAR_KCONFIG_MPU6050_ENABLED */

/* Provide a stub function if the component is disabled */
#include "esp_log.h"
/* Include FreeRTOS headers needed ONLY for the halt loop in the stub */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG_EX_STUB = "MPU6050_Example";
void               example_mpu6050_app_main(void)
{
  ESP_LOGE(TAG_EX_STUB,
           "MPU6050 Example is selected in Kconfig, but the MPU6050 component is disabled!");
  ESP_LOGE(
    TAG_EX_STUB,
    "Please enable 'PSTAR_KCONFIG_MPU6050_ENABLED' or select a different example/Main Application.");
  while (1) {
    vTaskDelay(portMAX_DELAY); /* Halt */
  }
}

#endif /* CONFIG_PSTAR_KCONFIG_MPU6050_ENABLED */