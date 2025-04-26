/* pstar_examples/qmc5883_example.c */

#include "sdkconfig.h"

/* Only compile if QMC5883 component is enabled */
#ifdef CONFIG_PSTAR_KCONFIG_QMC5883_ENABLED

#include "pstar_bus_manager.h"
#include "pstar_pin_validator.h"
#include "pstar_qmc5883_hal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h" /* For ESP_ERROR_CHECK */
#include "esp_log.h"
#include "qmc5883_example.h"

static const char* TAG_EX = "QMC5883_Example";

void example_qmc5883_app_main(void)
{
  ESP_LOGI(TAG_EX, "Running: QMC5883 Example");
  esp_err_t                  ret = ESP_OK;
  pstar_bus_manager_t        example_bus_manager;
  pstar_qmc5883_hal_handle_t qmc5883_handle      = NULL;
  bool                       manager_initialized = false;
  bool                       validator_used      = false; /* Track if validator was active */

/* --- Setup Step 1: Register Example Pins (if validator enabled) --- */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  validator_used = true;
  ESP_LOGI(TAG_EX, "Registering QMC5883 example pins...");
  /* Use Kconfig pins for the example */
  ret = pstar_qmc5883_register_kconfig_pins();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to register QMC5883 pins: %s", esp_err_to_name(ret));
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
  ret = pstar_bus_manager_init(&example_bus_manager, "QMC5883_Example_BusMgr");
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to init example bus manager: %s", esp_err_to_name(ret));
    goto example_fail; /* Cannot proceed */
  }
  manager_initialized = true;

  /* --- Setup Step 4: Create QMC5883 device --- */
  ESP_LOGI(TAG_EX, "Initializing QMC5883 HAL...");
  /* This function is already guarded internally by the HAL C file */
  ret = pstar_qmc5883_hal_create_kconfig_default(&example_bus_manager, &qmc5883_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to create QMC5883 default: %s", esp_err_to_name(ret));
    goto example_fail;
  }
  ESP_LOGI(TAG_EX, "QMC5883 Initialized Successfully.");

  /* --- Setup Step 5: Example Loop --- */
  ESP_LOGI(TAG_EX, "Starting example loop...");
  qmc5883_data_t mag_data;
  int16_t        raw_temp;

  while (1) {
    bool data_ready = false;
    ret             = pstar_qmc5883_hal_is_data_ready(qmc5883_handle, &data_ready);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "Failed to check data ready status: %s", esp_err_to_name(ret));
    }

    if (data_ready) {
      /* Read Magnetometer Data */
      ret = pstar_qmc5883_hal_read_data(qmc5883_handle, &mag_data);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG_EX, "Failed to read magnetometer data: %s", esp_err_to_name(ret));
      }

      /* Read Temperature Data (Optional) */
      ret = pstar_qmc5883_hal_read_raw_temp(qmc5883_handle, &raw_temp);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG_EX, "Failed to read temperature data: %s", esp_err_to_name(ret));
        raw_temp = INT16_MIN; /* Indicate error */
      }

      /* Print results */
      ESP_LOGI(TAG_EX,
               "Mag [Gauss]: X=%.3f Y=%.3f Z=%.3f | Raw Temp: %d",
               mag_data.x,
               mag_data.y,
               mag_data.z,
               raw_temp);
    } else {
      ESP_LOGD(TAG_EX, "Data not ready, skipping read.");
    }

    /* Delay based roughly on ODR, but add some margin */
    /* This simple delay isn't perfect synchronization */
    vTaskDelay(pdMS_TO_TICKS(100)); /* Adjust delay as needed */
  }

example_fail:
  /* Cleanup resources if setup failed or loop exited unexpectedly */
  ESP_LOGE(TAG_EX, "Example failed or ending. Cleaning up and halting.");
  if (qmc5883_handle) {
    esp_err_t deinit_ret = pstar_qmc5883_hal_deinit(qmc5883_handle, true); /* Put to standby */
    if (deinit_ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "QMC5883 HAL deinit failed: %s", esp_err_to_name(deinit_ret));
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

#else /* CONFIG_PSTAR_KCONFIG_QMC5883_ENABLED */

/* Provide a stub function if the component is disabled */
#include "esp_log.h"
/* Include FreeRTOS headers needed ONLY for the halt loop in the stub */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG_EX_STUB = "QMC5883_Example";
void               example_qmc5883_app_main(void)
{
  ESP_LOGE(TAG_EX_STUB,
           "QMC5883 Example is selected in Kconfig, but the QMC5883 component is disabled!");
  ESP_LOGE(
    TAG_EX_STUB,
    "Please enable 'PSTAR_KCONFIG_QMC5883_ENABLED' or select a different example/Main Application.");
  while (1) {
    vTaskDelay(portMAX_DELAY); /* Halt */
  }
}

#endif /* CONFIG_PSTAR_KCONFIG_QMC5883_ENABLED */