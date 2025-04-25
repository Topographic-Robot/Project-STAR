/* pstar_examples/error_handler_example.c */

#include "sdkconfig.h"

/* Only compile if Error Handler component is enabled */
#ifdef CONFIG_PSTAR_KCONFIG_ERROR_HANDLER_ENABLED

#include "error_handler_example.h"
#include "pstar_error_handler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h" /* For ESP_ERROR_CHECK */
#include "esp_log.h"

static const char* TAG_EX = "ErrorHandler_Example";

/* --- Dummy Reset Function --- */
static esp_err_t dummy_reset_function(void* context)
{
  int* reset_counter = (int*)context;
  (*reset_counter)++;
  ESP_LOGW(TAG_EX, "Dummy Reset Function Called! (Count: %d)", *reset_counter);
  /* Simulate reset success sometimes */
  if ((*reset_counter % 2) == 0) {
    ESP_LOGW(TAG_EX, "Simulating reset SUCCESS.");
    return ESP_OK;
  } else {
    ESP_LOGW(TAG_EX, "Simulating reset FAILURE.");
    return ESP_FAIL;
  }
}

void example_error_handler_app_main(void)
{
  ESP_LOGI(TAG_EX, "Running: Error Handler Example");
  esp_err_t       ret;
  error_handler_t my_error_handler;
  int             reset_call_count = 0;

  /* Configuration for the error handler */
  uint32_t max_retries      = 3;
  uint32_t base_retry_delay = 200;  /* ms */
  uint32_t max_retry_delay  = 1000; /* ms */

  ESP_LOGI(TAG_EX,
           "Initializing error handler: Max Retries=%lu, Base Delay=%lums, Max Delay=%lums",
           (unsigned long)max_retries,
           (unsigned long)base_retry_delay,
           (unsigned long)max_retry_delay);

  /* Initialize the error handler */
  ret = error_handler_init(&my_error_handler,
                           max_retries,
                           base_retry_delay,
                           max_retry_delay,
                           dummy_reset_function,
                           &reset_call_count); /* Pass counter address as context */
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to initialize error handler: %s", esp_err_to_name(ret));
    goto example_halt;
  }
  ESP_LOGI(TAG_EX, "Error handler initialized successfully.");

  /* Simulate some operations and errors */
  for (int i = 0; i < 6; i++) {
    ESP_LOGI(TAG_EX, "\n--- Simulation Step %d ---", i + 1);

    /* Simulate an error */
    esp_err_t simulated_error = ESP_FAIL; /* Generic failure */
    if ((i % 3) == 1) {
      simulated_error = ESP_ERR_TIMEOUT;
    } else if ((i % 3) == 2) {
      simulated_error = ESP_ERR_NO_MEM;
    }

    ESP_LOGW(TAG_EX, "Simulating an error: %s", esp_err_to_name(simulated_error));

    /* Record the error using the macro (simpler) or the function */
    // ret = error_handler_record_error(&my_error_handler, simulated_error, "Simulated failure", __FILE__, __LINE__, __func__);
    RECORD_ERROR(&my_error_handler, simulated_error, "Simulated failure in loop");

    /* Check if we can retry */
    if (error_handler_can_retry(&my_error_handler)) {
      uint32_t delay_ms = my_error_handler.current_retry_delay;
      ESP_LOGI(TAG_EX,
               "Retry possible. Waiting for %lums before next attempt...",
               (unsigned long)delay_ms);
      vTaskDelay(pdMS_TO_TICKS(delay_ms));
    } else {
      ESP_LOGE(TAG_EX, "Retry not possible or max retries reached.");
      /* Check if the error handler managed to recover via reset */
      if (my_error_handler.last_error == ESP_OK) {
        ESP_LOGI(TAG_EX, "Error handler recovered via reset function.");
        /* If reset worked, the state is reset, loop can continue as if starting fresh */
      } else {
        ESP_LOGE(TAG_EX, "Error handler failed to recover. Example stopping simulation.");
        break; /* Exit the loop if retries exhausted and reset failed */
      }
    }

    /* Simulate successful operation sometimes */
    if (i == 2) {
      ESP_LOGI(TAG_EX, "Simulating successful operation after errors.");
      ret = error_handler_reset_state(&my_error_handler);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG_EX, "Failed to reset error state: %s", esp_err_to_name(ret));
      } else {
        ESP_LOGI(TAG_EX, "Error handler state reset manually after success.");
      }
    }
  } /* End of simulation loop */

  ESP_LOGI(TAG_EX, "\n--- Simulation Finished ---");
  ESP_LOGI(TAG_EX, "Final Error Handler State:");
  ESP_LOGI(TAG_EX, "  Error Count: %lu", (unsigned long)my_error_handler.error_count);
  ESP_LOGI(TAG_EX, "  Current Retry: %lu", (unsigned long)my_error_handler.current_retry);
  ESP_LOGI(TAG_EX, "  In Error State: %s", my_error_handler.in_error_state ? "Yes" : "No");
  ESP_LOGI(TAG_EX, "  Last Error: %s", esp_err_to_name(my_error_handler.last_error));
  ESP_LOGI(TAG_EX, "  Reset Function Calls: %d", reset_call_count);

  /* Deinitialize the error handler */
  ESP_LOGI(TAG_EX, "Deinitializing error handler...");
  error_handler_deinit(&my_error_handler);

example_halt:
  ESP_LOGI(TAG_EX, "Example finished or halted. Looping forever...");
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}

#else /* CONFIG_PSTAR_KCONFIG_ERROR_HANDLER_ENABLED */

/* Provide a stub function if the component is disabled */
#include "esp_log.h"
/* Include FreeRTOS headers needed ONLY for the halt loop in the stub */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG_EX_STUB = "ErrorHandler_Example";
void               example_error_handler_app_main(void)
{
  ESP_LOGE(
    TAG_EX_STUB,
    "Error Handler Example is selected in Kconfig, but the Error Handler component is disabled!");
  ESP_LOGE(
    TAG_EX_STUB,
    "Please enable 'PSTAR_KCONFIG_ERROR_HANDLER_ENABLED' or select a different example/Main Application.");
  while (1) {
    vTaskDelay(portMAX_DELAY); /* Halt */
  }
}

#endif /* CONFIG_PSTAR_KCONFIG_ERROR_HANDLER_ENABLED */
