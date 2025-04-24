/* main/main.c */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

/* --- Conditional Includes --- */

#ifndef CONFIG_PSTAR_EXAMPLE_NONE /* If building the main application */
#include "pstar_examples.h"
#endif /* CONFIG_PSTAR_EXAMPLE_NONE */

static const char* TAG = "Project Star";

/* --- MAIN APPLICATION LOGIC (Only compiled if selected in Kconfig) --- */
#ifdef CONFIG_PSTAR_EXAMPLE_NONE

/**
 * @brief Runs the simplified main Project-STAR application logic.
 * Currently, just prints a startup message and loops.
 */
void run_main_application(void)
{
  ESP_LOGI(TAG, "Running: Main Application (Placeholder)");

  /*
   * NOTE: Main application logic is currently bypassed in favor of examples.
   * Implement actual robot setup, task creation, and control loops here
   * when developing the core firmware functionality.
   * Remember to initialize necessary components (Bus Manager, HALs)
   * and handle pin validation if required by the main application.
   */

  while (1) {
    ESP_LOGD(TAG, "Main application loop tick (placeholder)...");
    vTaskDelay(pdMS_TO_TICKS(10000)); /* Prevent busy-looping */
  }

  /* Cleanup code (if any) would go here, but the loop is infinite */
}
#endif /* CONFIG_PSTAR_EXAMPLE_NONE */

/* --- PRIMARY ENTRY POINT --- */
void app_main(void)
{
  ESP_LOGI(TAG, "*** Project Star Firmware Starting ***");

#if CONFIG_PSTAR_EXAMPLE_NONE
  /* Run the actual main application */
  run_main_application();
#else
  /* Run the selected example via the dispatcher */
  run_selected_example();
#endif

  /* This point should ideally not be reached */
  ESP_LOGW(TAG, "app_main has unexpectedly finished.");
  /* Halt */
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}