/* main/main.c - Simple SD Card Write Example */

#include "pstar_error_handler.h"
#include "pstar_log_handler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "nvs_flash.h"

#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
#include "pstar_pin_validator.h"
#endif

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
#include "pstar_sd_card_hal.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#endif

/* Constants ******************************************************************/
static const char* TAG = "SD Card Example"; /**< Tag for logging */

/* Global variables ***********************************************************/
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
static sd_card_hal_t g_sd_card; /**< Global SD card HAL instance */
#endif
static error_handler_t g_error_handler; /**< Global error handler */

void app_main(void)
{
  printf("Simple SD Card Write Example\n");

  /* Initialize the error handler */
  esp_err_t err = error_handler_init(&g_error_handler,
                                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_MAX_RETRIES,
                                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_RETRY_DELAY_MS,
                                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_MAX_DELAY_MS,
                                     NULL, /* No reset function */
                                     NULL);
  if (err != ESP_OK) {
    printf("Failed to initialize error handler: %s\n", esp_err_to_name(err));
    return;
  }

  /* Initialize minimal logging */
  err = log_init(NULL, NULL);
  if (err != ESP_OK) {
    printf("Failed to initialize logging: %s\n", esp_err_to_name(err));
    error_handler_deinit(&g_error_handler);
    return;
  }

  /* Initialize NVS flash */
  log_info(TAG, "Init", "Initializing NVS flash...");
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    log_warn(TAG, "NVS Warning", "NVS requires erase. Erasing...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    log_error(TAG, "Error", "Failed to initialize NVS: %s", esp_err_to_name(err));
    goto cleanup;
  }

#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  /* Initialize pin validator */
  log_info(TAG, "Init", "Initializing pin validator...");
  err = pin_validator_init();
  if (err != ESP_OK) {
    log_error(TAG, "Error", "Failed to initialize pin validator: %s", esp_err_to_name(err));
    goto cleanup;
  }
#endif

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  /* Initialize SD card */
  log_info(TAG, "Init", "Initializing SD Card HAL...");

  /* Select 1-bit bus width for simplicity */
  sd_bus_width_t bus_width = k_sd_bus_width_1bit;

  err = sd_card_init_default(&g_sd_card,
                             "SD Card",
                             CONFIG_PSTAR_KCONFIG_SD_CARD_MOUNT_POINT,
                             "sd_card_example",
                             bus_width);
  if (err != ESP_OK) {
    log_error(TAG, "Error", "Failed to initialize SD card HAL: %s", esp_err_to_name(err));
    goto cleanup;
  }

  /* Configure SD card task */
  err = sd_card_set_task_config(&g_sd_card,
                                CONFIG_PSTAR_KCONFIG_SD_CARD_TASK_STACK_SIZE,
                                CONFIG_PSTAR_KCONFIG_SD_CARD_TASK_PRIORITY,
                                pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_SD_CARD_MUTEX_TIMEOUT_MS));
  if (err != ESP_OK) {
    log_error(TAG, "Error", "Failed to set SD card task config: %s", esp_err_to_name(err));
    goto cleanup;
  }

  /* Initialize SD card hardware */
  log_info(TAG, "Init", "Initializing SD Card hardware...");
  err = sd_card_init(&g_sd_card);
  if (err != ESP_OK) {
    log_error(TAG, "Error", "Failed to initialize SD card hardware: %s", esp_err_to_name(err));
    goto cleanup;
  }
#endif

#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  /* Now print pin assignments AFTER SD card HAL has registered its pins */
  log_info(TAG, "Pin Table", "Printing pin assignment table...");
  err = pin_validator_print_assignments();
  if (err != ESP_OK) {
    log_error(TAG, "Warning", "Failed to print pin assignments: %s", esp_err_to_name(err));
  }

  /* Validate pins */
  log_info(TAG, "Init", "Validating pins...");
  err = pin_validator_validate_all(false);
  if (err != ESP_OK) {
    log_error(TAG, "Warning", "Pin validation failed: %s", esp_err_to_name(err));
    /* Continue anyway */
  }
#endif

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  /* Check if SD card is available - no delay, just check directly */
  log_info(TAG, "SD Card", "Checking if SD card is available...");

  /* Poll for SD card availability with short intervals, up to 10 attempts */
  bool card_available = false;
  for (int i = 0; i < 10; i++) {
    card_available = sd_card_is_available(&g_sd_card);
    if (card_available) {
      log_info(TAG, "SD Card", "SD card is available after %d checks", i + 1);
      break;
    }
    log_info(TAG, "SD Card", "SD card not ready, checking again (%d/10)...", i + 1);
    vTaskDelay(pdMS_TO_TICKS(1000)); /* Short 1000ms delay between checks */
  }

  if (!card_available) {
    log_error(TAG,
              "Error",
              "SD card is not available after multiple checks. Please insert a card and restart.");
    goto cleanup;
  }

  /* Write "hello world" to a file using standard POSIX file API */
  char filepath[64];
  snprintf(filepath, sizeof(filepath), "%s/hello.txt", CONFIG_PSTAR_KCONFIG_SD_CARD_MOUNT_POINT);
  log_info(TAG, "File Write", "Writing 'hello world' to %s", filepath);

  /* Simple direct file write */
  FILE* f = fopen(filepath, "w");
  if (f == NULL) {
    log_error(TAG, "Error", "Failed to open file for writing: %s", strerror(errno));
    goto cleanup;
  }

  const char* message = "hello world";
  fprintf(f, "%s", message);
  fclose(f);

  log_info(TAG, "Success", "Successfully wrote 'hello world' to %s", filepath);
#else
  log_error(TAG,
            "Error",
            "SD card support is not enabled. Enable CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED.");
#endif

cleanup:
  log_info(TAG, "Cleanup", "Cleaning up resources...");

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  /* Clean up SD card */
  if (g_sd_card.initialized) {
    sd_card_cleanup(&g_sd_card);
  }
#endif

#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  /* Clean up pin validator */
  pin_validator_clear_all();
#endif

  /* Final log flush and cleanup */
  log_flush();
  log_cleanup();

  /* Clean up error handler */
  error_handler_deinit(&g_error_handler);

  printf("SD Card example completed\n");
}