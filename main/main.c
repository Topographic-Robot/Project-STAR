/* main/main.c */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h" /* Needed for default event loop */
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

/* Project Star Includes */
#include "pstar_bus_manager.h"
#include "pstar_error_handler.h"      /* Though not directly used here, SD HAL uses it */
#include "pstar_file_write_manager.h" /* Needed for SD logging */
#include "pstar_log_handler.h"
#include "pstar_pin_validator.h"
#include "pstar_time_manager.h" /* For logger timestamps */

/* Storage HAL Includes */
#include "pstar_sd_card_hal.h"    /* Includes types and main HAL functions */
#include "pstar_storage_common.h" /* For storage_bus_width_to_string */

/* Kconfig */
#include "sdkconfig.h"

/* Static Variables */
static const char* TAG = "AppMain";
// static pstar_bus_manager_t g_bus_manager; // Bus manager is now internal to SD HAL
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
static sd_card_hal_t g_sd_card;
#endif
#if defined(CONFIG_PSTAR_KCONFIG_FILE_MANAGER_ENABLED) &&                                          \
  defined(CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED)
static file_write_manager_t g_file_manager;
#endif
static bool g_sd_logging_enabled = false; /* Track if SD logging is active */

void app_main(void)
{
  esp_err_t ret;

  /* --- Early Initialization --- */

  /* Initialize NVS Flash */
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  /* Create default event loop */
  ESP_ERROR_CHECK(esp_event_loop_create_default());

/* Initialize Logger (Minimal - Console Only Initially) */
#if CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED
  ret = log_init(NULL, NULL); /* Minimal init */
#else
  ret = log_init(NULL, NULL); /* Only init possible */
#endif
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Minimal Logger initialization failed: %s", esp_err_to_name(ret));
    /* Continue without logging? */
  }

  log_info(TAG, "Init", "Starting Project-Star Application");

/* Initialize Pin Validator */
#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  ret = pin_validator_init();
  if (ret != ESP_OK) {
    log_error(TAG, "Init Error", "Pin Validator initialization failed: %s", esp_err_to_name(ret));
    return; /* Critical for hardware safety */
  }
#endif

/* Initialize Bus Manager - No longer needed globally if only used by SD HAL */
// ret = pstar_bus_manager_init(&g_bus_manager, "MainBusMgr");
// if (ret != ESP_OK) {
//     log_error(TAG, "Init Error", "Bus Manager initialization failed: %s", esp_err_to_name(ret));
//     return;
// }

/* Initialize Time Manager (for logger timestamps) */
#ifdef CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
  ret = time_manager_init();
  if (ret != ESP_OK) {
    log_error(TAG, "Init Error", "Time Manager initialization failed: %s", esp_err_to_name(ret));
    /* Continue, logging might lack timestamps */
  }
  log_info(TAG, "Init", "Waiting briefly for time sync...");
  vTaskDelay(pdMS_TO_TICKS(2000)); // Allow time for sync
#endif

/* --- SD Card HAL Initialization --- */
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  log_info(TAG, "SD Init", "Initializing SD Card HAL...");

  // Determine initial bus width based on Kconfig (primarily for SDIO preference)
  sd_bus_width_t init_bus_width = k_sd_bus_width_1bit; // Default to 1-bit
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_4BIT_MODE
  init_bus_width = k_sd_bus_width_4bit;
#endif

  // Initialize the HAL structure with default settings (pins from Kconfig)
  // sd_card_init_default handles internal bus manager init
  ret = sd_card_init_default(&g_sd_card,
                             "SD_HAL",
                             CONFIG_PSTAR_KCONFIG_SD_CARD_MOUNT_POINT,
                             "AppMainSD",     // Component ID for error handler
                             init_bus_width); // Pass desired width

  if (ret != ESP_OK) {
    log_error(TAG,
              "SD Init Error",
              "SD Card HAL default initialization failed: %s",
              esp_err_to_name(ret));
  } else {
    // Start the SD card detection and mounting task
    ret = sd_card_init(&g_sd_card);
    if (ret != ESP_OK) {
      log_error(TAG,
                "SD Init Error",
                "SD Card HAL task initialization failed: %s",
                esp_err_to_name(ret));
    } else {
      log_info(TAG, "SD Init", "SD Card HAL initialized. Waiting for mount task...");
      // Give the task some time to attempt mounting
      vTaskDelay(pdMS_TO_TICKS(3000)); // Adjust delay as needed

      if (sd_card_is_available(&g_sd_card)) {
        log_info(TAG,
                 "SD Mount",
                 "SD card mounted successfully at %s (Interface: %s, Width: %s)",
                 g_sd_card.mount_path,
                 sd_card_interface_to_string(sd_card_get_current_interface(&g_sd_card)),
                 storage_bus_width_to_string(sd_card_get_bus_width(&g_sd_card)));

/* --- Initialize File Manager and Full Logger (ONLY if SD mounted) --- */
#if defined(CONFIG_PSTAR_KCONFIG_FILE_MANAGER_ENABLED) &&                                          \
  defined(CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED)
        log_info(TAG, "FM Init", "Initializing File Write Manager...");
        // Pass the initialized SD HAL instance
        ret = file_write_manager_init(&g_file_manager, &g_sd_card, NULL);
        if (ret != ESP_OK) {
          log_error(TAG,
                    "FM Init Error",
                    "File Write Manager initialization failed: %s",
                    esp_err_to_name(ret));
        } else {
          log_info(TAG, "Logger Full Init", "Performing full logger initialization for SD card...");
          // Call log_init AGAIN with valid pointers
          ret = log_init(&g_file_manager, &g_sd_card);
          if (ret != ESP_OK) {
            log_error(TAG,
                      "Logger Full Init Error",
                      "Full logger initialization failed: %s",
                      esp_err_to_name(ret));
          } else {
            g_sd_logging_enabled = true;
            log_info(TAG,
                     "Logger Ready",
                     "Logger fully initialized. SD Card logging is now ACTIVE.");
            log_warn(TAG, "SD Logging Test", "This message should be logged to the SD card!");
          }
        }
#else
        log_warn(TAG,
                 "SD Logging Skip",
                 "File Manager or SD Logging disabled in Kconfig. Logs will only go to console.");
#endif

      } else {
        log_error(TAG, "SD Mount Error", "SD card failed to mount after initialization.");
        // SD card is not available, proceed without SD features
      }
    }
  }
#else
  log_warn(TAG, "SD Init Skip", "SD Card support disabled in Kconfig.");
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED */

/* --- Post-Initialization Validation --- */
#if defined(CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED) &&                                         \
  defined(CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_VALIDATE_AT_STARTUP)
  log_info(TAG, "Validation", "Running pin validation...");
  bool halt_on_conflict = false;
#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_HALT_ON_CONFLICT
  halt_on_conflict = true;
#endif
  ret = pin_validator_validate_all(halt_on_conflict);
  if (ret != ESP_OK) {
    log_error(TAG, "Validation Error", "Pin validation failed!");
    /* Halt is handled internally if configured */
  }
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_PRINT_ASSIGNMENTS
  pin_validator_print_assignments();
#endif
#endif

  log_info(TAG, "Init Complete", "Initialization sequence finished.");

  /* --- Application Logic --- */
  log_info(TAG, "App Start", "Entering main loop...");
  int counter = 0;
  while (1) {
    log_info(TAG, "Heartbeat", "Main loop running... Counter: %d", counter++);

    if (g_sd_logging_enabled) {
      log_debug(TAG, "Log Flush", "Attempting to flush logs to SD card...");
      ret = log_flush();
      if (ret != ESP_OK) {
        log_warn(TAG, "Log Flush Error", "Log flush failed: %s", esp_err_to_name(ret));
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10000));
  }

  /* --- Cleanup (won't be reached in this example loop) --- */
  // log_info(TAG, "Cleanup", "Starting cleanup...");
  // #if defined(CONFIG_PSTAR_KCONFIG_FILE_MANAGER_ENABLED) && defined(CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED)
  //     if (g_sd_logging_enabled) { file_write_manager_cleanup(&g_file_manager); }
  // #endif
  // #ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  //     sd_card_cleanup(&g_sd_card); // Use generic cleanup
  // #endif
  // #ifdef CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
  //     time_manager_cleanup();
  // #endif
  // // pstar_bus_manager_deinit(&g_bus_manager); // No longer global
  // log_cleanup();
  // ESP_ERROR_CHECK(esp_event_loop_delete_default());
  // ESP_ERROR_CHECK(nvs_flash_deinit());
  // log_info(TAG, "Cleanup", "Cleanup finished.");
}
