/* main/main.c */

#include "pstar_error_handler.h"
#include "pstar_log_handler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_system.h"
#include "sdkconfig.h" // Must be included for Kconfig options

/* --- Add ESP-IDF Network/Event/NVS/Sleep includes --- */
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sleep.h" // Include for deep sleep
#include "nvs_flash.h"
/* --- End Add --- */

#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
#include "pstar_pin_validator.h" // Include pin validator header
#endif

#ifdef CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
#include "pstar_time_manager.h"
#endif

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
#include "pstar_file_write_manager.h"
#include "pstar_sd_card_hal.h"
#endif

/* Constants ******************************************************************/

static const char* TAG = "Project Star"; /**< Tag for logging */
// #define DEEP_SLEEP_WAKEUP_SEC (10) // No longer needed for indefinite sleep

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
static sd_card_hal_t        g_sd_card;      /**< Global static SD card HAL instance */
static file_write_manager_t g_file_manager; /**< Global static file manager instance */
#endif
static error_handler_t g_main_error_handler;      /**< Global static main error handler */
static error_handler_t g_heartbeat_error_handler; /**< Global static heartbeat error handler */

/* Function Prototypes ********************************************************/

static void app_cleanup(void);
static void app_init(void);
static void app_halt(const char* reason);       // Keep declaration for potential future use
static void app_deep_sleep(const char* reason); // Function for deep sleep

void app_main(void)
{
  app_init(); /* Initialize components and error handlers */

  /* --- Main Application Logic Goes Here --- */
  /* Setup recovery error handler for the heartbeat loop */
  esp_err_t err = error_handler_init(&g_heartbeat_error_handler,
                                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_MAX_RETRIES,
                                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_RETRY_DELAY_MS,
                                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_MAX_DELAY_MS,
                                     NULL, /* No reset function for heartbeat errors */
                                     NULL);
  if (err != ESP_OK) {
    RECORD_ERROR(&g_main_error_handler, err, "Failed to initialize heartbeat error handler");
    log_error(TAG,
              "Heartbeat Error",
              "Heartbeat Error Handler Init Failed: %s",
              esp_err_to_name(err));
  }

  /* Example: Log a message periodically */
  int count = 0;
  while (1) {
    log_info(TAG, "Heartbeat", "System running... Count: %d", count++);

    vTaskDelay(pdMS_TO_TICKS(5000)); /* Delay 5 seconds */

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
    /* Example file write with error handling */
    bool fm_init = atomic_load(&g_file_manager.initialized);
    bool sd_init = atomic_load(&g_sd_card.initialized);

    if (fm_init && sd_init) {
      if (sd_card_is_available(&g_sd_card)) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "Log entry number %d", count);

        esp_err_t write_err = file_write_enqueue(&g_file_manager, "test.log", buffer);
        if (write_err != ESP_OK) {
          RECORD_ERROR(&g_heartbeat_error_handler, write_err, "Failed to enqueue file write");
          log_warn(TAG,
                   "File Write Failed",
                   "Failed to enqueue log entry: %s",
                   esp_err_to_name(write_err));

          if (!error_handler_can_retry(&g_heartbeat_error_handler)) {
            log_warn(TAG,
                     "Error Recovery",
                     "Max retries reached for file writes, trying to flush logs");
            esp_err_t flush_err = log_flush();
            if (flush_err != ESP_OK) {
              log_error(TAG,
                        "Recovery Error",
                        "Log flush failed during error recovery: %s",
                        esp_err_to_name(flush_err));
            }
            error_handler_reset_state(&g_heartbeat_error_handler);
          }
        } else {
          if (g_heartbeat_error_handler.in_error_state) {
            error_handler_reset_state(&g_heartbeat_error_handler);
          }
        }
      } else {
        log_warn(TAG, "File Write Skip", "SD card not available, skipping write.");
        RECORD_ERROR(&g_heartbeat_error_handler,
                     ESP_ERR_NOT_FOUND,
                     "SD card unavailable for writing");
      }
    } else {
      static bool init_warning_logged = false;
      if (!init_warning_logged) {
        log_warn(TAG,
                 "File Write Skip",
                 "SD Card HAL or File Manager not initialized, skipping write.");
        init_warning_logged = true;
      }
    }
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED */

    if (count > 0 && count % 10 == 0) {
      if (g_heartbeat_error_handler.in_error_state) {
        log_info(TAG, "Error Reset", "Resetting heartbeat error state after %d iterations.", count);
        error_handler_reset_state(&g_heartbeat_error_handler);
      }
    }
  } /* End while(1) */

  app_cleanup(); // This line might not be reached in a typical embedded loop
}

/**
 * @brief Halts the system by entering an infinite loop.
 *        Logs the reason before halting.
 * @param reason Short description of why the system is halting.
 */
static void app_halt(const char* reason)
{
  log_error(TAG, "SYSTEM HALTED", "Reason: %s", reason);
  log_flush(); // Attempt final flush
  // Optional: Add code here to signal hardware failure (e.g., LED pattern)

  // Loop indefinitely, allowing WDT to be fed by idle task
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to allow other tasks (like idle) to run
  }
}

/**
 * @brief Logs the reason and enters deep sleep indefinitely.
 * @param reason Short description of why the system is sleeping.
 */
static void app_deep_sleep(const char* reason)
{
  log_error(TAG, "ENTERING INDEFINITE DEEP SLEEP", "Reason: %s", reason); // Updated log message
  log_flush(); // Attempt final flush before sleeping

  // --- DO NOT ENABLE ANY WAKEUP SOURCES ---
  // Ensure no wakeup sources (timer, GPIO, etc.) are enabled here
  // unless specifically intended for a different recovery mechanism.

  log_info(TAG, "Deep Sleep", "Entering indefinite deep sleep now.");

  // Enter deep sleep
  esp_deep_sleep_start();

  // Code below this line will not be executed.
  // The device will remain in deep sleep until power cycle or external reset.
}

/**
 * @brief Application cleanup function.
 */
static void app_cleanup(void)
{
  log_info(TAG, "Shutdown", "Initiating system cleanup...");

#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  log_info(TAG, "Cleanup", "Cleaning up Pin Validator...");
  pin_validator_clear_all();
#endif

#ifdef CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
  log_info(TAG, "Cleanup", "Cleaning up Time Manager...");
  time_manager_cleanup();
#endif

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  if (atomic_load(&g_file_manager.initialized)) {
    log_info(TAG, "Cleanup", "Cleaning up File Manager...");
    file_write_manager_cleanup(&g_file_manager);
  } else {
    log_info(TAG, "Cleanup Skip", "File Manager not initialized, skipping cleanup.");
  }

  if (atomic_load(&g_sd_card.initialized)) {
    log_info(TAG, "Cleanup", "Cleaning up SD Card HAL...");
    sd_card_cleanup(&g_sd_card);
  } else {
    log_info(TAG, "Cleanup Skip", "SD Card HAL not initialized, skipping cleanup.");
  }
#endif

  log_info(TAG, "Cleanup", "Attempting final log flush...");
  log_flush();
  log_info(TAG, "Cleanup", "Cleaning up Logger...");
  log_cleanup();

  printf("[%s] Cleaning up Error Handlers...\n", TAG);
  error_handler_deinit(&g_heartbeat_error_handler);
  error_handler_deinit(&g_main_error_handler);

  printf("[%s] System cleanup finished.\n", TAG);
}

static void app_init(void)
{
  esp_err_t err = ESP_OK;

  /* Initialize the main error handler first */
  err = error_handler_init(&g_main_error_handler,
                           CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_MAX_RETRIES,
                           CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_RETRY_DELAY_MS,
                           CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_MAX_DELAY_MS,
                           NULL,
                           NULL);
  if (err != ESP_OK) {
    printf("[%s] CRITICAL: Failed to initialize main error handler: %d. Halting.\n", TAG, err);
    app_halt("Main error handler init failed"); // Use original halt for this critical failure
  }

  /* --- Initialize Minimal Logger FIRST --- */
  err = log_init(NULL, NULL); // Pass NULLs for minimal init
  if (err != ESP_OK) {
    printf("[%s] CRITICAL FAILURE: Minimal logging init failed (%d). Halting.\n", TAG, err);
    app_halt("Minimal logger init failed"); // Use original halt
  }
  log_info(TAG, "Init Start", "Minimal logger initialized.");

  /* --- Initialize Core ESP-IDF Services --- */
  log_info(TAG, "Init Core", "Initializing NVS Flash...");
  err = nvs_flash_init(); // Initialize NVS
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    log_warn(TAG, "NVS Warning", "NVS requires erase. Erasing...");
    ESP_ERROR_CHECK(nvs_flash_erase()); // Halt if erase fails
    err = nvs_flash_init();
  }
  if (err != ESP_OK) { // Check error *after* potential erase/retry
    RECORD_ERROR(&g_main_error_handler, err, "Failed to initialize NVS flash");
    log_error(TAG, "CRITICAL FAILURE", "NVS init failed (%s). Halting.", esp_err_to_name(err));
    app_halt("NVS init failed"); // Use original halt
  }

  log_info(TAG, "Init Core", "Initializing Network Interface...");
  ESP_ERROR_CHECK(esp_netif_init()); // Halt on failure

  log_info(TAG, "Init Core", "Initializing Event Loop...");
  ESP_ERROR_CHECK(esp_event_loop_create_default()); // Halt on failure

  /* --- Initialize Pin Validator --- */
#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  log_info(TAG, "Init Pins", "Initializing Pin Validator...");
  err = pin_validator_init();
  if (err != ESP_OK) {
    RECORD_ERROR(&g_main_error_handler, err, "Failed to initialize Pin Validator");
    log_error(TAG,
              "CRITICAL FAILURE",
              "Pin Validator init failed (%s). Halting.",
              esp_err_to_name(err));
    app_halt("Pin validator init failed"); // Use original halt
  } else {
    log_info(TAG, "Init Complete", "Pin validator initialized successfully"); // Log success
  }
#endif

  /* --- Initialize Other Components --- */
#ifdef CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
  log_info(TAG, "Init Time", "Initializing Time Manager...");
  err = time_manager_init(); // This now should succeed if event loop is up
  if (err != ESP_OK) {
    RECORD_ERROR(&g_main_error_handler, err, "Failed to initialize Time Manager");
    log_warn(TAG,
             "Time Warning",
             "Failed to initialize Time Manager, time might not be accurate: %s",
             esp_err_to_name(err));
    /* Continue execution */
  }
#endif

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  /* SD Card and File Manager Init */
  atomic_init(&g_sd_card.initialized, false);
  atomic_init(&g_file_manager.initialized, false);

  log_info(TAG, "SD Init", "SD Card support enabled. Initializing SD HAL and File Manager...");

  sd_card_hal_t*        sd_card      = &g_sd_card;
  file_write_manager_t* file_manager = &g_file_manager;

  sd_bus_width_t bus_width =
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_4BIT_MODE
    k_sd_bus_width_4bit;
#else
    k_sd_bus_width_1bit;
#endif

  log_info(TAG,
           "Init SD HAL",
           "Initializing SD Card HAL structure (Mount: %s, Width: %d-bit)...",
           CONFIG_PSTAR_KCONFIG_SD_CARD_MOUNT_POINT,
           bus_width);
  err = sd_card_init_default(sd_card,
                             "SD Card",
                             CONFIG_PSTAR_KCONFIG_SD_CARD_MOUNT_POINT,
                             "sd_card_component",
                             bus_width);
  if (err != ESP_OK) {
    RECORD_ERROR(&g_main_error_handler, err, "Failed to initialize SD card HAL structure");
    log_error(TAG,
              "CRITICAL FAILURE",
              "SD HAL struct init failed (%s). Halting.",
              esp_err_to_name(err));
    app_halt("SD Card HAL structure init failed"); // Use original halt
  }

  err = sd_card_set_task_config(sd_card,
                                CONFIG_PSTAR_KCONFIG_SD_CARD_TASK_STACK_SIZE,
                                CONFIG_PSTAR_KCONFIG_SD_CARD_TASK_PRIORITY,
                                pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_SD_CARD_MUTEX_TIMEOUT_MS));
  if (err != ESP_OK) {
    RECORD_ERROR(&g_main_error_handler, err, "Failed to set SD card task config");
    log_error(TAG,
              "CRITICAL FAILURE",
              "SD task config failed (%s). Halting.",
              esp_err_to_name(err));
    app_halt("SD Card task config failed"); // Use original halt
  }

  log_info(TAG, "Init SD Hardware", "Initializing SD Card hardware...");
  err = sd_card_init(sd_card);
  if (err != ESP_OK) {
    RECORD_ERROR(&g_main_error_handler, err, "Failed to initialize SD card hardware");
    log_error(TAG,
              "SD HW Init Warning", // Changed to Warning, might not be fatal yet
              "SD HW init failed (%s). Continuing to Pin Validation.",
              esp_err_to_name(err));
    // Do NOT halt here yet. Let pin validation run.
  } else {
    log_info(TAG, "Init Complete", "SD card HAL initialization complete"); // Log success
  }

  // Initialize File Manager *even if* SD HW init failed,
  // as the logger might need it (though writes will fail).
  file_writer_config_t fm_config = {
    .priority    = CONFIG_PSTAR_KCONFIG_FILE_MANAGER_TASK_PRIORITY,
    .stack_depth = CONFIG_PSTAR_KCONFIG_FILE_MANAGER_TASK_STACK_SIZE,
  };
  log_info(TAG,
           "Init File Manager",
           "Initializing File Write Manager (Task Priority: %d, Stack: %lu)...",
           fm_config.priority,
           (unsigned long)fm_config.stack_depth);
  err = file_write_manager_init(file_manager, sd_card, &fm_config);
  if (err != ESP_OK) {
    RECORD_ERROR(&g_main_error_handler, err, "Failed to initialize file write manager");
    log_error(TAG,
              "CRITICAL FAILURE",
              "File Manager init failed (%s). Halting.",
              esp_err_to_name(err));
    app_halt("File manager init failed"); // Use original halt
  } else {
    log_info(TAG, "Init Complete", "File write manager initialized successfully"); // Log success
  }

  /* Re-Initialize logging system with SD card support */
  log_cleanup(); // Cleanup minimal logger
  log_info(TAG, "Init Logging", "Re-initializing Logging System (with SD support)...");
  err = log_init(file_manager, sd_card); // Init full logger
  if (err != ESP_OK) {
    log_error(TAG,
              "Full Logger Init Failed",
              "Full Logging init failed (%s). Continuing to Pin Validation.",
              esp_err_to_name(err));
    RECORD_ERROR(&g_main_error_handler, err, "Full logger init failed");
    // Do NOT call app_halt here.
  } else {
    log_info(TAG, "Init Complete", "Full logger initialized successfully"); // Log success
  }
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED */

  log_info(TAG, "System Startup", "Core services initialized successfully");

  /* --- Pin Validation (Moved to the end of initialization) --- */
#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_VALIDATE_AT_STARTUP
  log_info(TAG, "Pin Validation", "Validating pin assignments...");
#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_PRINT_ASSIGNMENTS
  log_info(TAG, "Pin Assignments", "--- Start Pin Assignment Table ---");
  pin_validator_print_assignments(); // Print assignments registered so far
  log_info(TAG, "Pin Assignments", "--- End Pin Assignment Table ---");
#endif

  // --- Call validator ---
  // The halt_on_conflict parameter passed here is now just informational
  err = pin_validator_validate_all(
#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_HALT_ON_CONFLICT
    true
#else
    false
#endif
  );

  // --- Check the result and decide action ---
  if (err != ESP_OK) { // Conflicts were found
    RECORD_ERROR(&g_main_error_handler, err, "Pin validation failed");
    log_error(TAG, "Pin Error", "Pin validation failed! Check logs for conflicts.");

// --- Always use deep sleep on pin conflict ---
#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_HALT_ON_CONFLICT
    log_warn(TAG,
             "Pin Error",
             "Pin validation failed (Kconfig HALT enabled). Entering indefinite deep sleep.");
    app_deep_sleep("Pin validation failed (Kconfig HALT enabled)");
#else
    log_warn(TAG,
             "Pin Error",
             "Pin validation failed (Kconfig HALT disabled). Entering indefinite deep sleep.");
    app_deep_sleep("Pin validation failed (Kconfig HALT disabled)");
#endif
    // --- END MODIFICATION ---

    // Code below this point won't be reached if deep sleep is entered.
  } else {
    log_info(TAG, "Pin Success", "Pin validation passed.");
  }
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_VALIDATE_AT_STARTUP */
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED */

  log_info(TAG, "Initialization Complete", "Entering main application logic...");
}