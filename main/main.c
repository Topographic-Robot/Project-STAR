/* main/main.c */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <errno.h> // Include for errno
#include <stdio.h>
#include <string.h>
#include <sys/stat.h> // Include for stat
#include <unistd.h>   // Include for unlink

#include "esp_event.h" // Needed for default event loop
#include "esp_log.h"
#include "nvs_flash.h" // Needed for NVS init

/* Project Star Includes */
#include "pstar_bus_config.h"
#include "pstar_bus_gpio.h"
#include "pstar_bus_manager.h"
#include "pstar_bus_spi.h"
#include "pstar_error_handler.h"
#include "pstar_file_write_manager.h" // Needed for SD logging
#include "pstar_log_handler.h"
#include "pstar_pin_validator.h" // <-- Include Pin Validator header
#include "pstar_time_manager.h"  // For logger timestamps

/* Storage HAL Includes */
#include "pstar_storage_common.h" // For storage_bus_width_to_string
#include "pstar_storage_hal.h"    // Includes types and main HAL functions

/* Kconfig - Still needed for component-level configs */
#include "sdkconfig.h"

// --- Simple Macro for SD Card Selection ---
// 1 = Card 1 (using default pins, mount path /sdcard)
// 2 = Card 2 (using custom pins, mount path /sdcard2)
// 3 = Both Card 1 and Card 2
// Other = No SD Cards
#define WHAT_SD_CARD_TO_USE (1) // <--- CHANGE THIS TO TEST DIFFERENT CONFIGURATIONS

/* Static Variables */
static const char* TAG = "DUAL_SD_EXAMPLE";

// --- Conditionally define variables based on the MACRO ---

#if defined(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED)
// Declare HAL instances only if SD card support is enabled globally
#if (WHAT_SD_CARD_TO_USE == 1 || WHAT_SD_CARD_TO_USE == 3)
static sd_card_hal_t g_sd_card_1; // Instance for Card 1
#endif
#if (WHAT_SD_CARD_TO_USE == 2 || WHAT_SD_CARD_TO_USE == 3)
// Define custom pins for the second card
static const sd_card_pin_config_t second_card_pins = {
  // Card detect pin for second card - Use a different pin if needed
  .gpio_det_pin = CONFIG_PSTAR_KCONFIG_SD_CARD_DET_GPIO, // Example: Use same CD pin as Card 1
  // .gpio_det_pin = 27, // Or override if Card 2 uses a different CD pin

  // SPI pins for second SD card (using SPI3_HOST, HSPI)
  .spi_cs_pin   = 26, // GPIO26 for CS
  .spi_sclk_pin = 25, // GPIO25 for SCLK
  .spi_di_pin   = 33, // GPIO33 for MISO (DI for ESP32)
  .spi_do_pin   = 32, // GPIO32 for MOSI (DO for ESP32)

  // SDIO pins - not used but initialize to -1
  .sdio_clk_pin = -1,
  .sdio_cmd_pin = -1,
  .sdio_d0_pin  = -1,
  .sdio_d1_pin  = -1,
  .sdio_d2_pin  = -1,
  .sdio_d3_pin  = -1};
static sd_card_hal_t g_sd_card_2; // Instance for Card 2
#endif                            // End Card 2 specific declarations

// File manager instance (needed only if logging to SD is enabled)
#if defined(CONFIG_PSTAR_KCONFIG_FILE_MANAGER_ENABLED) &&                                          \
  defined(CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED)
static file_write_manager_t g_file_manager;
static bool                 g_fm_initialized = false; // Track file manager init status
#endif                                                // End File Manager declaration

// Logger SD init flag
#if defined(CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED)
static bool g_sd_logging_fully_initialized = false; // Track if SD logging is fully active
#endif

#endif // CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED

// --- Test function using standard file I/O ---
void test_sd_card(const char* card_name, const char* mount_path)
{
#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  log_warn(TAG, "Test Skipped", "SD Card support is disabled, cannot test %s", card_name);
  return;
#else
  char filename_full[CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_PATH_LENGTH * 2];
  char message[64];
  char buffer[200] = {0}; // Increased buffer size
  int  num_writes  = 3;

  // Construct full path using the provided mount_path
  snprintf(filename_full, sizeof(filename_full), "%s/test_%s.txt", mount_path, card_name);

  log_info(TAG, "Test Start", "Testing %s (Path: %s)", card_name, filename_full);

  // --- Write Test ---
  for (int i = 0; i < num_writes; ++i) {
    snprintf(message, sizeof(message), "Hello from %s SD card! (Write %d)", card_name, i + 1);
    bool append_mode = (i > 0);

    FILE* f = fopen(filename_full, append_mode ? "a" : "w");
    if (f == NULL) {
      log_error(TAG,
                "Test Write Error",
                "Failed to open file '%s' for %s: %s (errno %d)",
                filename_full,
                append_mode ? "append" : "write",
                strerror(errno),
                errno);
      return; // Exit test on failure
    }
    int written_len = fprintf(f, "%s%s", (append_mode ? "\n" : ""), message);
    fclose(f);

    if (written_len < 0) {
      log_error(TAG,
                "Test Write Error",
                "fprintf failed for file '%s' (attempt %d)",
                filename_full,
                i + 1);
      return; // Exit test on failure
    }
    log_info(TAG,
             "Test Write OK",
             "Successfully wrote (attempt %d) to %s: \"%s\"",
             i + 1,
             card_name,
             message);
    vTaskDelay(pdMS_TO_TICKS(50)); // Small delay between writes
  }

  // --- Read Test ---
  memset(buffer, 0, sizeof(buffer));
  FILE* f_read = fopen(filename_full, "r");
  if (f_read == NULL) {
    log_error(TAG,
              "Test Read Error",
              "Failed to open file '%s' for reading: %s (errno %d)",
              filename_full,
              strerror(errno),
              errno);
    // Attempt cleanup even if read fails
    if (unlink(filename_full) != 0) {
      log_error(TAG,
                "Test Cleanup Error",
                "Failed to delete test file '%s' after read error: %s",
                filename_full,
                strerror(errno));
    }
    return; // Exit test
  }
  size_t bytes_read = fread(buffer, 1, sizeof(buffer) - 1, f_read);
  // Check for read errors specifically
  if (ferror(f_read)) {
    log_error(TAG, "Test Read Error", "Error occurred during fread for '%s'", filename_full);
    fclose(f_read);
    // Attempt cleanup
    if (unlink(filename_full) != 0) {
      log_error(TAG,
                "Test Cleanup Error",
                "Failed to delete test file '%s' after read error: %s",
                filename_full,
                strerror(errno));
    }
    return; // Exit test
  }
  fclose(f_read);
  buffer[bytes_read] = '\0'; // Null-terminate the buffer

  log_info(TAG,
           "Test Read OK",
           "Successfully read from %s (%d bytes):\n--- File Content ---\n%s\n--- End Content ---",
           card_name,
           (int)bytes_read,
           buffer);

  // --- Verification ---
  char expected_content[sizeof(message) * num_writes + num_writes]; // Rough estimate
  expected_content[0] = '\0';
  for (int i = 0; i < num_writes; ++i) {
    snprintf(message, sizeof(message), "Hello from %s SD card! (Write %d)", card_name, i + 1);
    if (i > 0)
      strcat(expected_content, "\n");
    strcat(expected_content, message);
  }

  if (strcmp(buffer, expected_content) == 0) {
    log_info(TAG, "Test Verify OK", "Content verified successfully for %s.", card_name);
  } else {
    log_error(TAG, "Test Verify FAIL", "Content mismatch for %s!", card_name);
    log_error(TAG, "Test Verify FAIL", "Expected:\n%s", expected_content);
    log_error(TAG, "Test Verify FAIL", "Got:\n%s", buffer);
  }

  // --- Delete Test File ---
  if (unlink(filename_full) != 0) {
    log_error(TAG,
              "Test Delete Error",
              "Failed to delete test file '%s': %s (errno %d)",
              filename_full,
              strerror(errno),
              errno);
  } else {
    log_info(TAG, "Test Delete OK", "Successfully deleted test file for %s.", card_name);
  }
#endif // CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
}

void app_main(void)
{
  esp_err_t err;
  // Declare status flags *only* if SD card support is enabled globally
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
#if (WHAT_SD_CARD_TO_USE == 1 || WHAT_SD_CARD_TO_USE == 3)
  bool card1_struct_initialized = false; // Tracks if sd_card_init_default was called
  bool card1_hal_initialized    = false; // Tracks if sd_card_init was called
  bool card1_ready              = false; // Tracks if sd_card_is_available is true
#endif
#if (WHAT_SD_CARD_TO_USE == 2 || WHAT_SD_CARD_TO_USE == 3)
  bool card2_struct_initialized = false;
  bool card2_hal_initialized    = false;
  bool card2_ready              = false;
#endif
  sd_card_hal_t* target_sd_for_logging = NULL; // Pointer to the card used for logging
#endif                                         // CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED

  // --- Essential Initializations ---
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  ESP_ERROR_CHECK(esp_event_loop_create_default());
#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  ESP_ERROR_CHECK(pin_validator_init());
#endif
  // Perform minimal logger init first (console only)
  ESP_ERROR_CHECK(log_init(NULL, NULL));
#ifdef CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
  ESP_ERROR_CHECK(time_manager_init());
#endif
  log_info(TAG, "System Init", "Basic system services initialized.");
  log_info(TAG, "Dual SD Start", "----------------------------------");
  log_info(TAG, "Dual SD Start", "Starting Dual SD Card Example:");
#if WHAT_SD_CARD_TO_USE == 1
  log_info(TAG, "Dual SD Start", "Mode: Card 1 Only (/sdcard)");
#elif WHAT_SD_CARD_TO_USE == 2
  log_info(TAG, "Dual SD Start", "Mode: Card 2 Only (/sdcard2)");
#elif WHAT_SD_CARD_TO_USE == 3
  log_info(TAG, "Dual SD Start", "Mode: Both Cards (/sdcard, /sdcard2)");
#else
  log_info(TAG, "Dual SD Start", "Mode: No SD Cards Selected");
#endif
  log_info(TAG, "Dual SD Start", "----------------------------------");

  /************************************************************************
     * Initialize Card 1 HAL if selected
     ************************************************************************/
#if defined(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED) &&                                               \
  (WHAT_SD_CARD_TO_USE == 1 || WHAT_SD_CARD_TO_USE == 3)
  log_info(TAG, "Card 1 Init", "Initializing HAL for first SD card (/sdcard)...");
  // Step 1: Initialize the HAL structure with default pins and mount path
  err = sd_card_init_default(&g_sd_card_1,
                             "SD_Card_1",
                             CONFIG_PSTAR_KCONFIG_SD_CARD_MOUNT_POINT, // Use Kconfig default mount
                             "storage_card_1",
                             k_sd_bus_width_1bit); // Use 1-bit for SPI
  if (err != ESP_OK) {
    log_error(TAG,
              "Card 1 Error",
              "Failed to initialize SD card 1 struct: %s",
              esp_err_to_name(err));
  } else {
    card1_struct_initialized = true;
    // Step 2: Initialize the hardware and start the mount task
    err = sd_card_init(&g_sd_card_1);
    if (err != ESP_OK) {
      log_error(TAG,
                "Card 1 Error",
                "Failed to start SD card 1 HAL (task/detection): %s",
                esp_err_to_name(err));
      // Attempt cleanup if struct init succeeded but HAL init failed
      sd_card_cleanup(&g_sd_card_1);
      card1_struct_initialized = false; // Mark as not fully initialized
    } else {
      log_info(TAG, "Card 1 OK", "First SD card HAL initialized successfully!");
      card1_hal_initialized = true; // Set flag on success
    }
  }
#endif

  /************************************************************************
     * Initialize Card 2 HAL if selected
     ************************************************************************/
#if defined(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED) &&                                               \
  (WHAT_SD_CARD_TO_USE == 2 || WHAT_SD_CARD_TO_USE == 3)
  log_info(TAG, "Card 2 Init", "Initializing HAL for second SD card (/sdcard2)...");

  // --- Step 1: Initialize HAL struct with custom pins and mount path ---
  err = sd_card_init_with_pins(&g_sd_card_2,
                               "SD_Card_2",
                               "/sdcard2", // Custom mount path for second card
                               "storage_card_2",
                               k_sd_bus_width_1bit, // Use 1-bit for SPI
                               &second_card_pins);
  if (err != ESP_OK) {
    log_error(TAG,
              "Card 2 Error",
              "Failed to initialize SD card 2 struct: %s",
              esp_err_to_name(err));
  } else {
    card2_struct_initialized = true;
    // --- Step 2: Initialize the hardware and start the mount task ---
    // Note: sd_card_init will internally handle creating/managing the necessary
    // SPI and GPIO buses using the pin config provided in step 1.
    err = sd_card_init(&g_sd_card_2);
    if (err != ESP_OK) {
      log_error(TAG,
                "Card 2 Error",
                "Failed to start SD card 2 HAL (task/detection): %s",
                esp_err_to_name(err));
      // Attempt cleanup if struct init succeeded but HAL init failed
      sd_card_cleanup(&g_sd_card_2);
      card2_struct_initialized = false; // Mark as not fully initialized
    } else {
      log_info(TAG, "Card 2 OK", "Second SD card HAL initialized successfully!");
      card2_hal_initialized = true;
    }
  }
#endif // End Card 2 Init block

  /************************************************************************
     * COMPLETE LOGGER INITIALIZATION (if SD logging enabled)
     ************************************************************************/
#if defined(CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED) &&                                       \
  defined(CONFIG_PSTAR_KCONFIG_FILE_MANAGER_ENABLED) &&                                            \
  defined(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED)

  // Determine which SD card HAL instance to use for logging based on the macro
  target_sd_for_logging           = NULL;
  const char* logging_target_name = "NONE";

#if WHAT_SD_CARD_TO_USE == 1
  // Card 1 only
  if (card1_hal_initialized) {
    target_sd_for_logging = &g_sd_card_1;
    logging_target_name   = "Card 1";
  } else {
    log_warn(TAG,
             "Logger Config",
             "SD Logging target Card 1 selected but failed HAL init. Cannot log to SD.");
  }
#elif WHAT_SD_CARD_TO_USE == 2
  // Card 2 only
  if (card2_hal_initialized) {
    target_sd_for_logging = &g_sd_card_2;
    logging_target_name   = "Card 2";
  } else {
    log_warn(TAG,
             "Logger Config",
             "SD Logging target Card 2 selected but failed HAL init. Cannot log to SD.");
  }
#elif WHAT_SD_CARD_TO_USE == 3
  // Both cards - Default to Card 1 for logging (can be changed)
  if (card1_hal_initialized) {
    target_sd_for_logging = &g_sd_card_1;
    logging_target_name   = "Card 1";
  } else if (card2_hal_initialized) {
    // Fallback to Card 2 if Card 1 failed but Card 2 succeeded
    target_sd_for_logging = &g_sd_card_2;
    logging_target_name   = "Card 2 (Fallback)";
    log_warn(TAG, "Logger Config", "SD Logging target Card 1 failed init, falling back to Card 2.");
  } else {
    log_warn(TAG,
             "Logger Config",
             "SD Logging enabled, but both Card 1 and Card 2 failed HAL init. Cannot log to SD.");
  }
#endif

  // Initialize File Manager and Logger only if a valid target SD card was determined and initialized
  if (target_sd_for_logging != NULL) {
    log_info(TAG,
             "Logger Init",
             "Attempting full logger initialization (File Manager + %s)...",
             logging_target_name);
    err = file_write_manager_init(&g_file_manager, target_sd_for_logging, NULL);
    if (err != ESP_OK) {
      log_error(TAG,
                "File Mgr Error",
                "Failed to initialize File Write Manager: %s",
                esp_err_to_name(err));
      g_sd_logging_fully_initialized = false;
    } else {
      g_fm_initialized = true; // Mark file manager as initialized
      log_info(TAG, "File Mgr OK", "File Write Manager initialized.");
      // Now complete logger init
      err = log_init(&g_file_manager, target_sd_for_logging);
      if (err != ESP_OK) {
        log_error(TAG,
                  "Logger Error",
                  "Failed to complete logger initialization for SD card: %s",
                  esp_err_to_name(err));
        g_sd_logging_fully_initialized = false;
        // Cleanup file manager if logger init failed
        file_write_manager_cleanup(&g_file_manager);
        g_fm_initialized = false;
      } else {
        log_info(TAG,
                 "Logger OK",
                 "Logger fully initialized for SD card writing (%s).",
                 logging_target_name);
        g_sd_logging_fully_initialized = true; // Set flag
      }
    }
  } else if (CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED && WHAT_SD_CARD_TO_USE != 0 &&
             WHAT_SD_CARD_TO_USE != -1) {
    // Log a warning if SD logging was enabled but no suitable target was found/initialized
    log_warn(
      TAG,
      "Logger Skip",
      "SD Logging enabled, but no suitable target SD card was initialized. SD Logging disabled.");
    g_sd_logging_fully_initialized = false;
  }

#endif // End Logger/FM Init block

  /************************************************************************
     * PRINT PIN ASSIGNMENTS (if validator enabled)
     ************************************************************************/
#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  log_info(TAG, "Pin Assignments", "Printing final pin assignments...");
  err = pin_validator_print_assignments();
  if (err != ESP_OK) {
    log_error(TAG, "Pin Print Error", "Failed to print pin assignments: %s", esp_err_to_name(err));
    // Continue execution even if printing fails
  }
#endif // CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED

  /************************************************************************
     * WAIT FOR CARDS TO BE READY
     ************************************************************************/
  log_info(TAG, "Wait Ready", "Waiting for selected SD cards to be ready (up to 5 seconds)...");

  int timeout = 50; // 5 seconds (100ms * 50)

#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  while (timeout > 0) {
    bool card1_target_met = false;
    bool card2_target_met = false;

#if (WHAT_SD_CARD_TO_USE == 1 || WHAT_SD_CARD_TO_USE == 3)
    if (card1_hal_initialized)
      card1_ready = sd_card_is_available(&g_sd_card_1); // Use the correct HAL function
    card1_target_met = card1_ready;
#else
    card1_target_met = true; // Not targeting card 1
#endif

#if (WHAT_SD_CARD_TO_USE == 2 || WHAT_SD_CARD_TO_USE == 3)
    if (card2_hal_initialized)
      card2_ready = sd_card_is_available(&g_sd_card_2);
    card2_target_met = card2_ready;
#else
    card2_target_met = true; // Not targeting card 2
#endif

    if (card1_target_met && card2_target_met) {
      log_info(TAG, "Wait Ready", "All selected cards are ready.");
      break; // Exit if all targeted cards are ready
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    timeout--;
  }
  if (timeout <= 0) {
    log_warn(TAG, "Wait Timeout", "Timeout waiting for SD card(s) to become ready.");
  }
#else
  timeout = 0; // Skip wait if SD disabled
#endif

  /************************************************************************
     * DISPLAY CARD STATUS AND PERFORM OPERATIONS
     ************************************************************************/
  log_info(TAG, "Card Status", "----------------------------------");
#if defined(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED) &&                                               \
  (WHAT_SD_CARD_TO_USE == 1 || WHAT_SD_CARD_TO_USE == 3)
  log_info(TAG,
           "Card Status",
           "Card #1 (/sdcard): %s",
           card1_ready ? "READY" : (card1_hal_initialized ? "INIT FAILED" : "NOT INITIALIZED"));
#endif
#if defined(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED) &&                                               \
  (WHAT_SD_CARD_TO_USE == 2 || WHAT_SD_CARD_TO_USE == 3)
  log_info(TAG,
           "Card Status",
           "Card #2 (/sdcard2): %s",
           card2_ready ? "READY" : (card2_hal_initialized ? "INIT FAILED" : "NOT INITIALIZED"));
#endif
#if WHAT_SD_CARD_TO_USE <= 0 // Covers 0, -1, etc.
  log_info(TAG, "Card Status", "No SD Cards selected via WHAT_SD_CARD_TO_USE macro.");
#endif
#ifndef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  log_info(TAG, "Card Status", "SD Card support is disabled globally via Kconfig.");
#endif
  log_info(TAG, "Card Status", "----------------------------------");

  // Test operations on each targeted and ready card
#if defined(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED) &&                                               \
  (WHAT_SD_CARD_TO_USE == 1 || WHAT_SD_CARD_TO_USE == 3)
  if (card1_ready) {
    test_sd_card("Card1", CONFIG_PSTAR_KCONFIG_SD_CARD_MOUNT_POINT); // Use standard file ops
  } else if (card1_hal_initialized) { // Log skip only if init was attempted
    log_warn(TAG, "Test Skip", "Skipping test for Card #1 (not ready).");
  }
#endif

#if defined(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED) &&                                               \
  (WHAT_SD_CARD_TO_USE == 2 || WHAT_SD_CARD_TO_USE == 3)
  if (card2_ready) {
    test_sd_card("Card2", "/sdcard2"); // Use standard file ops
  } else if (card2_hal_initialized) {  // Log skip only if init was attempted
    log_warn(TAG, "Test Skip", "Skipping test for Card #2 (not ready).");
  }
#endif

  log_info(TAG, "Example End", "Dual SD card example finished testing.");

  // --- Cleanup ---
  log_info(TAG, "Cleanup", "Starting cleanup...");

#if defined(CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED) &&                                       \
  defined(CONFIG_PSTAR_KCONFIG_FILE_MANAGER_ENABLED) &&                                            \
  defined(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED)
  // Cleanup logger and file manager only if they were fully initialized
  if (g_sd_logging_fully_initialized) {
    log_cleanup(); // Cleanup logger first (flushes buffers)
  }
  if (g_fm_initialized) {
    file_write_manager_cleanup(&g_file_manager);
  }
#endif

#if defined(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED) &&                                               \
  (WHAT_SD_CARD_TO_USE == 2 || WHAT_SD_CARD_TO_USE == 3)
  // Cleanup second SD card if its HAL was initialized
  if (card2_hal_initialized) {
    log_info(TAG, "Cleanup", "Cleaning up Card #2 HAL...");
    sd_card_cleanup(&g_sd_card_2);
  }
#endif

#if defined(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED) &&                                               \
  (WHAT_SD_CARD_TO_USE == 1 || WHAT_SD_CARD_TO_USE == 3)
  // Cleanup first SD card if its HAL was initialized
  if (card1_hal_initialized) {
    log_info(TAG, "Cleanup", "Cleaning up Card #1 HAL...");
    sd_card_cleanup(&g_sd_card_1);
  }
#endif

#ifdef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  // pin_validator_cleanup(); // Add if a cleanup function exists
  log_info(TAG, "Cleanup", "Pin validator cleanup (if implemented).");
#endif

#ifdef CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
  time_manager_cleanup();
#endif

  log_info(TAG, "Cleanup", "Cleanup finished. Exiting app_main.");
} // End app_main