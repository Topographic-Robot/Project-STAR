/* main/main.c */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "pstar_log_handler.h"

#include "pstar_error_handler.h"

#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
#include "pstar_pin_validator.h"
#endif

#if CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
#include "pstar_time_manager.h"
#endif

#if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
#include "pstar_sd_card_hal.h"
#include "pstar_file_write_manager.h"
#endif

#define MAIN_TAG ("Main")

void app_main(void)
{
  esp_err_t err = ESP_OK;
  
  /* Initialize the error handler first to support error handling during startup */
  error_handler_t main_error_handler;
  error_handler_init(&main_error_handler, 
                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_MAX_RETRIES, 
                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_RETRY_DELAY_MS,
                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_MAX_DELAY_MS,
                     NULL, /* No reset function for main error handler */
                     NULL);

#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  /*
   * Initialize Pin Validator early.
   * NOTE: If PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED is true, this init call is mandatory.
   * The AUTO_INIT Kconfig option was removed; initialization is now always
   * performed here when the component is enabled via Kconfig. If manual timing
   * control is needed, this block could be removed, and the user MUST call
   * pin_validator_init() manually elsewhere.
   */
  log_info(MAIN_TAG, "Init", "Initializing Pin Validator...");
  err = pin_validator_init();
  if (err != ESP_OK) {
    RECORD_ERROR(&main_error_handler, err, "Failed to initialize Pin Validator");
    /* Decide how to handle this - maybe halt or continue with caution */
    /* For now, we'll halt using ESP_ERROR_CHECK */
    ESP_ERROR_CHECK(err);
  }
#endif

#if CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
  /* Initialize Time Manager */
  log_info(MAIN_TAG, "Init Time", "Initializing Time Manager...");
  err = time_manager_init();
  if (err != ESP_OK) {
    /* Time manager failure might not be critical depending on the application */
    RECORD_ERROR(&main_error_handler, err, "Failed to initialize Time Manager");
    log_warn(MAIN_TAG, "Time Warning", "Failed to initialize Time Manager, time might not be accurate: %s", esp_err_to_name(err));
    /* Continue execution */
  }
#endif

  /* --- SD Card and related component initialization --- */
#if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  log_info(MAIN_TAG, "SD Init", "SD Card support enabled. Initializing components...");

  /* Allocate memory for HAL structures */
  sd_card_hal_t* sd_card = malloc(sizeof(sd_card_hal_t));
  file_write_manager_t* file_manager = malloc(sizeof(file_write_manager_t));

  if (sd_card == NULL || file_manager == NULL) {
    RECORD_ERROR(&main_error_handler, ESP_ERR_NO_MEM, "Failed to allocate memory for SD/File managers");
    log_error(MAIN_TAG, "Memory Error", "Failed to allocate memory for SD/File managers");
    free(sd_card); /* Safe to call free(NULL) */
    free(file_manager);
    return; /* Or halt */
  }

  /* Determine SD card bus width from Kconfig */
  sd_bus_width_t bus_width = CONFIG_PSTAR_KCONFIG_SD_CARD_4BIT_MODE ? k_sd_bus_width_4bit : k_sd_bus_width_1bit;

  /* Initialize the SD card HAL structure with Kconfig values */
  log_info(MAIN_TAG, "Init SD HAL", "Initializing SD Card HAL structure (Mount: %s, Width: %d-bit)...",
           CONFIG_PSTAR_KCONFIG_SD_CARD_MOUNT_POINT, bus_width);
  err = sd_card_init_default(sd_card,
                             "SD Card", /* Tag */
                             CONFIG_PSTAR_KCONFIG_SD_CARD_MOUNT_POINT, /* Mount point from Kconfig */
                             "sd_card_component", /* Component ID */
                             bus_width); /* Bus width from Kconfig */
  if (err != ESP_OK) {
    RECORD_ERROR(&main_error_handler, err, "Failed to initialize SD card HAL structure");
    log_error(MAIN_TAG, "SD HAL Error", "Failed to initialize SD card HAL structure: %s", esp_err_to_name(err));
    free(sd_card);
    free(file_manager);
    ESP_ERROR_CHECK(err); /* Halt on critical error */
  }

  /* Initialize the SD card hardware with Kconfig priority */
  log_info(MAIN_TAG, "Init SD Hardware", "Initializing SD Card hardware (Task Priority: %d)...", CONFIG_PSTAR_KCONFIG_SD_CARD_TASK_PRIORITY);
  err = sd_card_init(sd_card, CONFIG_PSTAR_KCONFIG_SD_CARD_TASK_PRIORITY);
  if (err != ESP_OK) {
    RECORD_ERROR(&main_error_handler, err, "Failed to initialize SD card hardware");
    log_error(MAIN_TAG, "SD HW Error", "Failed to initialize SD card hardware: %s", esp_err_to_name(err));
    /* sd_card_init_default might have initialized some parts, attempt cleanup */
    sd_card_cleanup(sd_card); /* Attempt cleanup */
    free(sd_card);
    free(file_manager);
    ESP_ERROR_CHECK(err); /* Halt on critical error */
  }

  /* Configure and initialize file write manager using Kconfig values */
  file_writer_config_t fm_config = {
    .priority    = CONFIG_PSTAR_KCONFIG_FILE_MANAGER_TASK_PRIORITY,
    .stack_depth = CONFIG_PSTAR_KCONFIG_FILE_MANAGER_TASK_STACK_SIZE,
    .enabled     = true /* Assuming enabled if SD card is enabled */
  };
  log_info(MAIN_TAG, "Init File Manager", "Initializing File Write Manager (Task Priority: %d, Stack: %lu)...",
           fm_config.priority, (unsigned long)fm_config.stack_depth); /* Use %lu for uint32_t */
  err = file_write_manager_init(file_manager, sd_card, &fm_config);
  if (err != ESP_OK) {
    RECORD_ERROR(&main_error_handler, err, "Failed to initialize file write manager");
    log_error(MAIN_TAG, "File Manager Error", "Failed to initialize file write manager: %s", esp_err_to_name(err));
    sd_card_cleanup(sd_card);
    free(sd_card);
    free(file_manager);
    ESP_ERROR_CHECK(err); /* Halt on critical error */
  }

  /* Initialize logging system with SD card support */
  log_info(MAIN_TAG, "Init Logging", "Initializing Logging System (with SD support)...");
  err = log_init(file_manager, sd_card);
  if (err != ESP_OK) {
    RECORD_ERROR(&main_error_handler, err, "Failed to initialize logging system");
    log_error(MAIN_TAG, "Log Init Error", "Failed to initialize logging system: %s", esp_err_to_name(err));
    file_write_manager_cleanup(file_manager);
    sd_card_cleanup(sd_card);
    free(file_manager);
    free(sd_card);
    ESP_ERROR_CHECK(err); /* Halt on critical error */
  }
#else
  /* When SD card is disabled, initialize minimal logging */
  log_info(MAIN_TAG, "No SD", "SD Card support disabled. Initializing minimal logging...");
  err = log_init(NULL, NULL);
  if (err != ESP_OK) {
    RECORD_ERROR(&main_error_handler, err, "Failed to initialize minimal logging system");
    log_error(MAIN_TAG, "Log Init Error", "Failed to initialize minimal logging system: %s", esp_err_to_name(err));
    ESP_ERROR_CHECK(err); /* Halt on critical error */
  }
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED */

  log_info(MAIN_TAG, "System Startup", "Core services initialized successfully");

  /* --- Pin Validation (Post-Initialization) --- */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_VALIDATE_AT_STARTUP
  log_info(MAIN_TAG, "Pin Validation", "Validating pin assignments...");
  #if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_PRINT_ASSIGNMENTS
    pin_validator_print_assignments(); /* Print before validating if desired */
  #endif
  err = pin_validator_validate_all(CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_HALT_ON_CONFLICT);
  if (err != ESP_OK) {
    RECORD_ERROR(&main_error_handler, err, "Pin validation failed");
    log_error(MAIN_TAG, "Pin Error", "Pin validation failed!");
    /* If not halting, the system continues here despite conflicts. */
    /* If halting is enabled, pin_validator_validate_all will not return on error. */
  } else {
    log_info(MAIN_TAG, "Pin Success", "Pin validation passed.");
  }
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_VALIDATE_AT_STARTUP */

  log_info(MAIN_TAG, "Initialization Complete", "Entering main application logic...");

  /* --- Main Application Logic Goes Here --- */
  /* Setup recovery error handler for the heartbeat loop */
  error_handler_t heartbeat_error_handler;
  error_handler_init(&heartbeat_error_handler, 
                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_MAX_RETRIES, 
                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_RETRY_DELAY_MS,
                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_MAX_DELAY_MS,
                     NULL, /* No reset function for heartbeat errors */
                     NULL);
  
  /* Example: Log a message periodically */
  int count = 0;
  while(1) {
    /* Log with error handler integration - don't try to assign return value */
    log_info(MAIN_TAG, "Heartbeat", "System running... Count: %d", count++);
    
    vTaskDelay(pdMS_TO_TICKS(5000)); /* Delay 5 seconds */

    #if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
    /* Example file write with error handling */
    if (file_manager && sd_card) {
      if (sd_card_is_available(sd_card)) {
        char buffer[50];
        snprintf(buffer, sizeof(buffer), "Log entry number %d", count);
        
        esp_err_t write_err = file_write_enqueue(file_manager, "test.log", buffer);
        if (write_err != ESP_OK) {
          RECORD_ERROR(&heartbeat_error_handler, write_err, "Failed to enqueue file write");
          log_warn(MAIN_TAG, "File Write", "Failed to enqueue log entry: %s", esp_err_to_name(write_err));
          
          /* If we've reached max retries for file writes, try to flush the log system */
          if (!error_handler_can_retry(&heartbeat_error_handler)) {
            log_warn(MAIN_TAG, "Error Recovery", "Max retries reached for file writes, flushing logs");
            log_flush();
            error_handler_reset_state(&heartbeat_error_handler); /* Reset error state to try again */
          }
        }
      } else {
        log_warn(MAIN_TAG, "File Write", "SD card not available, skipping write.");
      }
    }
    #endif
    
    /* Demonstrate error counter reset after successful operations */
    if (count % 10 == 0 && count > 0) {
      log_info(MAIN_TAG, "Error Reset", "Resetting error state after %d successful iterations", count);
      error_handler_reset_state(&heartbeat_error_handler);
    }
  }

/* --- Cleanup Code (for low battery or shutdown) --- */
#if 0 /* This block is currently disabled, activate for shutdown */
  log_info(MAIN_TAG, "Shutdown", "Shutting down...");
  log_cleanup(); /* Cleanup logger first */
#if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  if (file_manager) {
    file_write_manager_cleanup(file_manager);
    free(file_manager);
    file_manager = NULL; /* Prevent double free */
  }
  if (sd_card) {
    sd_card_cleanup(sd_card);
    free(sd_card);
    sd_card = NULL; /* Prevent double free */
  }
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED */
#if CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
  time_manager_cleanup();
#endif /* CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED */
  /* Pin validator cleanup is usually not needed unless re-initializing */
  log_info(MAIN_TAG, "Shutdown Complete", "System shutdown complete.");
  /* Optionally enter deep sleep or halt */
#endif /* 0 */
}