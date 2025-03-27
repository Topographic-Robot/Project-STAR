/* main/main.c */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "pstar_log_handler.h" /* Should be included early for logging during init */
#include "esp_log.h" /* Include esp_log ONLY for unavoidable early logs */

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

/* Fix: Use consistent TAG */
static const char *TAG = "MainApp"; /* Renamed MAIN_TAG to TAG */

/* --- Fix: Static Allocation for Core Managers/HALs --- */
#if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
static sd_card_hal_t        g_sd_card; /* Global static SD card HAL instance */
static file_write_manager_t g_file_manager; /* Global static file manager instance */
#endif
static error_handler_t      g_main_error_handler; /* Global static main error handler */
static error_handler_t      g_heartbeat_error_handler; /* Global static heartbeat error handler */
/* ---------------------------------------------------- */

/* --- Forward Declaration for potential cleanup --- */
/* FIX: Add unused attribute to suppress warning since while(1) prevents call */
static void __attribute__((unused)) app_cleanup(void);

void app_main(void)
{
  esp_err_t err = ESP_OK;

  /* Initialize the main error handler first */
  err = error_handler_init(&g_main_error_handler,
                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_MAX_RETRIES,
                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_RETRY_DELAY_MS,
                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_MAX_DELAY_MS,
                     NULL, /* No reset function for main error handler */
                     NULL);
   if (err != ESP_OK) {
       /* CRITICAL: Cannot use log_xxx yet. MUST use ESP_LOGE */
       ESP_LOGE(TAG, "CRITICAL: Failed to initialize main error handler: %s", esp_err_to_name(err));
       esp_restart();
   }

   /* --- Initialize Minimal Logger FIRST --- */
   /* This allows subsequent init functions to use log_xxx */
   err = log_init(NULL, NULL);
   if (err != ESP_OK) {
       /* CRITICAL: If even minimal logging fails, use ESP_LOGE and restart */
       ESP_LOGE(TAG, "CRITICAL FAILURE: Minimal logging init failed (%s). Restarting.", esp_err_to_name(err));
       esp_restart();
   }
   /* Now log_xxx functions can be used for console output */
   log_info(TAG, "Init Start", "Minimal logger initialized.");


#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  /* Initialize Pin Validator early */
  log_info(TAG, "Init Pins", "Initializing Pin Validator...");
  err = pin_validator_init();
  if (err != ESP_OK) {
    /* Use log_error now that minimal logger is up */
    log_error(TAG, "CRITICAL FAILURE", "Pin Validator init failed (%s). Restarting.", esp_err_to_name(err));
    /* RECORD_ERROR could be used here if g_main_error_handler is considered safe after its init */
    RECORD_ERROR(&g_main_error_handler, err, "Failed to initialize Pin Validator");
    esp_restart();
  }
#endif

  /* --- Initialize Other Components --- */

#if CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
  /* Initialize Time Manager */
  log_info(TAG, "Init Time", "Initializing Time Manager...");
  err = time_manager_init();
  if (err != ESP_OK) {
    /* Time manager failure might not be critical depending on the application */
    RECORD_ERROR(&g_main_error_handler, err, "Failed to initialize Time Manager");
    log_warn(TAG, "Time Warning", "Failed to initialize Time Manager, time might not be accurate: %s", esp_err_to_name(err));
    /* Continue execution */
  }
#endif

  /* --- SD Card and related component initialization --- */
#if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
  log_info(TAG, "SD Init", "SD Card support enabled. Initializing SD HAL and File Manager...");

  /* Use statically allocated structs */
  sd_card_hal_t* sd_card = &g_sd_card;
  file_write_manager_t* file_manager = &g_file_manager;

  /* Determine SD card bus width from Kconfig */
  sd_bus_width_t bus_width = CONFIG_PSTAR_KCONFIG_SD_CARD_4BIT_MODE ? k_sd_bus_width_4bit : k_sd_bus_width_1bit;

  /* Initialize the SD card HAL structure with Kconfig values */
  log_info(TAG, "Init SD HAL", "Initializing SD Card HAL structure (Mount: %s, Width: %d-bit)...",
           CONFIG_PSTAR_KCONFIG_SD_CARD_MOUNT_POINT, bus_width);
  err = sd_card_init_default(sd_card,
                             "SD Card", /* Tag */
                             CONFIG_PSTAR_KCONFIG_SD_CARD_MOUNT_POINT, /* Mount point from Kconfig */
                             "sd_card_component", /* Component ID */
                             bus_width); /* Bus width from Kconfig */
  if (err != ESP_OK) {
    RECORD_ERROR(&g_main_error_handler, err, "Failed to initialize SD card HAL structure");
    log_error(TAG, "CRITICAL FAILURE: SD HAL struct init failed (%s). Restarting.", esp_err_to_name(err));
    app_cleanup(); /* Attempt cleanup before restart */
    esp_restart();
  }

  /* Optionally configure task parameters if needed (example using Kconfig values) */
  err = sd_card_set_task_config(sd_card,
                                CONFIG_PSTAR_KCONFIG_SD_CARD_TASK_STACK_SIZE,
                                CONFIG_PSTAR_KCONFIG_SD_CARD_TASK_PRIORITY,
                                pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_SD_CARD_MUTEX_TIMEOUT_MS));
   if (err != ESP_OK) {
       RECORD_ERROR(&g_main_error_handler, err, "Failed to set SD card task config");
       log_error(TAG, "CRITICAL FAILURE: SD task config failed (%s). Restarting.", esp_err_to_name(err));
       app_cleanup(); /* Attempt cleanup before restart */
       esp_restart();
   }


  /* Initialize the SD card hardware */
  log_info(TAG, "Init SD Hardware", "Initializing SD Card hardware...");
  err = sd_card_init(sd_card, CONFIG_PSTAR_KCONFIG_SD_CARD_TASK_PRIORITY); /* Pass priority from Kconfig (or task config?) */
  if (err != ESP_OK) {
    RECORD_ERROR(&g_main_error_handler, err, "Failed to initialize SD card hardware");
    log_error(TAG, "CRITICAL FAILURE: SD HW init failed (%s). Restarting.", esp_err_to_name(err));
    app_cleanup(); /* Attempt cleanup before restart */
    esp_restart();
  }

  /* Configure and initialize file write manager using Kconfig values */
  file_writer_config_t fm_config = {
    .priority    = CONFIG_PSTAR_KCONFIG_FILE_MANAGER_TASK_PRIORITY,
    .stack_depth = CONFIG_PSTAR_KCONFIG_FILE_MANAGER_TASK_STACK_SIZE,
  };
  log_info(TAG, "Init File Manager", "Initializing File Write Manager (Task Priority: %d, Stack: %lu)...",
           fm_config.priority, (unsigned long)fm_config.stack_depth);
  err = file_write_manager_init(file_manager, sd_card, &fm_config);
  if (err != ESP_OK) {
    RECORD_ERROR(&g_main_error_handler, err, "Failed to initialize file write manager");
    log_error(TAG, "CRITICAL FAILURE: File Manager init failed (%s). Restarting.", esp_err_to_name(err));
    app_cleanup(); /* Attempt cleanup before restart */
    esp_restart();
  }

  /* Re-Initialize logging system with SD card support */
  /* Cleanup minimal logger first */
  log_cleanup();
  /* Init full logger */
  log_info(TAG, "Init Logging", "Re-initializing Logging System (with SD support)...");
  err = log_init(file_manager, sd_card);
  if (err != ESP_OK) {
    /* Use ESP_LOGE as full logger init failed */
    ESP_LOGE(TAG, "CRITICAL FAILURE: Full Logging init failed (%s). Restarting.", esp_err_to_name(err));
    app_cleanup(); /* Attempt cleanup before restart */
    esp_restart();
  }
#endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED */

  log_info(TAG, "System Startup", "Core services initialized successfully");

  /* --- Pin Validation (Post-Initialization) --- */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_VALIDATE_AT_STARTUP
  log_info(TAG, "Pin Validation", "Validating pin assignments...");
  #if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_PRINT_ASSIGNMENTS
    pin_validator_print_assignments(); /* Print before validating if desired */
  #endif
  err = pin_validator_validate_all(CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_HALT_ON_CONFLICT);
  if (err != ESP_OK) {
    /* Error logged by validator. If not halting, record the error. */
    RECORD_ERROR(&g_main_error_handler, err, "Pin validation failed");
    log_error(TAG, "Pin Error", "Pin validation failed! Check logs for conflicts.");
    /* Continue execution if not halting */
  } else {
    log_info(TAG, "Pin Success", "Pin validation passed.");
  }
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_VALIDATE_AT_STARTUP */

  log_info(TAG, "Initialization Complete", "Entering main application logic...");

  /* --- Main Application Logic Goes Here --- */
  /* Setup recovery error handler for the heartbeat loop */
  err = error_handler_init(&g_heartbeat_error_handler,
                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_MAX_RETRIES,
                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_RETRY_DELAY_MS,
                     CONFIG_PSTAR_KCONFIG_ERROR_DEFAULT_MAX_DELAY_MS,
                     NULL, /* No reset function for heartbeat errors */
                     NULL);
   if (err != ESP_OK) {
       /* This is less critical than main handler, log error and continue */
       RECORD_ERROR(&g_main_error_handler, err, "Failed to initialize heartbeat error handler");
       log_error(TAG, "Heartbeat Error Handler Init Failed: %s", esp_err_to_name(err));
       /* Continue without specific heartbeat error handling? */
   }


  /* Example: Log a message periodically */
  int count = 0;
  while(1) {
    log_info(TAG, "Heartbeat", "System running... Count: %d", count++);

    vTaskDelay(pdMS_TO_TICKS(5000)); /* Delay 5 seconds */

    #if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
    /* Example file write with error handling */
    /* Use static pointers defined above */
    if (atomic_load(&g_file_manager.initialized) && atomic_load(&g_sd_card.initialized)) { /* Check init status */
      if (sd_card_is_available(&g_sd_card)) { /* Checks atomic flag internally */
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "Log entry number %d", count);

        esp_err_t write_err = file_write_enqueue(&g_file_manager, "test.log", buffer);
        if (write_err != ESP_OK) {
          /* Use heartbeat error handler */
          RECORD_ERROR(&g_heartbeat_error_handler, write_err, "Failed to enqueue file write");
          log_warn(TAG, "File Write Failed", "Failed to enqueue log entry: %s", esp_err_to_name(write_err));

          /* Check retry status (can_retry handles mutex) */
          if (!error_handler_can_retry(&g_heartbeat_error_handler)) {
            log_warn(TAG, "Error Recovery", "Max retries reached for file writes, trying to flush logs");
            esp_err_t flush_err = log_flush(); /* Attempt to flush */
            if (flush_err != ESP_OK) {
                 log_error(TAG, "Recovery Error", "Log flush failed during error recovery: %s", esp_err_to_name(flush_err));
            }
            /* Reset error state to allow trying again later */
            error_handler_reset_state(&g_heartbeat_error_handler);
          }
        } else {
           /* If write succeeds, maybe reset the error handler state? */
           /* Only reset if it was actually in an error state before. */
           if (g_heartbeat_error_handler.in_error_state) {
                error_handler_reset_state(&g_heartbeat_error_handler);
           }
        }
      } else {
        log_warn(TAG, "File Write Skip", "SD card not available, skipping write.");
        /* Optional: Record an error if SD card *should* be available? */
        /* RECORD_ERROR(&g_heartbeat_error_handler, ESP_ERR_NOT_FOUND, "SD card unavailable for writing"); */
      }
    }
    #endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED */

    /* Demonstrate error counter reset after successful operations */
    if (count > 0 && count % 10 == 0) {
       /* Check if handler is in error state before resetting */
       if(g_heartbeat_error_handler.in_error_state) {
           log_info(TAG, "Error Reset", "Resetting heartbeat error state after %d iterations.", count);
           error_handler_reset_state(&g_heartbeat_error_handler); /* Handles mutex */
       }
    }

    /* --- Add other application logic here --- */

  } /* End while(1) */

/* --- Cleanup Code (never reached in this loop, but good practice) --- */
/* Fix: Enable this block for proper shutdown sequence if loop could exit */
#if 0 /* Enable if loop can exit or via shutdown hook */
  app_cleanup();
#endif
}


/**
 * @brief Application cleanup function.
 */
/* FIX: Added unused attribute */
static void __attribute__((unused)) app_cleanup(void) {
    /* Use log_xxx for messages BEFORE log_cleanup() */
    log_info(TAG, "Shutdown", "Initiating system cleanup...");

    /* Cleanup components in a reasonable reverse order */

#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
    /* Pin Validator cleanup (if any needed - currently none) */
    log_info(TAG, "Cleanup", "Cleaning up Pin Validator...");
    /* pin_validator_clear_all(); // Optional, if needed */
#endif

#if CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
    log_info(TAG, "Cleanup", "Cleaning up Time Manager...");
    time_manager_cleanup();
#endif

    /* --- SD Card Related Cleanup --- */
#if CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
    /* 1. Cleanup File Manager (depends on SD HAL & Logger) */
    if (atomic_load(&g_file_manager.initialized)) {
        log_info(TAG, "Cleanup", "Cleaning up File Manager...");
        file_write_manager_cleanup(&g_file_manager);
    }

    /* 2. Cleanup SD Card HAL (depends on Bus Manager) */
    if (atomic_load(&g_sd_card.initialized)) {
         log_info(TAG, "Cleanup", "Cleaning up SD Card HAL...");
        sd_card_cleanup(&g_sd_card); /* This also cleans up its internal bus_manager */
    }
#endif

    /* 3. Final Log Flush and Cleanup */
    /* Attempt a final flush BEFORE cleaning up the logger itself */
    log_info(TAG, "Cleanup", "Attempting final log flush...");
    log_flush();
    log_info(TAG, "Cleanup", "Cleaning up Logger...");
    log_cleanup(); /* This internally calls log_storage_cleanup */

    /* --- Cleanup Error Handlers (Now that logging is fully down) --- */
    /* Use ESP_LOGx from here on out */
    ESP_LOGI(TAG, "Cleaning up Error Handlers...");
    error_handler_deinit(&g_heartbeat_error_handler);
    error_handler_deinit(&g_main_error_handler);

    ESP_LOGI(TAG, "Shutdown Complete - System cleanup finished.");

    /* Optionally enter deep sleep or halt */
     /* esp_deep_sleep_start(); */
     esp_restart(); /* Or simply restart */
}