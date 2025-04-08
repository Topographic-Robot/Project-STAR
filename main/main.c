/* main/main.c */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h" /* Needed for default event loop */

/* Project Star Includes */
#include "pstar_log_handler.h"
#include "pstar_pin_validator.h"
#include "pstar_bus_manager.h"
#include "pstar_error_handler.h" /* Though not directly used here, SD HAL uses it */
#include "pstar_time_manager.h"  /* For logger timestamps */
#include "pstar_file_write_manager.h" /* Needed for SD logging */

/* Storage HAL Includes (Specific SPI for now) */
#include "pstar_sd_card_hal.h" /* Includes types and SPI header */

/* Kconfig */
#include "sdkconfig.h"

/* Static Variables */
static const char* TAG = "AppMain";
static pstar_bus_manager_t g_bus_manager;
#ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
static sd_card_hal_t g_sd_card;
#endif
#if defined(CONFIG_PSTAR_KCONFIG_FILE_MANAGER_ENABLED) && defined(CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED)
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
    /* We call log_init again later if SD card mounts successfully */
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

    /* Initialize Bus Manager */
    ret = pstar_bus_manager_init(&g_bus_manager, "MainBusMgr");
    if (ret != ESP_OK) {
        log_error(TAG, "Init Error", "Bus Manager initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Initialize Time Manager (for logger timestamps) */
    #ifdef CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED
        ret = time_manager_init();
        if (ret != ESP_OK) {
            log_error(TAG, "Init Error", "Time Manager initialization failed: %s", esp_err_to_name(ret));
            /* Continue, logging might lack timestamps */
        }
        /* Wait a bit for time sync to potentially complete for better timestamps */
        /* In a real app, use time_manager_is_initialized() check */
        log_info(TAG, "Init", "Waiting briefly for time sync...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    #endif

    /* --- SD Card SPI HAL Initialization & Mount --- */
    #ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED
        #ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED
            log_info(TAG, "SD Init", "Initializing SD Card HAL for SPI...");

            /* Manually populate necessary parts of g_sd_card for SPI init */
            memset(&g_sd_card, 0, sizeof(g_sd_card));
            g_sd_card.bus_manager = g_bus_manager; /* Assign the initialized manager */
            g_sd_card.spi_mode_enabled = true;
            g_sd_card.tag = "SD_HAL"; /* Assign a logging tag */
            g_sd_card.pin_config.spi_cs_pin   = CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_CS_GPIO;
            g_sd_card.pin_config.spi_do_pin   = CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_DO_GPIO;
            g_sd_card.pin_config.spi_di_pin   = CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_DI_GPIO;
            g_sd_card.pin_config.spi_sclk_pin = CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_CLK_GPIO;
            g_sd_card.mount_path = CONFIG_PSTAR_KCONFIG_SD_CARD_MOUNT_POINT;
            /* Assign non-const members */
            g_sd_card.max_files               = CONFIG_PSTAR_KCONFIG_SD_CARD_MAX_FILES;
            g_sd_card.allocation_unit_size    = CONFIG_PSTAR_KCONFIG_SD_CARD_ALLOCATION_UNIT_SIZE;


            /* Initialize the SPI interface specifically */
            ret = pstar_sd_card_spi_init(&g_sd_card);
            if (ret != ESP_OK) {
                log_error(TAG, "SD Init Error", "SD Card SPI interface initialization failed: %s", esp_err_to_name(ret));
            } else {
                log_info(TAG, "SD SPI Init", "SD Card SPI interface initialized.");

                /* Attempt to mount the card via SPI */
                ret = pstar_sd_card_spi_mount(&g_sd_card);
                if (ret != ESP_OK) {
                    log_error(TAG, "SD Mount Error", "Failed to mount SD card via SPI: %s", esp_err_to_name(ret));
                } else {
                    log_info(TAG, "SD Mount", "SD card mounted successfully via SPI at %s", g_sd_card.mount_path);

                    /* --- Initialize File Manager and Full Logger (ONLY if SD mounted) --- */
                    #if defined(CONFIG_PSTAR_KCONFIG_FILE_MANAGER_ENABLED) && defined(CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED)
                        log_info(TAG, "FM Init", "Initializing File Write Manager...");
                        ret = file_write_manager_init(&g_file_manager, &g_sd_card, NULL); /* Pass SD HAL instance */
                        if (ret != ESP_OK) {
                            log_error(TAG, "FM Init Error", "File Write Manager initialization failed: %s", esp_err_to_name(ret));
                        } else {
                            log_info(TAG, "Logger Full Init", "Performing full logger initialization for SD card...");
                            /* Call log_init AGAIN with valid pointers */
                            ret = log_init(&g_file_manager, &g_sd_card);
                            if (ret != ESP_OK) {
                                log_error(TAG, "Logger Full Init Error", "Full logger initialization failed: %s", esp_err_to_name(ret));
                            } else {
                                g_sd_logging_enabled = true;
                                log_info(TAG, "Logger Ready", "Logger fully initialized. SD Card logging is now ACTIVE.");
                                /* Log a message specifically after full init */
                                log_warn(TAG, "SD Logging Test", "This message should be logged to the SD card!");
                            }
                        }
                    #else
                        log_warn(TAG, "SD Logging Skip", "File Manager or SD Logging disabled in Kconfig. Logs will only go to console.");
                    #endif
                }
            }
        #else
            log_warn(TAG, "SD Init Skip", "SD Card enabled but SPI mode is disabled.");
        #endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED */
    #endif /* CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED */

    /* --- Post-Initialization Validation --- */
    #if defined(CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED) && defined(CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_VALIDATE_AT_STARTUP)
        log_info(TAG, "Validation", "Running pin validation...");
        /* Determine halt flag based on Kconfig */
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
        /* Log a message periodically */
        log_info(TAG, "Heartbeat", "Main loop running... Counter: %d", counter++);

        /* Flush logs periodically if SD logging is enabled */
        if (g_sd_logging_enabled) {
            log_debug(TAG, "Log Flush", "Attempting to flush logs to SD card...");
            ret = log_flush();
            if (ret != ESP_OK) {
                log_warn(TAG, "Log Flush Error", "Log flush failed: %s", esp_err_to_name(ret));
                /* SD card might have been removed or encountered an error */
                /* In a real app, you might need to handle this (e.g., stop trying to flush) */
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10000)); /* Log every 10 seconds */
    }

    /* --- Cleanup (won't be reached in this example loop) --- */
    /* log_info(TAG, "Cleanup", "Starting cleanup..."); */
    /* #if defined(CONFIG_PSTAR_KCONFIG_FILE_MANAGER_ENABLED) && defined(CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED) */
    /*     if (g_sd_logging_enabled) { file_write_manager_cleanup(&g_file_manager); } */
    /* #endif */
    /* #ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED */
    /*     #ifdef CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED */
    /*         if (g_sd_card.card != NULL) { pstar_sd_card_spi_unmount(&g_sd_card); } */
    /*         pstar_sd_card_spi_deinit(&g_sd_card); */
    /*     #endif */
    /* #endif */
    /* #ifdef CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED */
    /*     time_manager_cleanup(); */
    /* #endif */
    /* pstar_bus_manager_deinit(&g_bus_manager); */
    /* log_cleanup(); */
    /* ESP_ERROR_CHECK(esp_event_loop_delete_default()); */
    /* ESP_ERROR_CHECK(nvs_flash_deinit()); */
    /* log_info(TAG, "Cleanup", "Cleanup finished."); */
}
