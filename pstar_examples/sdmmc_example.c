/* pstar_examples/sdmmc_example.c */

#include "sdkconfig.h"

/* Only compile if SDMMC HAL component is enabled */
#ifdef CONFIG_PSTAR_KCONFIG_SDMMC_ENABLED

#include "pstar_bus_manager.h"
#include "pstar_pin_validator.h"
#include "pstar_sdmmc_hal.h" /* Include the SDMMC HAL header */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h> /* For file operations (fopen, fprintf, etc.) */
#include <string.h>
#include <sys/stat.h> /* For stat() */
#include <sys/unistd.h>

#include "esp_check.h" /* For ESP_ERROR_CHECK */
#include "esp_log.h"
#include "esp_timer.h" /* Include esp_timer.h */
#include "sdmmc_cmd.h" /* For sdmmc_card_print_info */
#include "sdmmc_example.h"

static const char* TAG_EX = "SDMMC_Example";

/* Helper function to create the full path */
static void
build_filepath(const char* mount_point, const char* filename, char* out_path, size_t max_len)
{
  snprintf(out_path, max_len, "%s/%s", mount_point, filename);
}

void example_sdmmc_app_main(void)
{
  /* Determine mode string from Kconfig for logging */
#if CONFIG_PSTAR_KCONFIG_SDMMC_MODE_SPI
  const char* mode_str = "SPI";
#elif CONFIG_PSTAR_KCONFIG_SDMMC_MODE_SDMMC_1BIT
  const char* mode_str = "SDMMC 1-bit";
#elif CONFIG_PSTAR_KCONFIG_SDMMC_MODE_SDMMC_4BIT
  const char* mode_str = "SDMMC 4-bit";
#else
  const char* mode_str = "Unknown";
#endif

  ESP_LOGI(TAG_EX, "Running: SD Card Example (Mode: %s)", mode_str);
  esp_err_t                ret = ESP_OK;
  pstar_bus_manager_t      example_bus_manager; /* Needed for SPI mode */
  pstar_sdmmc_hal_handle_t sd_handle           = NULL;
  bool                     manager_initialized = false;
  bool                     validator_used      = false; /* Track if validator was active */
  const char*              mount_point         = NULL;

/* --- Setup Step 1: Register Example Pins (if validator enabled) --- */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  validator_used = true;
  ESP_LOGI(TAG_EX, "Registering SDMMC example pins (Mode: %s)...", mode_str);
  /* Use Kconfig pins for the example - this function now handles mode internally */
  ret = pstar_sdmmc_register_kconfig_pins();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to register SDMMC pins: %s", esp_err_to_name(ret));
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

  /* --- Setup Step 3: Init Bus Manager (Required for SPI mode init) --- */
  ESP_LOGI(TAG_EX, "Initializing Example Bus Manager...");
  ret = pstar_bus_manager_init(&example_bus_manager, "SDMMC_Example_BusMgr");
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to init example bus manager: %s", esp_err_to_name(ret));
    goto example_fail; /* Cannot proceed */
  }
  manager_initialized = true;

  /* --- Setup Step 4: Create SDMMC HAL device --- */
  ESP_LOGI(TAG_EX, "Initializing SDMMC HAL (Mode: %s)...", mode_str);
  /* This function now handles mode selection internally based on Kconfig */
  ret = pstar_sdmmc_hal_create_kconfig_default(&example_bus_manager, &sd_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to create SDMMC default: %s", esp_err_to_name(ret));
    /* Specific error handling */
    if (ret == ESP_ERR_NOT_FOUND) {
      ESP_LOGE(TAG_EX, "SD Card not detected or initialization failed.");
    }
    /* --- FIXED: Use #ifdef for the Kconfig check --- */
    else if (ret == ESP_FAIL) {
#ifdef CONFIG_PSTAR_KCONFIG_SDMMC_FORMAT_IF_FAIL
      /* This block only compiles if the Kconfig option was checked */
      ESP_LOGW(TAG_EX, "Mounting failed, card might have been formatted.");
      /* Potentially add logic here if needed when formatting happened */
#else
      /* This block compiles if the Kconfig option was NOT checked */
      ESP_LOGE(TAG_EX, "Mounting failed and formatting was disabled.");
#endif
    }
    goto example_fail; /* Go to cleanup if init failed */
  }
  ESP_LOGI(TAG_EX, "SDMMC HAL Initialized Successfully.");
  mount_point = pstar_sdmmc_hal_get_mount_point(sd_handle);
  ESP_LOGI(TAG_EX, "SD Card mounted at: %s", mount_point ? mount_point : "UNKNOWN");

  /* --- Setup Step 5: Example File Operations --- */
  if (mount_point) {
    char filepath[128]; /* Buffer for full file path */

    /* --- Write File --- */
    const char* filename_write = "hello.txt";
    build_filepath(mount_point, filename_write, filepath, sizeof(filepath));
    ESP_LOGI(TAG_EX, "Opening file for writing: %s", filepath);
    FILE* f = fopen(filepath, "w");
    if (f == NULL) {
      ESP_LOGE(TAG_EX, "Failed to open file for writing");
    } else {
      fprintf(f, "Hello from Project-STAR!\n");
      fprintf(f, "SD Card HAL Example (Mode: %s).\n", mode_str);
      fprintf(f, "Timestamp: %lld\n", esp_timer_get_time());
      fclose(f);
      ESP_LOGI(TAG_EX, "File written successfully");
    }

    /* --- Read File --- */
    const char* filename_read = "hello.txt"; /* Same file */
    build_filepath(mount_point, filename_read, filepath, sizeof(filepath));
    ESP_LOGI(TAG_EX, "Opening file for reading: %s", filepath);
    f = fopen(filepath, "r");
    if (f == NULL) {
      ESP_LOGE(TAG_EX, "Failed to open file for reading");
    } else {
      char line[64];
      ESP_LOGI(TAG_EX, "Reading from file:");
      while (fgets(line, sizeof(line), f) != NULL) {
        /* Remove potential newline character from fgets */
        line[strcspn(line, "\n")] = 0;
        ESP_LOGI(TAG_EX, "  Read: '%s'", line);
      }
      fclose(f);
    }

    /* --- Get File Status --- */
    struct stat st;
    if (stat(filepath, &st) == 0) {
      ESP_LOGI(TAG_EX, "File stats for %s:", filepath);
      ESP_LOGI(TAG_EX, "  Size: %ld bytes", (long)st.st_size);
      ESP_LOGI(TAG_EX, "  Mode: %lo (octal)", (unsigned long)st.st_mode);
    } else {
      ESP_LOGE(TAG_EX, "Failed to get file status for %s", filepath);
    }

    /* --- Rename File --- */
    const char* filename_new = "renamed.txt";
    char        new_filepath[128];
    build_filepath(mount_point, filename_new, new_filepath, sizeof(new_filepath));
    ESP_LOGI(TAG_EX, "Renaming %s to %s", filepath, new_filepath);
    if (rename(filepath, new_filepath) != 0) {
      ESP_LOGE(TAG_EX, "Rename failed");
    } else {
      ESP_LOGI(TAG_EX, "Rename successful");
      /* Verify new file exists */
      if (stat(new_filepath, &st) == 0) {
        ESP_LOGI(TAG_EX, "Verified renamed file exists.");
      } else {
        ESP_LOGW(TAG_EX, "Could not stat renamed file.");
      }
      /* Try deleting the renamed file */
      ESP_LOGI(TAG_EX, "Deleting %s", new_filepath);
      if (unlink(new_filepath) != 0) {
        ESP_LOGE(TAG_EX, "Delete failed");
      } else {
        ESP_LOGI(TAG_EX, "Delete successful");
      }
    }
  } else {
    ESP_LOGE(TAG_EX, "Mount point is NULL, cannot perform file operations.");
  }

  ESP_LOGI(TAG_EX, "SD Card Example finished file operations.");

example_fail:
  /* Cleanup resources */
  ESP_LOGW(TAG_EX, "Example ending. Cleaning up and halting.");
  if (sd_handle) {
    esp_err_t deinit_ret = pstar_sdmmc_hal_deinit(sd_handle);
    if (deinit_ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "SDMMC HAL deinit failed: %s", esp_err_to_name(deinit_ret));
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

#else /* CONFIG_PSTAR_KCONFIG_SDMMC_ENABLED */

/* Provide a stub function if the component is disabled */
#include "esp_log.h"
/* Include FreeRTOS headers needed ONLY for the halt loop in the stub */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG_EX_STUB = "SDMMC_Example";
void               example_sdmmc_app_main(void)
{
  ESP_LOGE(TAG_EX_STUB,
           "SDMMC Example is selected in Kconfig, but the SDMMC HAL component is disabled!");
  ESP_LOGE(
    TAG_EX_STUB,
    "Please enable 'PSTAR_KCONFIG_SDMMC_ENABLED' or select a different example/Main Application.");
  while (1) {
    vTaskDelay(portMAX_DELAY); /* Halt */
  }
}

#endif /* CONFIG_PSTAR_KCONFIG_SDMMC_ENABLED */