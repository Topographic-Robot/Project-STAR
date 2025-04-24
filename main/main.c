/* main/main.c */

#include "pstar_bus_manager.h"
#include "pstar_pca9685_hal.h"
#include "pstar_pin_validator.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#if CONFIG_PSTAR_KCONFIG_BH1750_ENABLED
#include "pstar_bh1750_hal.h"
#endif
#if CONFIG_PSTAR_KCONFIG_JTAG_ENABLED
#include "pstar_jtag.h"
#endif

static const char* TAG = "Project Star";

/* --- Global Bus Manager Instance --- */
static pstar_bus_manager_t g_bus_manager;

/* --- Global Handles for Default PCA9685 Boards --- */
#if CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED && (CONFIG_PSTAR_KCONFIG_PCA9685_DEFAULT_INIT_COUNT > 0)
#define NUM_DEFAULT_PCA9685 CONFIG_PSTAR_KCONFIG_PCA9685_DEFAULT_INIT_COUNT
static pstar_pca9685_hal_handle_t g_pca9685_handles[NUM_DEFAULT_PCA9685] = {NULL};
#else
#define NUM_DEFAULT_PCA9685 0
#endif

void app_main(void)
{
  ESP_LOGI(TAG, "Application Start: Project Star (Servo Sweep)");

  esp_err_t ret                 = ESP_OK;
  bool      manager_initialized = false;
#if CONFIG_PSTAR_KCONFIG_BH1750_ENABLED
  bh1750_hal_handle_t bh1750_handle = NULL;
#endif

  /* 1. Initialize the Bus Manager */
  ESP_LOGI(TAG, "Initializing Bus Manager...");
  ret = pstar_bus_manager_init(&g_bus_manager, "MainBusMgr");
  ESP_GOTO_ON_ERROR(ret,
                    fatal_error,
                    TAG,
                    "Failed to initialize Bus Manager: %s",
                    esp_err_to_name(ret));
  manager_initialized = true;

  /* 2. Register Pins (if validator enabled) */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  ESP_LOGI(TAG, "Registering component pins with validator...");
#if CONFIG_PSTAR_KCONFIG_BH1750_ENABLED
  ret = pstar_bh1750_register_kconfig_pins();
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to register BH1750 pins: %s", esp_err_to_name(ret));
#endif
#if CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED
  ret = pstar_pca9685_register_kconfig_pins();
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to register PCA9685 default pins: %s",
                    esp_err_to_name(ret));
#endif
#if CONFIG_PSTAR_KCONFIG_JTAG_ENABLED
  ret = pstar_jtag_register_kconfig_pins();
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to register JTAG pins: %s", esp_err_to_name(ret));
#endif
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED */

  /* 3. Create and Initialize Devices */
#if CONFIG_PSTAR_KCONFIG_BH1750_ENABLED
  ret = pstar_bh1750_hal_create_kconfig_default(&g_bus_manager, &bh1750_handle);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to create/init BH1750: %s", esp_err_to_name(ret));
#else
  ESP_LOGI(TAG, "BH1750 component disabled. Skipping initialization.");
#endif

#if NUM_DEFAULT_PCA9685 > 0
  ESP_LOGI(TAG, "Initializing %d default PCA9685 board(s)...", NUM_DEFAULT_PCA9685);
  ret = pstar_pca9685_hal_create_multiple_defaults(&g_bus_manager,
                                                   NUM_DEFAULT_PCA9685,
                                                   g_pca9685_handles);
  if (ret != ESP_OK) {
    ESP_LOGE(
      TAG,
      "Failed to initialize one or more default PCA9685 boards (first error: %s). Check logs.",
      esp_err_to_name(ret));
    /* Continue if some boards initialized, but log the error */
  } else {
    ESP_LOGI(TAG, "Successfully initialized %d default PCA9685 board(s).", NUM_DEFAULT_PCA9685);
  }
#elif CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED
  ESP_LOGI(TAG, "Default PCA9685 initialization count is 0. Skipping.");
#endif

  /* 4. Validate All Registered Pins (if validator enabled) */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  ESP_LOGI(TAG, "Validating all registered pins...");
  ret = pstar_validate_pins();
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Pin validation failed: %s", esp_err_to_name(ret));
  ESP_LOGI(TAG, "Pin validation successful.");
#endif

  /* 5. Enable Output and Prepare for Sweep */
#if NUM_DEFAULT_PCA9685 > 0
  ESP_LOGI(TAG, "Enabling output for initialized PCA9685 boards...");
  bool any_board_ready = false;
  for (int board_idx = 0; board_idx < NUM_DEFAULT_PCA9685; ++board_idx) {
    if (g_pca9685_handles[board_idx]) { /* Check if handle is valid */
      ret = pstar_pca9685_hal_output_enable(g_pca9685_handles[board_idx]);
      if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { /* Ignore error if OE not configured */
        ESP_LOGE(TAG,
                 "Failed to enable output for PCA9685 board %d: %s",
                 board_idx,
                 esp_err_to_name(ret));
      } else if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Output enabled for PCA9685 board %d", board_idx);
        any_board_ready = true; /* Mark that at least one board is ready */
      } else {
        /* Output enable failed likely because OE pin not configured, still consider ready */
        ESP_LOGI(TAG, "Output enable skipped for PCA9685 board %d (OE not configured)", board_idx);
        any_board_ready = true;
      }
    }
  }

  /* Only proceed if at least one board is ready */
  if (!any_board_ready) {
    ESP_LOGE(TAG, "No PCA9685 boards initialized or output enabled. Halting.");
    goto fatal_error; /* Or handle error appropriately */
  }

  float     servo_max_angle = (float)CONFIG_PSTAR_KCONFIG_PCA9685_SERVO_ANGLE_RANGE;
  float     current_angle   = 0.0f;
  float     angle_step      = 1.0f; /* Degrees to step each iteration */
  int       sweep_direction = 1;    /* 1 for increasing, -1 for decreasing */
  const int step_delay_ms   = 30;   /* Delay between steps (adjust for speed) */

  ESP_LOGI(TAG, "Setup complete. Starting servo sweep (0 to %.1f deg).", servo_max_angle);

  /* 6. Servo Sweep Loop */
  while (1) {
    /* Update all servos on all initialized boards using the HAL function */
    for (int board_idx = 0; board_idx < NUM_DEFAULT_PCA9685; ++board_idx) {
      if (g_pca9685_handles[board_idx]) { /* Check if board handle is valid */
        /* Use the HAL function to set the angle for all channels on this board */
        esp_err_t set_all_ret =
          pstar_pca9685_hal_set_all_servos_angle(g_pca9685_handles[board_idx], current_angle);

        if (set_all_ret != ESP_OK) {
          ESP_LOGE(TAG,
                   "Sweep: Failed set board %d to angle %.1f: %s",
                   board_idx,
                   current_angle,
                   esp_err_to_name(set_all_ret));
          /* Optional: Add error handling per board if needed */
        }
      }
    }
    ESP_LOGD(TAG, "Sweep Angle: %.1f deg", current_angle);

    /* Update angle for next iteration */
    current_angle += (angle_step * sweep_direction);

    /* Check limits and reverse direction */
    if (current_angle >= servo_max_angle) {
      current_angle   = servo_max_angle; /* Clamp to max */
      sweep_direction = -1;              /* Reverse direction */
      ESP_LOGI(TAG, "Sweep: Reached max angle (%.1f). Reversing.", current_angle);
      vTaskDelay(pdMS_TO_TICKS(500)); /* Pause at the end */
    } else if (current_angle <= 0.0f) {
      current_angle   = 0.0f; /* Clamp to min */
      sweep_direction = 1;    /* Reverse direction */
      ESP_LOGI(TAG, "Sweep: Reached min angle (%.1f). Reversing.", current_angle);
      vTaskDelay(pdMS_TO_TICKS(500)); /* Pause at the end */
    }

    /* Delay for sweep speed control */
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));

  } /* End while(1) */

#else  /* Case where NUM_DEFAULT_PCA9685 is 0 */
  ESP_LOGW(TAG, "No PCA9685 boards configured. Entering idle loop.");
  while (1) {
    vTaskDelay(portMAX_DELAY); /* Keep the task alive but idle */
  }
#endif /* NUM_DEFAULT_PCA9685 > 0 */

cleanup: /* Label for GOTO on error during setup */
  ESP_LOGW(TAG, "Starting cleanup sequence due to error...");

  /* Deinitialize devices */
#if NUM_DEFAULT_PCA9685 > 0
  ESP_LOGI(TAG, "Deinitializing default PCA9685 board(s)...");
  for (int i = 0; i < NUM_DEFAULT_PCA9685; ++i) {
    if (g_pca9685_handles[i]) {
      /* Attempt to disable output before deinit, ignore errors */
      pstar_pca9685_hal_output_disable(g_pca9685_handles[i]);
      esp_err_t deinit_ret =
        pstar_pca9685_hal_deinit(g_pca9685_handles[i],
                                 true,
                                 false); /* Sleep=true, DisableOutput=false (already attempted) */
      if (deinit_ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 HAL deinit failed for board %d: %s", i, esp_err_to_name(deinit_ret));
      }
      g_pca9685_handles[i] = NULL;
    }
  }
#endif

#if CONFIG_PSTAR_KCONFIG_BH1750_ENABLED
  if (bh1750_handle) {
    ESP_LOGI(TAG, "Deinitializing BH1750 HAL...");
    bool power_down = false;
#ifdef CONFIG_PSTAR_KCONFIG_BH1750_POWER_SAVE_MODE
    power_down = true;
#endif
    esp_err_t deinit_ret = pstar_bh1750_hal_deinit(bh1750_handle, power_down);
    if (deinit_ret != ESP_OK) {
      ESP_LOGE(TAG, "BH1750 HAL deinit failed: %s", esp_err_to_name(deinit_ret));
    }
    bh1750_handle = NULL;
  }
#endif

  /* Deinitialize Bus Manager */
  if (manager_initialized) {
    ESP_LOGI(TAG, "Deinitializing Bus Manager...");
    esp_err_t deinit_ret = pstar_bus_manager_deinit(&g_bus_manager);
    if (deinit_ret != ESP_OK) {
      ESP_LOGE(TAG, "Bus Manager deinit reported errors: %s", esp_err_to_name(deinit_ret));
    }
    manager_initialized = false;
  }

  /* Free pin validator resources */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  ESP_LOGI(TAG, "Freeing pin validator resources...");
  esp_err_t pin_validator_ret = pstar_free_pin_validator();
  if (pin_validator_ret != ESP_OK) {
    ESP_LOGE(TAG, "Pin validator free failed: %s", esp_err_to_name(pin_validator_ret));
  }
#endif

fatal_error: /* Label for GOTO on critical error during setup */
  ESP_LOGE(TAG, "Application ending due to fatal error or normal exit from cleanup.");
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}