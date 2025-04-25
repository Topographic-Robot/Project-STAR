/* pstar_examples/pca9685_example.c */

#include "sdkconfig.h"

/* Only include and compile if component enabled */
#ifdef CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED

#include "pstar_bus_manager.h"
#include "pstar_pca9685_hal.h"
#include "pstar_pin_validator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "pca9685_example.h"
// sdkconfig.h is already included

static const char* TAG_EX = "PCA9685_MultiBoard_Example";

/* --- Determine Number of Default Boards from Kconfig --- */
/* Use the macro derived from Kconfig */
#define NUM_DEFAULT_PCA9685 CONFIG_PSTAR_KCONFIG_PCA9685_DEFAULT_INIT_COUNT

/* Example Configuration */
#define SWEEP_STEP_DEG 1   /* Angle increment/decrement per step (adjust for speed) */
#define STEP_DELAY_MS 15   /* Delay between angle steps (adjust for speed) */
#define PAUSE_DELAY_MS 500 /* Delay at the end limits */

void example_pca9685_app_main(void)
{
  ESP_LOGI(TAG_EX,
           "Running: PCA9685 Multi-Board Servo Sweep Example (%d board(s))",
           NUM_DEFAULT_PCA9685);
  esp_err_t           ret = ESP_OK;
  pstar_bus_manager_t example_bus_manager;
  bool                manager_initialized = false;
  bool                validator_used      = false; /* Track if validator was active */

/* Array to hold handles for all initialized boards */
#if NUM_DEFAULT_PCA9685 > 0 /* Array declaration only if needed */
  pstar_pca9685_hal_handle_t pca9685_handles[NUM_DEFAULT_PCA9685] = {NULL};
#else
  /* Define a dummy pointer to avoid compiler errors if the array is never used, */
  /* although the logic below should prevent access anyway. */
  pstar_pca9685_hal_handle_t* pca9685_handles = NULL;
#endif

/* --- Setup Step 1: Register Example Pins (if validator enabled AND boards configured) --- */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && (NUM_DEFAULT_PCA9685 > 0)
  validator_used = true;
  ESP_LOGI(TAG_EX, "Registering PCA9685 example pins...");
  /* Use Kconfig pins for the example. This registers I2C and potentially OE pin for the defaults. */
  /* This function is already guarded internally by the HAL C file */
  ret = pstar_pca9685_register_kconfig_pins();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to register PCA9685 pins: %s", esp_err_to_name(ret));
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
#elif NUM_DEFAULT_PCA9685 > 0 /* If boards configured but validator disabled */
  ESP_LOGW(TAG_EX, "Pin Validator disabled. Skipping pin checks for example.");
  validator_used = false;
#else                         /* If no boards configured */
  ESP_LOGI(TAG_EX, "No default PCA9685 boards configured. Skipping pin registration/validation.");
  validator_used = false;
#endif

  /* --- Setup Step 3: Init Bus Manager (local to this example) --- */
  ESP_LOGI(TAG_EX, "Initializing Example Bus Manager...");
  ret = pstar_bus_manager_init(&example_bus_manager, "PCA9685_Example_BusMgr");
  if (ret != ESP_OK) {
    ESP_LOGE(TAG_EX, "Failed to init example bus manager: %s", esp_err_to_name(ret));
    goto example_fail; /* Cannot proceed */
  }
  manager_initialized = true;

  /* --- Setup Step 4: Create MULTIPLE PCA9685 devices using Kconfig defaults --- */
#if NUM_DEFAULT_PCA9685 > 0 /* Wrap initialization logic */
  ESP_LOGI(TAG_EX, "Initializing %d default PCA9685 board(s)...", NUM_DEFAULT_PCA9685);
  /* This function is already guarded internally by the HAL C file */
  ret = pstar_pca9685_hal_create_multiple_defaults(&example_bus_manager,
                                                   NUM_DEFAULT_PCA9685,
                                                   pca9685_handles); /* Pass the array */
  if (ret != ESP_OK) {
    ESP_LOGE(
      TAG_EX,
      "Failed to initialize one or more default PCA9685 boards (first error: %s). Check logs.",
      esp_err_to_name(ret));
    /* Continue if some boards initialized, but log the error */
  } else {
    ESP_LOGI(TAG_EX, "Successfully initialized %d default PCA9685 board(s).", NUM_DEFAULT_PCA9685);
  }
#else
  ESP_LOGI(TAG_EX, "No default PCA9685 boards to initialize (Count is 0).");
#endif

  /* --- Setup Step 5: Enable Output and Check Readiness --- */
  bool any_board_ready = false;
#if NUM_DEFAULT_PCA9685 > 0 /* Wrap output enable logic */
  ESP_LOGI(TAG_EX, "Enabling output for initialized PCA9685 boards...");
  for (int board_idx = 0; board_idx < NUM_DEFAULT_PCA9685; ++board_idx) {
    if (pca9685_handles[board_idx]) { /* Check if handle is valid (init succeeded) */
      esp_err_t enable_ret = pstar_pca9685_hal_output_enable(pca9685_handles[board_idx]);
      if (enable_ret != ESP_OK &&
          enable_ret != ESP_ERR_INVALID_STATE) { /* Ignore error if OE not configured */
        ESP_LOGE(TAG_EX,
                 "Failed to enable output for PCA9685 board %d: %s",
                 board_idx,
                 esp_err_to_name(enable_ret));
      } else if (enable_ret == ESP_OK) {
        ESP_LOGI(TAG_EX, "Output enabled for PCA9685 board %d", board_idx);
        any_board_ready = true; /* Mark that at least one board is ready */
      } else {
        /* Output enable failed likely because OE pin not configured, still consider ready */
        ESP_LOGI(TAG_EX,
                 "Output enable skipped for PCA9685 board %d (OE not configured)",
                 board_idx);
        any_board_ready = true;
      }
    } else {
      ESP_LOGW(TAG_EX, "Skipping output enable for board %d (initialization failed).", board_idx);
    }
  }

  /* Only proceed if at least one board is ready */
  if (!any_board_ready) {
    ESP_LOGE(TAG_EX, "No PCA9685 boards initialized or output enabled. Halting.");
    goto example_fail;
  }
#else  /* Case where NUM_DEFAULT_PCA9685 is 0 */
  ESP_LOGW(TAG_EX, "No PCA9685 boards configured. Entering idle state.");
  goto example_fail; /* Go to cleanup and halt */
#endif /* NUM_DEFAULT_PCA9685 > 0 */

  /* --- Setup Step 6: Example Loop (Multi-Board Continuous Servo Sweep) --- */
#if NUM_DEFAULT_PCA9685 > 0 /* Wrap the main example logic */
  /* Use Kconfig value for servo angle range */
  const float servo_max_angle = (float)CONFIG_PSTAR_KCONFIG_PCA9685_SERVO_ANGLE_RANGE;
  float       current_angle   = 0.0f;
  int         sweep_direction = 1; /* 1 for increasing, -1 for decreasing */

  ESP_LOGI(
    TAG_EX,
    "Starting continuous servo sweep on ALL channels of ALL %d initialized boards (0 to %.0f degrees)...",
    NUM_DEFAULT_PCA9685,
    servo_max_angle);

  while (1) {
    /* --- Update all servos on all initialized boards --- */
    for (int board_idx = 0; board_idx < NUM_DEFAULT_PCA9685; ++board_idx) {
      if (pca9685_handles[board_idx]) { /* Check if board handle is valid */
        /* Use the HAL function to set the angle for ALL channels on THIS board */
        esp_err_t set_all_ret =
          pstar_pca9685_hal_set_all_servos_angle(pca9685_handles[board_idx], current_angle);

        if (set_all_ret != ESP_OK) {
          ESP_LOGE(TAG_EX,
                   "Sweep: Failed set board %d to angle %.1f: %s",
                   board_idx,
                   current_angle,
                   esp_err_to_name(set_all_ret));
          /* Optional: Add error handling per board if needed */
        }
      }
    }
    ESP_LOGD(TAG_EX, "Sweep Angle: %.1f deg", current_angle);

    /* --- Update angle for next iteration --- */
    current_angle += ((float)SWEEP_STEP_DEG * sweep_direction);

    /* --- Check limits and reverse direction --- */
    if (current_angle >= servo_max_angle) {
      current_angle   = servo_max_angle; /* Clamp to max */
      sweep_direction = -1;              /* Reverse direction */
      ESP_LOGD(TAG_EX, "Sweep: Reached max angle (%.1f). Reversing.", current_angle);
      vTaskDelay(pdMS_TO_TICKS(PAUSE_DELAY_MS)); /* Pause at the end */
    } else if (current_angle <= 0.0f) {
      current_angle   = 0.0f; /* Clamp to min */
      sweep_direction = 1;    /* Reverse direction */
      ESP_LOGD(TAG_EX, "Sweep: Reached min angle (%.1f). Reversing.", current_angle);
      vTaskDelay(pdMS_TO_TICKS(PAUSE_DELAY_MS)); /* Pause at the end */
    }

    /* --- Delay for sweep speed control --- */
    vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));

  } /* End while(1) */
#endif /* NUM_DEFAULT_PCA9685 > 0 (End wrap for example logic) */

example_fail:
  /* Cleanup resources if setup failed or loop exited */
  ESP_LOGW(TAG_EX, "Example setup failed or ending. Cleaning up and halting.");

#if NUM_DEFAULT_PCA9685 > 0 /* Wrap cleanup logic accessing handles */
  ESP_LOGI(TAG_EX, "Deinitializing default PCA9685 board(s)...");
  for (int i = 0; i < NUM_DEFAULT_PCA9685; ++i) {
    if (pca9685_handles[i]) { /* Check handle is valid */
      /* Attempt to disable output before deinit, ignore errors */
      pstar_pca9685_hal_output_disable(pca9685_handles[i]);
      esp_err_t deinit_ret =
        pstar_pca9685_hal_deinit(pca9685_handles[i],
                                 true,
                                 false); /* Sleep=true, DisableOutput=false (already attempted) */
      if (deinit_ret != ESP_OK) {
        ESP_LOGE(TAG_EX,
                 "PCA9685 HAL deinit failed for board %d: %s",
                 i,
                 esp_err_to_name(deinit_ret));
      }
      pca9685_handles[i] = NULL; /* Clear handle after deinit */
    }
  }
#endif /* NUM_DEFAULT_PCA9685 > 0 */

  if (manager_initialized) {
    ESP_LOGI(TAG_EX, "Deinitializing Example Bus Manager...");
    esp_err_t deinit_ret = pstar_bus_manager_deinit(&example_bus_manager);
    if (deinit_ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "Bus Manager deinit reported errors: %s", esp_err_to_name(deinit_ret));
    }
  }

#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  if (validator_used) {
    ESP_LOGI(TAG_EX, "Freeing pin validator resources...");
    esp_err_t pin_validator_ret = pstar_free_pin_validator();
    if (pin_validator_ret != ESP_OK) {
      ESP_LOGE(TAG_EX, "Pin validator free failed: %s", esp_err_to_name(pin_validator_ret));
    }
  }
#endif
  /* Halt */
  ESP_LOGI(TAG_EX, "Example finished cleanup. Halting task.");
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}

#else /* CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED */

/* Provide a stub function if the component is disabled */
#include "esp_log.h"
/* Include FreeRTOS headers needed ONLY for the halt loop in the stub */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG_EX_STUB = "PCA9685_Example";
void               example_pca9685_app_main(void)
{
  ESP_LOGE(TAG_EX_STUB,
           "PCA9685 Example is selected in Kconfig, but the PCA9685 component is disabled!");
  ESP_LOGE(
    TAG_EX_STUB,
    "Please enable 'PSTAR_KCONFIG_PCA9685_ENABLED' or select a different example/Main Application.");
  while (1) {
    vTaskDelay(portMAX_DELAY); /* Halt */
  }
}

#endif /* CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED */