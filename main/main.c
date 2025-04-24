/* main/main.c */

// Include sdkconfig first to get Kconfig defines
#include "sdkconfig.h"

// Standard Includes
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h> // For strcmp in pin validator callback example

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP-IDF Drivers/Libs
#include "driver/gpio.h" // Include GPIO driver for explicit config

#include "esp_adc/adc_oneshot.h" // New Oneshot ADC driver
#include "esp_check.h"           // For ESP_GOTO_ON_ERROR etc.
#include "esp_log.h"
/* #include "esp_adc_cal.h" // ADC Calibration - Optional, can be added for better accuracy (needs different API too) */

// Project Star Components
#include "pstar_bus_manager.h"
#include "pstar_pca9685_hal.h" // Always include PCA9685 for servo control
#include "pstar_pin_validator.h"

// Conditionally include other components if enabled
#if CONFIG_PSTAR_KCONFIG_BH1750_ENABLED
#include "pstar_bh1750_hal.h"
#endif
#if CONFIG_PSTAR_KCONFIG_JTAG_ENABLED
#include "pstar_jtag.h"
#endif
/* #include "pstar_error_handler.h" */

static const char* TAG = "Project Star";

/* --- ADC Configuration --- */
#define POT_ADC_GPIO GPIO_NUM_32      // GPIO pin for the potentiometer (UPDATED)
#define POT_ADC_UNIT ADC_UNIT_1       // ADC unit for GPIO 32
#define POT_ADC_CHANNEL ADC_CHANNEL_4 // GPIO 32 is ADC1 Channel 4 on ESP32 (UPDATED)
#define POT_ADC_ATTEN ADC_ATTEN_DB_12 // Use non-deprecated 12dB attenuation for ~0-3.3V range
#define POT_ADC_WIDTH ADC_BITWIDTH_12 // 12-bit resolution (0-4095)

/* --- Global Bus Manager Instance --- */
static pstar_bus_manager_t g_bus_manager;

/* --- Global Handles for Default PCA9685 Boards --- */
#if CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED && (CONFIG_PSTAR_KCONFIG_PCA9685_DEFAULT_INIT_COUNT > 0)
#define NUM_DEFAULT_PCA9685 CONFIG_PSTAR_KCONFIG_PCA9685_DEFAULT_INIT_COUNT
static pstar_pca9685_hal_handle_t g_pca9685_handles[NUM_DEFAULT_PCA9685] = {NULL};
#else
#define NUM_DEFAULT_PCA9685 0
#endif

/* --- Global ADC Handle --- */
static adc_oneshot_unit_handle_t g_adc1_handle = NULL; // Handle for the ADC1 unit

/**
 * @brief Calculate the PCA9685 OFF value for a given servo angle.
 */
#if CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED
static uint16_t calculate_servo_pwm_off_value(float angle_degrees, float pwm_freq_hz)
{
  const uint32_t servo_min_pulse_us = CONFIG_PSTAR_KCONFIG_PCA9685_SERVO_MIN_PULSE_US;
  const uint32_t servo_max_pulse_us = CONFIG_PSTAR_KCONFIG_PCA9685_SERVO_MAX_PULSE_US;
  const float    servo_angle_range  = (float)CONFIG_PSTAR_KCONFIG_PCA9685_SERVO_ANGLE_RANGE;

  if (angle_degrees < 0.0f) {
    angle_degrees = 0.0f;
  } else if (angle_degrees > servo_angle_range) {
    angle_degrees = servo_angle_range;
  }

  if (pwm_freq_hz <= 0) {
    ESP_LOGE(TAG, "Invalid PWM frequency for servo calculation: %.1f Hz", pwm_freq_hz);
    return 0xFFFF;
  }

  float pulse_us = servo_min_pulse_us +
                   (angle_degrees / servo_angle_range) * (servo_max_pulse_us - servo_min_pulse_us);
  float    period_us        = 1000000.0f / pwm_freq_hz;
  float    time_per_tick_us = period_us / 4096.0f;
  uint16_t off_value        = 0;

  if (time_per_tick_us > 0) {
    off_value = (uint16_t)roundf(pulse_us / time_per_tick_us);
  } else {
    ESP_LOGE(TAG, "Division by zero prevented: time_per_tick_us is zero (check frequency).");
    return 0xFFFF;
  }

  if (off_value > PCA9685_MAX_PWM_VALUE) {
    off_value = PCA9685_MAX_PWM_VALUE;
  }

  ESP_LOGD(TAG,
           "Angle: %.1f -> Pulse: %.1f us -> OFF Value: %u (Freq: %.1f Hz)",
           angle_degrees,
           pulse_us,
           off_value,
           pwm_freq_hz);
  return off_value;
}
#endif // CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED

void app_main(void)
{
  ESP_LOGI(TAG, "Application Start: Project Star");

  esp_err_t ret                 = ESP_OK;
  bool      manager_initialized = false;
  bool      adc_initialized     = false; // Flag for ADC unit initialization
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
  ret = pstar_register_pin(POT_ADC_GPIO, "Potentiometer ADC", false); // Uses updated POT_ADC_GPIO
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to register Potentiometer ADC pin (%d): %s",
                    POT_ADC_GPIO,
                    esp_err_to_name(ret));
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

  /* 3. Configure ADC GPIO and Oneshot Driver */
  ESP_LOGI(TAG, "Configuring GPIO %d for ADC input...", POT_ADC_GPIO); // Uses updated POT_ADC_GPIO
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << POT_ADC_GPIO), // Uses updated POT_ADC_GPIO
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE,
  };
  ret = gpio_config(&io_conf);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to configure GPIO %d: %s",
                    POT_ADC_GPIO,
                    esp_err_to_name(ret)); // Uses updated POT_ADC_GPIO

  ESP_LOGI(TAG,
           "Configuring ADC1 Unit and Channel %d (GPIO %d) for potentiometer...",
           POT_ADC_CHANNEL,
           POT_ADC_GPIO); // Uses updated defines
  adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id  = POT_ADC_UNIT, // Uses updated POT_ADC_UNIT
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ret = adc_oneshot_new_unit(&init_config1, &g_adc1_handle);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to initialize ADC unit %d: %s",
                    POT_ADC_UNIT,
                    esp_err_to_name(ret));
  adc_initialized = true;

  adc_oneshot_chan_cfg_t config = {
    .bitwidth = POT_ADC_WIDTH,
    .atten    = POT_ADC_ATTEN,
  };
  ret = adc_oneshot_config_channel(g_adc1_handle,
                                   POT_ADC_CHANNEL,
                                   &config); // Uses updated POT_ADC_CHANNEL
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to configure ADC channel %d: %s",
                    POT_ADC_CHANNEL,
                    esp_err_to_name(ret)); // Uses updated POT_ADC_CHANNEL

  vTaskDelay(pdMS_TO_TICKS(10));

  /* 4. Create and Initialize Devices */
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
  } else {
    ESP_LOGI(TAG, "Successfully initialized %d default PCA9685 board(s).", NUM_DEFAULT_PCA9685);
  }
  ESP_LOGI(TAG, "Enabling output for initialized PCA9685 boards...");
  for (int board_idx = 0; board_idx < NUM_DEFAULT_PCA9685; ++board_idx) {
    if (g_pca9685_handles[board_idx]) {
      ret = pstar_pca9685_hal_output_enable(g_pca9685_handles[board_idx]);
      if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG,
                 "Failed to enable output for PCA9685 board %d: %s",
                 board_idx,
                 esp_err_to_name(ret));
      } else if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Output enabled for PCA9685 board %d", board_idx);
      }
    }
  }
#elif CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED
  ESP_LOGI(TAG, "Default PCA9685 initialization count is 0. Skipping.");
#endif

  /* 5. Validate All Registered Pins (if validator enabled) */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  ESP_LOGI(TAG, "Validating all registered pins...");
  ret = pstar_validate_pins();
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Pin validation failed: %s", esp_err_to_name(ret));
  ESP_LOGI(TAG, "Pin validation successful.");
#endif

  /* 6. Main Application Loop */
  ESP_LOGI(TAG, "Setup complete. Entering main loop (Potentiometer -> Servos).");

#if CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED
  float default_pwm_freq  = (float)CONFIG_PSTAR_KCONFIG_PCA9685_DEFAULT_PWM_FREQ_HZ;
  float servo_angle_range = (float)CONFIG_PSTAR_KCONFIG_PCA9685_SERVO_ANGLE_RANGE;
#endif
  int adc_raw         = 0;
  int adc_read_result = 0;

  while (1) {
    ESP_LOGD(TAG, "Main loop iteration start.");

    ret = adc_oneshot_read(g_adc1_handle,
                           POT_ADC_CHANNEL,
                           &adc_read_result); // Uses updated POT_ADC_CHANNEL
    ESP_LOGI(TAG,
             "adc_oneshot_read: ret=%d (%s), adc_read_result=%d",
             ret,
             esp_err_to_name(ret),
             adc_read_result);

    if (ret != ESP_OK) {
      ESP_LOGE(TAG,
               "ADC oneshot read failed on Channel %d: %s",
               POT_ADC_CHANNEL,
               esp_err_to_name(ret)); // Uses updated POT_ADC_CHANNEL
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    } else {
      adc_raw = adc_read_result;

#if NUM_DEFAULT_PCA9685 > 0
      float angle = ((float)adc_raw / 4095.0f) * servo_angle_range;
      if (angle < 0.0f)
        angle = 0.0f;
      if (angle > servo_angle_range)
        angle = servo_angle_range;
      ESP_LOGI(TAG, "Mapped Angle: %6.1f deg", angle);

      uint16_t pwm_off_value = calculate_servo_pwm_off_value(angle, default_pwm_freq);

      if (pwm_off_value != 0xFFFF) {
        for (int board_idx = 0; board_idx < NUM_DEFAULT_PCA9685; ++board_idx) {
          if (g_pca9685_handles[board_idx]) {
            esp_err_t set_all_ret =
              pstar_pca9685_hal_set_all_pwm_values(g_pca9685_handles[board_idx], 0, pwm_off_value);
            if (set_all_ret != ESP_OK) {
              ESP_LOGE(TAG,
                       "Failed to set all channels on board %d: %s",
                       board_idx,
                       esp_err_to_name(set_all_ret));
            }
          }
        }
      } else {
        ESP_LOGE(TAG, "Failed to calculate PWM value for angle %.1f. Servos not updated.", angle);
      }
#endif // NUM_DEFAULT_PCA9685 > 0
    } // End ADC read success block

#if CONFIG_PSTAR_KCONFIG_BH1750_ENABLED
    if (bh1750_handle) {
      float current_lux = 0.0f;
      ret               = pstar_bh1750_hal_read_lux(bh1750_handle, &current_lux);
      if (ret == ESP_OK) {
        ESP_LOGD(TAG, "BH1750 Light Level: %.2f Lux", current_lux);
      } else {
        ESP_LOGE(TAG, "Failed to read BH1750 data: %s (%d)", esp_err_to_name(ret), ret);
      }
    }
#endif

    ESP_LOGD(TAG, "Main loop iteration end.");
    vTaskDelay(pdMS_TO_TICKS(20));
  } // End while(1)

cleanup:
  ESP_LOGW(TAG, "Starting cleanup sequence due to error or exit...");

  /* Deinitialize devices */
#if NUM_DEFAULT_PCA9685 > 0
  ESP_LOGI(TAG, "Deinitializing default PCA9685 board(s)...");
  for (int i = 0; i < NUM_DEFAULT_PCA9685; ++i) {
    if (g_pca9685_handles[i]) {
      esp_err_t deinit_ret = pstar_pca9685_hal_deinit(g_pca9685_handles[i], true, true);
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

  /* Deinitialize ADC Unit */
  if (adc_initialized) {
    ESP_LOGI(TAG, "Deinitializing ADC Unit %d...", POT_ADC_UNIT);
    ret = adc_oneshot_del_unit(g_adc1_handle);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to deinitialize ADC unit %d: %s", POT_ADC_UNIT, esp_err_to_name(ret));
    }
    g_adc1_handle   = NULL;
    adc_initialized = false;
  }

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

fatal_error:
  ESP_LOGE(TAG, "Application ending due to fatal error or normal exit from cleanup.");
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}
