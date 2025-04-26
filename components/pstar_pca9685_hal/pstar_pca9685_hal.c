/* components/pstar_pca9685_hal/pstar_pca9685_hal.c */

#include "pstar_pca9685_hal.h"

#include "pstar_bus_config.h"
#include "pstar_bus_i2c.h"
#include "pstar_bus_manager.h"
#include "pstar_pin_validator.h"

#include "driver/gpio.h" /* Include GPIO driver for OE pin */
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" /* For mutex */
#include "freertos/task.h"   /* For vTaskDelay */

#include <math.h>   /* For floor, roundf */
#include <stdio.h>  /* For snprintf */
#include <stdlib.h> /* For malloc, free */
#include <string.h> /* For strdup, memset, snprintf */

#include "esp_check.h" /* For ESP_RETURN_ON_FALSE, ESP_GOTO_ON_ERROR */
#include "esp_log.h"
#include "sdkconfig.h" /* Include sdkconfig for Kconfig defines */

static const char* TAG = "PCA9685 HAL";

/* --- PCA9685 Register Definitions --- */
#define PCA9685_MODE1 0x00      /**< Mode register 1 */
#define PCA9685_MODE2 0x01      /**< Mode register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */

/* --- LED0 Registers --- */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 output and brightness control byte 0 */
#define PCA9685_LED0_ON_H 0x07  /**< LED0 output and brightness control byte 1 */
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 output and brightness control byte 2 */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 output and brightness control byte 3 */

/* --- LED1 Registers --- */
#define PCA9685_LED1_ON_L 0x0A
#define PCA9685_LED1_ON_H 0x0B
#define PCA9685_LED1_OFF_L 0x0C
#define PCA9685_LED1_OFF_H 0x0D

/* --- LED2 Registers --- */
#define PCA9685_LED2_ON_L 0x0E
#define PCA9685_LED2_ON_H 0x0F
#define PCA9685_LED2_OFF_L 0x10
#define PCA9685_LED2_OFF_H 0x11

/* --- LED3 Registers --- */
#define PCA9685_LED3_ON_L 0x12
#define PCA9685_LED3_ON_H 0x13
#define PCA9685_LED3_OFF_L 0x14
#define PCA9685_LED3_OFF_H 0x15

/* --- LED4 Registers --- */
#define PCA9685_LED4_ON_L 0x16
#define PCA9685_LED4_ON_H 0x17
#define PCA9685_LED4_OFF_L 0x18
#define PCA9685_LED4_OFF_H 0x19

/* --- LED5 Registers --- */
#define PCA9685_LED5_ON_L 0x1A
#define PCA9685_LED5_ON_H 0x1B
#define PCA9685_LED5_OFF_L 0x1C
#define PCA9685_LED5_OFF_H 0x1D

/* --- LED6 Registers --- */
#define PCA9685_LED6_ON_L 0x1E
#define PCA9685_LED6_ON_H 0x1F
#define PCA9685_LED6_OFF_L 0x20
#define PCA9685_LED6_OFF_H 0x21

/* --- LED7 Registers --- */
#define PCA9685_LED7_ON_L 0x22
#define PCA9685_LED7_ON_H 0x23
#define PCA9685_LED7_OFF_L 0x24
#define PCA9685_LED7_OFF_H 0x25

/* --- LED8 Registers --- */
#define PCA9685_LED8_ON_L 0x26
#define PCA9685_LED8_ON_H 0x27
#define PCA9685_LED8_OFF_L 0x28
#define PCA9685_LED8_OFF_H 0x29

/* --- LED9 Registers --- */
#define PCA9685_LED9_ON_L 0x2A
#define PCA9685_LED9_ON_H 0x2B
#define PCA9685_LED9_OFF_L 0x2C
#define PCA9685_LED9_OFF_H 0x2D

/* --- LED10 Registers --- */
#define PCA9685_LED10_ON_L 0x2E
#define PCA9685_LED10_ON_H 0x2F
#define PCA9685_LED10_OFF_L 0x30
#define PCA9685_LED10_OFF_H 0x31

/* --- LED11 Registers --- */
#define PCA9685_LED11_ON_L 0x32
#define PCA9685_LED11_ON_H 0x33
#define PCA9685_LED11_OFF_L 0x34
#define PCA9685_LED11_OFF_H 0x35

/* --- LED12 Registers --- */
#define PCA9685_LED12_ON_L 0x36
#define PCA9685_LED12_ON_H 0x37
#define PCA9685_LED12_OFF_L 0x38
#define PCA9685_LED12_OFF_H 0x39

/* --- LED13 Registers --- */
#define PCA9685_LED13_ON_L 0x3A
#define PCA9685_LED13_ON_H 0x3B
#define PCA9685_LED13_OFF_L 0x3C
#define PCA9685_LED13_OFF_H 0x3D

/* --- LED14 Registers --- */
#define PCA9685_LED14_ON_L 0x3E
#define PCA9685_LED14_ON_H 0x3F
#define PCA9685_LED14_OFF_L 0x40
#define PCA9685_LED14_OFF_H 0x41

/* --- LED15 Registers --- */
#define PCA9685_LED15_ON_L 0x42
#define PCA9685_LED15_ON_H 0x43
#define PCA9685_LED15_OFF_L 0x44
#define PCA9685_LED15_OFF_H 0x45

/* --- ALL LED Registers --- */
#define PCA9685_ALL_LED_ON_L 0xFA  /**< load all the LEDn_ON registers, byte 0 */
#define PCA9685_ALL_LED_ON_H 0xFB  /**< load all the LEDn_ON registers, byte 1 */
#define PCA9685_ALL_LED_OFF_L 0xFC /**< load all the LEDn_OFF registers, byte 0 */
#define PCA9685_ALL_LED_OFF_H 0xFD /**< load all the LEDn_OFF registers, byte 1 */

/* --- Other Registers --- */
#define PCA9685_PRE_SCALE 0xFE /**< prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF  /**< defines the test mode to be entered */

/* --- PCA9685 MODE1 Register Bits --- */
#define PCA9685_MODE1_RESTART (1 << 7) /**< Restart enabled */
#define PCA9685_MODE1_EXTCLK (1 << 6)  /**< Use EXTCLK pin clock */
#define PCA9685_MODE1_AI (1 << 5)      /**< Register Auto-Increment enabled */
#define PCA9685_MODE1_SLEEP (1 << 4)   /**< Low power mode. Oscillator off */
#define PCA9685_MODE1_SUB1 (1 << 3)    /**< PCA9685 responds to I2C subaddress 1 */
#define PCA9685_MODE1_SUB2 (1 << 2)    /**< PCA9685 responds to I2C subaddress 2 */
#define PCA9685_MODE1_SUB3 (1 << 1)    /**< PCA9685 responds to I2C subaddress 3 */
#define PCA9685_MODE1_ALLCALL (1 << 0) /**< PCA9685 responds to LED All Call I2C address */

/* --- PCA9685 MODE2 Register Bits --- */
#define PCA9685_MODE2_INVRT (1 << 4)  /**< Output logic state inverted */
#define PCA9685_MODE2_OCH (1 << 3)    /**< Outputs change on ACK */
#define PCA9685_MODE2_OUTDRV (1 << 2) /**< Output drive type: totem pole (1) or open-drain (0) */
#define PCA9685_MODE2_OUTNE1 (1 << 1) /**< Output mode bit 1 */
#define PCA9685_MODE2_OUTNE0 (1 << 0) /**< Output mode bit 0 */

/* --- Constants --- */
#define PCA9685_OSC_CLOCK_MHZ 25.0f   /**< Internal oscillator frequency in MHz */
#define PCA9685_WAKEUP_DELAY_US 500   /**< Delay needed after waking from sleep (datasheet) */
#define MAX_DEFAULT_BOARDS 16         /**< Sanity limit for default init count */
#define BUS_NAME_MAX_LEN 32           /**< Max length for generated bus names */
#define PCA9685_MUTEX_TIMEOUT_MS 1000 /* Timeout for acquiring HAL mutex */

/* --- KConfig Defaults --- */
#ifdef CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED /* Only define if component enabled */
#ifdef CONFIG_PSTAR_KCONFIG_PCA9685_I2C_PORT_0
#define PCA9685_DEFAULT_I2C_PORT (I2C_NUM_0)
#elif CONFIG_PSTAR_KCONFIG_PCA9685_I2C_PORT_1
#define PCA9685_DEFAULT_I2C_PORT (I2C_NUM_1)
#else
#error "No default I2C port selected for PCA9685 in Kconfig"
#define PCA9685_DEFAULT_I2C_PORT (I2C_NUM_0) /* Default fallback */
#endif
#define PCA9685_DEFAULT_BUS_NAME_PREFIX (CONFIG_PSTAR_KCONFIG_PCA9685_BUS_NAME_PREFIX)
#define PCA9685_DEFAULT_I2C_ADDR (CONFIG_PSTAR_KCONFIG_PCA9685_I2C_ADDR)
#define PCA9685_DEFAULT_SDA_PIN (CONFIG_PSTAR_KCONFIG_PCA9685_SDA_PIN)
#define PCA9685_DEFAULT_SCL_PIN (CONFIG_PSTAR_KCONFIG_PCA9685_SCL_PIN)
#define PCA9685_DEFAULT_I2C_FREQ_HZ (CONFIG_PSTAR_KCONFIG_PCA9685_I2C_FREQ_HZ)
#define PCA9685_DEFAULT_PWM_FREQ_HZ ((float)CONFIG_PSTAR_KCONFIG_PCA9685_DEFAULT_PWM_FREQ_HZ)
#define PCA9685_DEFAULT_INIT_COUNT (CONFIG_PSTAR_KCONFIG_PCA9685_DEFAULT_INIT_COUNT)
#ifdef CONFIG_PSTAR_KCONFIG_PCA9685_DEFAULT_ADDR_INCREMENT
#define PCA9685_DEFAULT_ADDR_INCREMENT (1)
#else
#define PCA9685_DEFAULT_ADDR_INCREMENT (0)
#endif
/* OE Pin Kconfig Defaults */
#ifdef CONFIG_PSTAR_KCONFIG_PCA9685_USE_OE_PIN
#define PCA9685_DEFAULT_USE_OE_PIN (1)
#define PCA9685_DEFAULT_OE_PIN (CONFIG_PSTAR_KCONFIG_PCA9685_OE_PIN)
#ifdef CONFIG_PSTAR_KCONFIG_PCA9685_OE_ACTIVE_LOW
#define PCA9685_DEFAULT_OE_ACTIVE_LOW (1)
#else
#define PCA9685_DEFAULT_OE_ACTIVE_LOW (0)
#endif
#else
#define PCA9685_DEFAULT_USE_OE_PIN (0)
#define PCA9685_DEFAULT_OE_PIN (-1)       /* Invalid pin if OE not used */
#define PCA9685_DEFAULT_OE_ACTIVE_LOW (1) /* Default to active low if somehow used without enable */
#endif
/* Servo Kconfig Defaults */
#define PCA9685_SERVO_MIN_PULSE_US CONFIG_PSTAR_KCONFIG_PCA9685_SERVO_MIN_PULSE_US
#define PCA9685_SERVO_MAX_PULSE_US CONFIG_PSTAR_KCONFIG_PCA9685_SERVO_MAX_PULSE_US
#define PCA9685_SERVO_ANGLE_RANGE ((float)CONFIG_PSTAR_KCONFIG_PCA9685_SERVO_ANGLE_RANGE)

#else /* Define fallbacks if component disabled to avoid compile errors elsewhere */
#define PCA9685_DEFAULT_INIT_COUNT (0)
#define PCA9685_DEFAULT_ADDR_INCREMENT (0)
#define PCA9685_DEFAULT_PWM_FREQ_HZ (50.0f)
#define PCA9685_DEFAULT_USE_OE_PIN (0)
#define PCA9685_DEFAULT_OE_PIN (-1)
#define PCA9685_DEFAULT_OE_ACTIVE_LOW (1)
/* Other defaults don't matter much if disabled, but define to avoid errors */
#define PCA9685_DEFAULT_I2C_PORT (I2C_NUM_0)
#define PCA9685_DEFAULT_BUS_NAME_PREFIX "pca9685_disabled"
#define PCA9685_DEFAULT_I2C_ADDR (0x40)
#define PCA9685_DEFAULT_SDA_PIN (21)
#define PCA9685_DEFAULT_SCL_PIN (22)
#define PCA9685_DEFAULT_I2C_FREQ_HZ (400000)
/* Servo defaults */
#define PCA9685_SERVO_MIN_PULSE_US (1000)
#define PCA9685_SERVO_MAX_PULSE_US (2000)
#define PCA9685_SERVO_ANGLE_RANGE (180.0f)
#endif /* CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED */

/* --- Internal Handle Structure --- */
typedef struct pstar_pca9685_hal_dev_t {
  const pstar_bus_manager_t* bus_manager;     /**< Pointer to the bus manager */
  char*                      bus_name;        /**< Name of the bus config (copied) */
  float                      current_freq_hz; /**< Currently configured PWM frequency */
  gpio_num_t                 oe_pin;          /**< GPIO number for OE pin, or < 0 if not used */
  bool                       oe_active_low;   /**< True if OE pin is active low */
  bool                       output_enabled;  /**< Current state of the output enable */
  SemaphoreHandle_t          mutex;           /**< Mutex for thread-safe access to handle state */
} pstar_pca9685_hal_dev_t;

/* --- Helper Functions --- */

/**
 * @brief Write a single byte to a PCA9685 register.
 *        Assumes mutex is already taken if needed.
 *
 * @param handle PCA9685 HAL handle.
 * @param reg_addr Register address to write to.
 * @param value Byte value to write.
 * @return esp_err_t ESP_OK on success, or error from pstar_bus_i2c_write.
 */
static esp_err_t
priv_pca9685_write_byte_nolock(pstar_pca9685_hal_handle_t handle, uint8_t reg_addr, uint8_t value)
{
  /* No NULL check here, assumed valid by caller */
  return pstar_bus_i2c_write(handle->bus_manager, handle->bus_name, &value, 1, reg_addr, NULL);
}

/**
 * @brief Read a single byte from a PCA9685 register.
 *        Assumes mutex is already taken if needed.
 *
 * @param handle PCA9685 HAL handle.
 * @param reg_addr Register address to read from.
 * @param[out] value Pointer to store the read byte value.
 * @return esp_err_t ESP_OK on success, or error from pstar_bus_i2c_read.
 */
static esp_err_t
priv_pca9685_read_byte_nolock(pstar_pca9685_hal_handle_t handle, uint8_t reg_addr, uint8_t* value)
{
  /* No NULL check here, assumed valid by caller */
  return pstar_bus_i2c_read(handle->bus_manager, handle->bus_name, value, 1, reg_addr, NULL);
}

/**
 * @brief Write multiple bytes to PCA9685 starting from a register address.
 *        Uses I2C auto-increment feature. Assumes mutex is already taken.
 *
 * @param handle PCA9685 HAL handle.
 * @param reg_addr Starting register address.
 * @param data Pointer to data buffer.
 * @param len Number of bytes to write.
 * @return esp_err_t ESP_OK on success, or error from pstar_bus_i2c_write.
 */
static esp_err_t priv_pca9685_write_bytes_nolock(pstar_pca9685_hal_handle_t handle,
                                                 uint8_t                    reg_addr,
                                                 const uint8_t*             data,
                                                 size_t                     len)
{
  /* No NULL check here, assumed valid by caller */
  return pstar_bus_i2c_write(handle->bus_manager, handle->bus_name, data, len, reg_addr, NULL);
}

/**
 * @brief Configure the OE GPIO pin.
 *        Assumes mutex is already taken.
 *
 * @param handle PCA9685 HAL handle containing OE pin info.
 * @return esp_err_t ESP_OK on success, or error from GPIO configuration.
 */
static esp_err_t priv_pca9685_configure_oe_pin_nolock(pstar_pca9685_hal_handle_t handle)
{
  /* No NULL check here, assumed valid by caller */

  if (handle->oe_pin < 0 || handle->oe_pin >= GPIO_NUM_MAX) {
    ESP_LOGI(TAG,
             "OE pin not configured or invalid (%d) for '%s'. Skipping GPIO setup.",
             handle->oe_pin,
             handle->bus_name);
    handle->oe_pin = GPIO_NUM_NC; /* Ensure it's marked as not configured */
    return ESP_OK;                /* Not an error if not configured */
  }

  ESP_LOGI(TAG,
           "Configuring OE pin %d for '%s' (Active Low: %s)",
           handle->oe_pin,
           handle->bus_name,
           handle->oe_active_low ? "Yes" : "No");

  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << handle->oe_pin),
    .mode         = GPIO_MODE_OUTPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE, /* Typically OE is driven, not pulled */
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE,
  };

  esp_err_t ret = gpio_config(&io_conf);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to configure OE GPIO pin %d", handle->oe_pin);

  /* Set initial state to disabled */
  uint32_t initial_level = handle->oe_active_low ? 1 : 0; /* Set to inactive level */
  ret                    = gpio_set_level(handle->oe_pin, initial_level);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set initial level for OE pin %d", handle->oe_pin);
  handle->output_enabled = false; /* Start disabled */

  ESP_LOGI(TAG, "OE pin %d configured successfully (initial state: disabled)", handle->oe_pin);
  return ESP_OK;
}

/**
 * @brief Calculate the PCA9685 OFF value for a given servo angle.
 *        Uses Kconfig values for servo parameters.
 *        Assumes mutex is already taken.
 *
 * @param handle PCA9685 HAL handle (used for current frequency).
 * @param angle_degrees Desired angle.
 * @param[out] out_off_value Pointer to store the calculated OFF value.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if frequency is invalid.
 */
static esp_err_t priv_calculate_servo_pwm_off_value_nolock(pstar_pca9685_hal_handle_t handle,
                                                           float                      angle_degrees,
                                                           uint16_t*                  out_off_value)
{
  /* No NULL check here, assumed valid by caller */
  const uint32_t servo_min_pulse_us = PCA9685_SERVO_MIN_PULSE_US;
  const uint32_t servo_max_pulse_us = PCA9685_SERVO_MAX_PULSE_US;
  const float    servo_angle_range  = PCA9685_SERVO_ANGLE_RANGE;
  const float    pwm_freq_hz        = handle->current_freq_hz;

  /* Clamp angle */
  if (angle_degrees < 0.0f) {
    angle_degrees = 0.0f;
  } else if (angle_degrees > servo_angle_range) {
    angle_degrees = servo_angle_range;
  }

  ESP_RETURN_ON_FALSE(pwm_freq_hz > 0,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid PWM frequency for servo calculation: %.1f Hz",
                      pwm_freq_hz);

  float pulse_us = servo_min_pulse_us +
                   (angle_degrees / servo_angle_range) * (servo_max_pulse_us - servo_min_pulse_us);
  float period_us        = 1000000.0f / pwm_freq_hz;
  float time_per_tick_us = period_us / 4096.0f;

  ESP_RETURN_ON_FALSE(time_per_tick_us > 0,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "Division by zero: time_per_tick_us is zero (check frequency).");

  uint16_t off_value = (uint16_t)roundf(pulse_us / time_per_tick_us);

  /* Clamp to max value */
  if (off_value > PCA9685_MAX_PWM_VALUE) {
    off_value = PCA9685_MAX_PWM_VALUE;
  }

  ESP_LOGD(TAG,
           "Angle: %.1f -> Pulse: %.1f us -> OFF Value: %u (Freq: %.1f Hz)",
           angle_degrees,
           pulse_us,
           off_value,
           pwm_freq_hz);

  *out_off_value = off_value;
  return ESP_OK;
}

/* --- Public Function Implementations --- */

/* Internal init function used by create_default and create_custom */
static esp_err_t priv_pca9685_hal_init_internal(const pstar_bus_manager_t*        manager,
                                                const pstar_pca9685_hal_config_t* config,
                                                float                             initial_freq_hz,
                                                gpio_num_t                        oe_pin,
                                                bool                              oe_active_low,
                                                pstar_pca9685_hal_handle_t*       out_handle)
{
  ESP_RETURN_ON_FALSE(manager && config && config->bus_name && out_handle,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Internal Init: Invalid arguments");
  *out_handle   = NULL;
  esp_err_t ret = ESP_OK;

  /* Find the bus configuration provided by the user */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, config->bus_name);
  ESP_RETURN_ON_FALSE(bus_config,
                      ESP_ERR_NOT_FOUND,
                      TAG,
                      "Bus '%s' not found in manager",
                      config->bus_name);
  ESP_RETURN_ON_FALSE(bus_config->initialized,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "Bus '%s' is not initialized",
                      config->bus_name);
  ESP_RETURN_ON_FALSE(bus_config->type == k_pstar_bus_type_i2c,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Bus '%s' is not I2C",
                      config->bus_name);

  /* Allocate memory for the device handle */
  pstar_pca9685_hal_handle_t dev =
    (pstar_pca9685_hal_handle_t)malloc(sizeof(pstar_pca9685_hal_dev_t));
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Failed to allocate device handle");
  memset(dev, 0, sizeof(pstar_pca9685_hal_dev_t));

  dev->bus_manager = manager;
  dev->bus_name    = strdup(config->bus_name); /* Store a copy of the name */
  if (!dev->bus_name) {
    ESP_LOGE(TAG, "Failed to duplicate bus name");
    free(dev);
    return ESP_ERR_NO_MEM;
  }

  /* Create mutex */
  dev->mutex = xSemaphoreCreateMutex();
  if (!dev->mutex) {
    ESP_LOGE(TAG, "Failed to create mutex for handle '%s'", dev->bus_name);
    free(dev->bus_name);
    free(dev);
    return ESP_ERR_NO_MEM;
  }

  /* Take mutex for the rest of initialization */
  if (xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(PCA9685_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex during init for '%s'", dev->bus_name);
    vSemaphoreDelete(dev->mutex);
    free(dev->bus_name);
    free(dev);
    return ESP_ERR_TIMEOUT;
  }

  /* Store OE pin configuration */
  dev->oe_pin         = oe_pin;
  dev->oe_active_low  = oe_active_low;
  dev->output_enabled = false; /* Start disabled */

  /* --- Configure OE Pin (if applicable) --- */
  ret = priv_pca9685_configure_oe_pin_nolock(dev);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure OE pin: %s", esp_err_to_name(ret));
    goto init_fail_mutex;
  }

  /* --- PCA9685 Initialization Sequence --- */
  ESP_LOGI(TAG, "Initializing PCA9685 on bus '%s'...", dev->bus_name);

  /* 1. Reset MODE1 register to default value (clears SLEEP bit, enables auto-increment) */
  ret = priv_pca9685_write_byte_nolock(dev, PCA9685_MODE1, PCA9685_MODE1_AI);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to reset MODE1: %s", esp_err_to_name(ret));
    goto init_fail_mutex;
  }
  vTaskDelay(pdMS_TO_TICKS(5)); /* Allow oscillator to stabilize after potential reset/power-on */

  /* 2. Set MODE2 register to default (totem pole output) */
  ret = priv_pca9685_write_byte_nolock(dev, PCA9685_MODE2, PCA9685_MODE2_OUTDRV);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set MODE2: %s", esp_err_to_name(ret));
    goto init_fail_mutex;
  }

  /* 3. Set initial PWM frequency (internal function takes care of sleep/restart) */
  /* Note: set_pwm_freq acquires the mutex itself, so release before calling */
  xSemaphoreGive(dev->mutex);
  ret = pstar_pca9685_hal_set_pwm_freq(dev, initial_freq_hz);
  /* Re-acquire mutex after call */
  if (xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(PCA9685_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout re-acquiring mutex after set_pwm_freq for '%s'", dev->bus_name);
    /* Cannot reliably continue, cleanup needed */
    vSemaphoreDelete(dev->mutex);
    free(dev->bus_name);
    free(dev);
    return ESP_ERR_TIMEOUT;
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set initial PWM frequency: %s", esp_err_to_name(ret));
    goto init_fail_mutex;
  }

  /* 4. Set all PWM channels OFF initially */
  uint8_t all_off_data[4] = {0x00, 0x00, 0x00, 0x10}; /* ON=0, OFF=4096 (Full OFF bit) */
  ret = priv_pca9685_write_bytes_nolock(dev, PCA9685_ALL_LED_ON_L, all_off_data, 4);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set all channels OFF: %s", esp_err_to_name(ret));
    goto init_fail_mutex;
  }

  /* Initialization successful */
  xSemaphoreGive(dev->mutex); /* Release mutex */

  ESP_LOGI(TAG,
           "PCA9685 initialized successfully on bus '%s' at %.2f Hz (OE Pin: %d)",
           dev->bus_name,
           dev->current_freq_hz, /* Freq updated by set_pwm_freq */
           dev->oe_pin);
  *out_handle = dev;
  return ESP_OK;

init_fail_mutex:
  xSemaphoreGive(dev->mutex); /* Release mutex on failure */
  ESP_LOGE(TAG, "PCA9685 initialization failed on bus '%s'", dev->bus_name);
  if (dev) {
    if (dev->mutex) {
      vSemaphoreDelete(dev->mutex);
    }
    if (dev->bus_name) {
      free(dev->bus_name);
    }
    free(dev);
  }
  *out_handle = NULL;
  return ret;
}

/* Public init function (calls internal init with Kconfig OE settings) */
esp_err_t pstar_pca9685_hal_init(const pstar_bus_manager_t*        manager,
                                 const pstar_pca9685_hal_config_t* config,
                                 float                             initial_freq_hz,
                                 pstar_pca9685_hal_handle_t*       out_handle)
{
#if CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED
  /* Use Kconfig defaults for OE pin for this specific init function */
  gpio_num_t oe_pin        = PCA9685_DEFAULT_USE_OE_PIN ? PCA9685_DEFAULT_OE_PIN : GPIO_NUM_NC;
  bool       oe_active_low = PCA9685_DEFAULT_OE_ACTIVE_LOW;
  return priv_pca9685_hal_init_internal(manager,
                                        config,
                                        initial_freq_hz,
                                        oe_pin,
                                        oe_active_low,
                                        out_handle);
#else
  ESP_LOGE(TAG, "PCA9685 component is disabled in Kconfig.");
  if (out_handle)
    *out_handle = NULL;
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t
pstar_pca9685_hal_deinit(pstar_pca9685_hal_handle_t handle, bool sleep, bool disable_output)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGI(TAG,
           "Deinitializing PCA9685 HAL for bus '%s'",
           handle->bus_name ? handle->bus_name : "UNKNOWN");

  /* Take mutex */
  if (handle->mutex &&
      xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(PCA9685_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex during deinit for '%s'", handle->bus_name);
    /* Continue cleanup cautiously */
  }

  if (disable_output) {
    if (handle->oe_pin >= 0) {
      uint32_t  inactive_level = handle->oe_active_low ? 1 : 0;
      esp_err_t disable_ret    = gpio_set_level(handle->oe_pin, inactive_level);
      if (disable_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable output during deinit: %s", esp_err_to_name(disable_ret));
      } else {
        handle->output_enabled = false;
        ESP_LOGD(TAG, "Output disabled via OE pin.");
      }
    }
  }

  if (sleep) {
    uint8_t   mode1;
    esp_err_t read_ret = priv_pca9685_read_byte_nolock(handle, PCA9685_MODE1, &mode1);
    if (read_ret == ESP_OK) {
      mode1               = (mode1 & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
      esp_err_t sleep_ret = priv_pca9685_write_byte_nolock(handle, PCA9685_MODE1, mode1);
      if (sleep_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sleep command during deinit: %s", esp_err_to_name(sleep_ret));
      } else {
        ESP_LOGD(TAG, "Sent Sleep command.");
      }
    } else {
      ESP_LOGE(TAG,
               "Failed to read MODE1 before sleep during deinit: %s",
               esp_err_to_name(read_ret));
    }
  }

  /* Release and delete mutex */
  if (handle->mutex) {
    xSemaphoreGive(handle->mutex);
    vSemaphoreDelete(handle->mutex);
    handle->mutex = NULL;
  }

  if (handle->bus_name) {
    free(handle->bus_name);
  }
  free(handle);
  return ESP_OK;
}

esp_err_t pstar_pca9685_hal_set_pwm_freq(pstar_pca9685_hal_handle_t handle, float freq_hz)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(PCA9685_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for set_pwm_freq on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGD(TAG, "Setting PWM frequency to %.2f Hz for '%s'", freq_hz, handle->bus_name);

  /* Clamp frequency */
  if (freq_hz < 24.0f)
    freq_hz = 24.0f;
  if (freq_hz > 1526.0f)
    freq_hz = 1526.0f;

  float   prescale_float = (PCA9685_OSC_CLOCK_MHZ * 1000000.0f) / (4096.0f * freq_hz) - 1.0f;
  uint8_t prescale       = (uint8_t)floor(prescale_float + 0.5f);
  ESP_LOGD(TAG, "Calculated prescale value: %d for %.2f Hz", prescale, freq_hz);

  uint8_t old_mode1;
  ret = priv_pca9685_read_byte_nolock(handle, PCA9685_MODE1, &old_mode1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read MODE1 before setting frequency: %s", esp_err_to_name(ret));
    goto freq_fail_mutex;
  }

  uint8_t new_mode1 = (old_mode1 & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
  ret               = priv_pca9685_write_byte_nolock(handle, PCA9685_MODE1, new_mode1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enter sleep mode before setting frequency: %s", esp_err_to_name(ret));
    goto freq_fail_mutex;
  }
  vTaskDelay(pdMS_TO_TICKS(1));

  ret = priv_pca9685_write_byte_nolock(handle, PCA9685_PRE_SCALE, prescale);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write prescale value: %s", esp_err_to_name(ret));
    goto freq_fail_mutex;
  }

  ret = priv_pca9685_write_byte_nolock(handle, PCA9685_MODE1, old_mode1 & ~PCA9685_MODE1_SLEEP);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to restore MODE1 after setting frequency: %s", esp_err_to_name(ret));
    goto freq_fail_mutex;
  }
  vTaskDelay(pdMS_TO_TICKS(1));

  ret = priv_pca9685_write_byte_nolock(handle, PCA9685_MODE1, old_mode1 | PCA9685_MODE1_RESTART);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set RESTART bit after setting frequency: %s", esp_err_to_name(ret));
    goto freq_fail_mutex;
  }

  handle->current_freq_hz = (PCA9685_OSC_CLOCK_MHZ * 1000000.0f) / (4096.0f * (prescale + 1));
  ESP_LOGD(TAG,
           "Frequency set for '%s'. Actual frequency: %.2f Hz",
           handle->bus_name,
           handle->current_freq_hz);

freq_fail_mutex:
  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_pca9685_hal_set_channel_pwm_values(pstar_pca9685_hal_handle_t handle,
                                                   uint8_t                    channel,
                                                   uint16_t                   on_value,
                                                   uint16_t                   off_value)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(channel < PCA9685_NUM_CHANNELS,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid channel number: %d",
                      channel);
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(PCA9685_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for set_channel_pwm on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  on_value &= 0x1FFF;
  off_value &= 0x1FFF;

  ESP_LOGD(TAG,
           "Setting channel %d on '%s' to ON=0x%04X, OFF=0x%04X",
           channel,
           handle->bus_name,
           on_value,
           off_value);

  uint8_t reg_base = PCA9685_LED0_ON_L + (4 * channel);
  uint8_t data[4]  = {(uint8_t)(on_value & 0xFF),
                      (uint8_t)(on_value >> 8),
                      (uint8_t)(off_value & 0xFF),
                      (uint8_t)(off_value >> 8)};

  ret = priv_pca9685_write_bytes_nolock(handle, reg_base, data, 4);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG,
             "Failed to write PWM values for channel %d on '%s': %s",
             channel,
             handle->bus_name,
             esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_pca9685_hal_set_channel_duty_cycle(pstar_pca9685_hal_handle_t handle,
                                                   uint8_t                    channel,
                                                   float                      duty_cycle_percent)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(channel < PCA9685_NUM_CHANNELS,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid channel number: %d",
                      channel);
  ESP_RETURN_ON_FALSE(duty_cycle_percent >= 0.0f && duty_cycle_percent <= 100.0f,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Duty cycle must be between 0.0 and 100.0");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  uint16_t on_value  = 0;
  uint16_t off_value = 0;

  if (duty_cycle_percent >= 100.0f) {
    on_value  = 0x1000;
    off_value = 0;
  } else if (duty_cycle_percent <= 0.0f) {
    on_value  = 0;
    off_value = 0x1000;
  } else {
    off_value = (uint16_t)roundf((duty_cycle_percent / 100.0f) * 4096.0f);
    if (off_value > PCA9685_MAX_PWM_VALUE) {
      off_value = PCA9685_MAX_PWM_VALUE;
    }
    on_value = 0;
  }

  /* Call the raw value setter (which handles mutex) */
  return pstar_pca9685_hal_set_channel_pwm_values(handle, channel, on_value, off_value);
}

esp_err_t pstar_pca9685_hal_set_all_pwm_values(pstar_pca9685_hal_handle_t handle,
                                               uint16_t                   on_value,
                                               uint16_t                   off_value)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(PCA9685_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for set_all_pwm on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  on_value &= 0x1FFF;
  off_value &= 0x1FFF;

  ESP_LOGD(TAG,
           "Setting all channels on '%s' to ON=0x%04X, OFF=0x%04X",
           handle->bus_name,
           on_value,
           off_value);

  uint8_t reg_base = PCA9685_ALL_LED_ON_L;
  uint8_t data[4]  = {(uint8_t)(on_value & 0xFF),
                      (uint8_t)(on_value >> 8),
                      (uint8_t)(off_value & 0xFF),
                      (uint8_t)(off_value >> 8)};

  ret = priv_pca9685_write_bytes_nolock(handle, reg_base, data, 4);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG,
             "Failed to write ALL_LED PWM values for '%s': %s",
             handle->bus_name,
             esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_pca9685_hal_set_servo_angle(pstar_pca9685_hal_handle_t handle,
                                            uint8_t                    channel,
                                            float                      angle_degrees)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(channel < PCA9685_NUM_CHANNELS,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid channel number: %d",
                      channel);
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret           = ESP_OK;
  uint16_t  pwm_off_value = 0;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(PCA9685_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for set_servo_angle on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  /* Calculate the OFF value based on angle and current frequency */
  ret = priv_calculate_servo_pwm_off_value_nolock(handle, angle_degrees, &pwm_off_value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG,
             "Failed to calculate PWM for angle %.1f on channel %d: %s",
             angle_degrees,
             channel,
             esp_err_to_name(ret));
    goto servo_angle_fail_mutex;
  }

  /* Set the calculated PWM value (ON=0, OFF=calculated) */
  uint8_t reg_base = PCA9685_LED0_ON_L + (4 * channel);
  uint8_t data[4]  = {0x00, 0x00, (uint8_t)(pwm_off_value & 0xFF), (uint8_t)(pwm_off_value >> 8)};

  ret = priv_pca9685_write_bytes_nolock(handle, reg_base, data, 4);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG,
             "Failed to set servo angle %.1f (PWM %u) for channel %d on '%s': %s",
             angle_degrees,
             pwm_off_value,
             channel,
             handle->bus_name,
             esp_err_to_name(ret));
  }

servo_angle_fail_mutex:
  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_pca9685_hal_set_all_servos_angle(pstar_pca9685_hal_handle_t handle,
                                                 float                      angle_degrees)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret           = ESP_OK;
  uint16_t  pwm_off_value = 0;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(PCA9685_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for set_all_servos_angle on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  /* Calculate the OFF value based on angle and current frequency */
  ret = priv_calculate_servo_pwm_off_value_nolock(handle, angle_degrees, &pwm_off_value);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG,
             "Failed to calculate PWM for angle %.1f for all channels: %s",
             angle_degrees,
             esp_err_to_name(ret));
    goto all_servo_angle_fail_mutex;
  }

  /* Set the calculated PWM value for all channels (ON=0, OFF=calculated) */
  uint8_t reg_base = PCA9685_ALL_LED_ON_L;
  uint8_t data[4]  = {0x00, 0x00, (uint8_t)(pwm_off_value & 0xFF), (uint8_t)(pwm_off_value >> 8)};

  ret = priv_pca9685_write_bytes_nolock(handle, reg_base, data, 4);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG,
             "Failed to set all servos angle %.1f (PWM %u) on '%s': %s",
             angle_degrees,
             pwm_off_value,
             handle->bus_name,
             esp_err_to_name(ret));
  }

all_servo_angle_fail_mutex:
  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_pca9685_hal_restart(pstar_pca9685_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(PCA9685_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for restart on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGD(TAG, "Restarting PCA9685 on '%s'", handle->bus_name);
  uint8_t mode1;
  ret = priv_pca9685_read_byte_nolock(handle, PCA9685_MODE1, &mode1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read MODE1 before restart: %s", esp_err_to_name(ret));
    goto restart_fail_mutex;
  }

  if (mode1 & PCA9685_MODE1_RESTART) {
    mode1 &= ~PCA9685_MODE1_SLEEP;
    ret = priv_pca9685_write_byte_nolock(handle, PCA9685_MODE1, mode1);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to clear sleep before restart: %s", esp_err_to_name(ret));
      goto restart_fail_mutex;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  uint8_t restart_mode = (mode1 & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART | PCA9685_MODE1_AI;
  ret                  = priv_pca9685_write_byte_nolock(handle, PCA9685_MODE1, restart_mode);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set RESTART bit: %s", esp_err_to_name(ret));
  }

  vTaskDelay(pdMS_TO_TICKS(1));

restart_fail_mutex:
  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_pca9685_hal_sleep(pstar_pca9685_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(PCA9685_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for sleep on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGD(TAG, "Putting PCA9685 on '%s' to sleep", handle->bus_name);
  uint8_t mode1;
  ret = priv_pca9685_read_byte_nolock(handle, PCA9685_MODE1, &mode1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read MODE1 before sleep: %s", esp_err_to_name(ret));
    goto sleep_fail_mutex;
  }

  mode1 = (mode1 & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
  ret   = priv_pca9685_write_byte_nolock(handle, PCA9685_MODE1, mode1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write MODE1 to enter sleep: %s", esp_err_to_name(ret));
  }

  vTaskDelay(pdMS_TO_TICKS(1));

sleep_fail_mutex:
  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_pca9685_hal_wakeup(pstar_pca9685_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(PCA9685_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for wakeup on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGD(TAG, "Waking up PCA9685 on '%s'", handle->bus_name);
  uint8_t mode1;
  ret = priv_pca9685_read_byte_nolock(handle, PCA9685_MODE1, &mode1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read MODE1 before wakeup: %s", esp_err_to_name(ret));
    goto wakeup_fail_mutex;
  }

  uint8_t wakeup_mode = mode1 & ~PCA9685_MODE1_SLEEP;
  ret                 = priv_pca9685_write_byte_nolock(handle, PCA9685_MODE1, wakeup_mode);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write MODE1 to wake up: %s", esp_err_to_name(ret));
    goto wakeup_fail_mutex;
  }

  vTaskDelay(pdMS_TO_TICKS(1));

  /* Release mutex before calling restart (which takes mutex itself) */
  xSemaphoreGive(handle->mutex);
  ret = pstar_pca9685_hal_restart(handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to restart after wakeup: %s", esp_err_to_name(ret));
  }
  return ret; /* Return result of restart */

wakeup_fail_mutex:
  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_pca9685_hal_output_enable(pstar_pca9685_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(PCA9685_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for output_enable on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  if (handle->oe_pin < 0) {
    ESP_LOGD(TAG, "Output enable requested for '%s', but OE pin not configured.", handle->bus_name);
    ret = ESP_ERR_INVALID_STATE;
    goto oe_enable_fail_mutex;
  }

  uint32_t active_level = handle->oe_active_low ? 0 : 1;
  ret                   = gpio_set_level(handle->oe_pin, active_level);
  if (ret == ESP_OK) {
    handle->output_enabled = true;
    ESP_LOGD(TAG, "Output enabled for '%s' via OE pin %d", handle->bus_name, handle->oe_pin);
  } else {
    ESP_LOGE(TAG,
             "Failed to set OE pin %d to active level (%lu) for '%s': %s",
             handle->oe_pin,
             (unsigned long)active_level,
             handle->bus_name,
             esp_err_to_name(ret));
  }

oe_enable_fail_mutex:
  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_pca9685_hal_output_disable(pstar_pca9685_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(PCA9685_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for output_disable on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  if (handle->oe_pin < 0) {
    ESP_LOGD(TAG,
             "Output disable requested for '%s', but OE pin not configured.",
             handle->bus_name);
    ret = ESP_ERR_INVALID_STATE;
    goto oe_disable_fail_mutex;
  }

  uint32_t inactive_level = handle->oe_active_low ? 1 : 0;
  ret                     = gpio_set_level(handle->oe_pin, inactive_level);
  if (ret == ESP_OK) {
    handle->output_enabled = false;
    ESP_LOGD(TAG, "Output disabled for '%s' via OE pin %d", handle->bus_name, handle->oe_pin);
  } else {
    ESP_LOGE(TAG,
             "Failed to set OE pin %d to inactive level (%lu) for '%s': %s",
             handle->oe_pin,
             (unsigned long)inactive_level,
             handle->bus_name,
             esp_err_to_name(ret));
  }

oe_disable_fail_mutex:
  xSemaphoreGive(handle->mutex);
  return ret;
}

/* --- Default/Custom Creation Functions --- */

esp_err_t pstar_pca9685_hal_create_kconfig_default(pstar_bus_manager_t*        manager,
                                                   pstar_pca9685_hal_handle_t* out_handle)
{
#if !CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED
  ESP_LOGW(TAG, "PCA9685 KConfig default creation called but component is disabled.");
  if (out_handle)
    *out_handle = NULL;
  return ESP_ERR_NOT_SUPPORTED;
#else
  ESP_RETURN_ON_FALSE(manager && out_handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

  *out_handle                            = NULL;
  esp_err_t           ret                = ESP_OK;
  bool                config_created     = false;
  bool                config_added       = false;
  pstar_bus_config_t* pca9685_i2c_config = NULL;
  char                bus_name_buffer[BUS_NAME_MAX_LEN];
  snprintf(bus_name_buffer, sizeof(bus_name_buffer), "%s_0", PCA9685_DEFAULT_BUS_NAME_PREFIX);

  ESP_LOGI(TAG, "Creating *first* PCA9685 with KConfig default configuration");

  /* 1. Create Bus Configuration using KConfig values */
  pca9685_i2c_config = pstar_bus_config_create_i2c(bus_name_buffer,
                                                   PCA9685_DEFAULT_I2C_PORT,
                                                   PCA9685_DEFAULT_I2C_ADDR,
                                                   PCA9685_DEFAULT_SDA_PIN,
                                                   PCA9685_DEFAULT_SCL_PIN,
                                                   PCA9685_DEFAULT_I2C_FREQ_HZ);

  ESP_GOTO_ON_FALSE(pca9685_i2c_config != NULL,
                    ESP_ERR_NO_MEM,
                    cleanup,
                    TAG,
                    "Failed to create PCA9685 I2C config");
  config_created = true;

  /* 2. Add Configuration to the Manager */
  ret = pstar_bus_manager_add_bus(manager, pca9685_i2c_config);
  if (ret == ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG,
             "Bus config '%s' already exists in manager, attempting to find it.",
             bus_name_buffer);
    pca9685_i2c_config = pstar_bus_manager_find_bus(manager, bus_name_buffer);
    if (!pca9685_i2c_config) {
      ESP_LOGE(TAG,
               "Bus config '%s' reported as existing but could not be found.",
               bus_name_buffer);
      ret = ESP_FAIL;
      goto cleanup;
    }
    ESP_LOGI(TAG, "Found existing bus config '%s'. Proceeding with HAL init.", bus_name_buffer);
    config_added   = true;
    config_created = false;
    ret            = ESP_OK;
  } else {
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed to add PCA9685 config to manager: %s",
                      esp_err_to_name(ret));
    config_added   = true;
    config_created = false;
  }

  /* 3. Initialize the bus hardware (only if we just added it and it's not initialized) */
  if (!pca9685_i2c_config->initialized) {
    // *** CORRECTED LINE ***
    ret = pstar_bus_config_init(pca9685_i2c_config, manager);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed to initialize bus hardware for '%s': %s",
                      bus_name_buffer,
                      esp_err_to_name(ret));
  }

  /* 4. Initialize the PCA9685 HAL Component using the internal init */
  pstar_pca9685_hal_config_t hal_config = {.bus_name = bus_name_buffer};
  gpio_num_t oe_pin        = PCA9685_DEFAULT_USE_OE_PIN ? PCA9685_DEFAULT_OE_PIN : GPIO_NUM_NC;
  bool       oe_active_low = PCA9685_DEFAULT_OE_ACTIVE_LOW;

  ret = priv_pca9685_hal_init_internal(manager,
                                       &hal_config,
                                       PCA9685_DEFAULT_PWM_FREQ_HZ,
                                       oe_pin,
                                       oe_active_low,
                                       out_handle);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to initialize PCA9685 HAL: %s",
                    esp_err_to_name(ret));

  ESP_LOGI(TAG, "First default PCA9685 created and initialized successfully");
  return ESP_OK;

cleanup:
  if (config_created && !config_added) {
    ESP_LOGW(TAG, "Destroying config that was created but not added to manager");
    pstar_bus_config_destroy(pca9685_i2c_config);
  }
  if (out_handle)
    *out_handle = NULL;
  return ret;
#endif /* CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED */
}

esp_err_t pstar_pca9685_hal_create_multiple_defaults(pstar_bus_manager_t*       manager,
                                                     uint8_t                    num_boards,
                                                     pstar_pca9685_hal_handle_t out_handles[])
{
#if !CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED
  ESP_LOGW(TAG, "PCA9685 multiple default creation called but component is disabled.");
  if (out_handles) {
    for (uint8_t i = 0; i < num_boards; ++i) {
      out_handles[i] = NULL;
    }
  }
  return ESP_ERR_NOT_SUPPORTED;
#else
  ESP_RETURN_ON_FALSE(manager && out_handles,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Manager or out_handles array is NULL");
  ESP_RETURN_ON_FALSE(num_boards <= MAX_DEFAULT_BOARDS,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "num_boards (%d) exceeds limit (%d)",
                      num_boards,
                      MAX_DEFAULT_BOARDS);

  if (num_boards == 0) {
    ESP_LOGI(TAG, "Requested 0 default boards to initialize.");
    return ESP_OK;
  }

  if (num_boards != PCA9685_DEFAULT_INIT_COUNT) {
    ESP_LOGW(TAG,
             "Requested num_boards (%d) differs from Kconfig default count (%d)",
             num_boards,
             PCA9685_DEFAULT_INIT_COUNT);
  }

  esp_err_t first_error           = ESP_OK;
  bool      port_driver_installed = false; /* Track if driver for the DEFAULT port is installed */

  ESP_LOGI(TAG, "Creating %d PCA9685 instances with KConfig defaults...", num_boards);

  /* Get common OE pin settings from Kconfig */
  gpio_num_t common_oe_pin = PCA9685_DEFAULT_USE_OE_PIN ? PCA9685_DEFAULT_OE_PIN : GPIO_NUM_NC;
  bool       common_oe_active_low = PCA9685_DEFAULT_OE_ACTIVE_LOW;
  if (PCA9685_DEFAULT_USE_OE_PIN) {
    ESP_LOGI(TAG,
             "Using common OE Pin %d (Active Low: %s) for all default boards",
             common_oe_pin,
             common_oe_active_low ? "Yes" : "No");
  }

  for (uint8_t i = 0; i < num_boards; ++i) {
    out_handles[i]                          = NULL;
    esp_err_t           loop_ret            = ESP_OK;
    bool                loop_config_created = false;
    bool                loop_config_added   = false;
    pstar_bus_config_t* loop_i2c_config     = NULL;
    char                loop_bus_name[BUS_NAME_MAX_LEN];
    uint8_t             current_addr = PCA9685_DEFAULT_I2C_ADDR;

    snprintf(loop_bus_name, sizeof(loop_bus_name), "%s_%d", PCA9685_DEFAULT_BUS_NAME_PREFIX, i);

    /* Calculate address */
    if (i > 0 && PCA9685_DEFAULT_ADDR_INCREMENT) {
      current_addr += i;
      if (current_addr > 0x7F) {
        ESP_LOGE(TAG,
                 "Calculated I2C address 0x%02X for board %d exceeds valid range.",
                 current_addr,
                 i);
        loop_ret = ESP_ERR_INVALID_ARG;
        goto loop_cleanup;
      }
    } else if (i > 0 && !PCA9685_DEFAULT_ADDR_INCREMENT) {
      /* Check if address is still the default, indicating a likely conflict */
      if (current_addr == PCA9685_DEFAULT_I2C_ADDR) {
        ESP_LOGE(
          TAG,
          "Address increment disabled, but multiple boards requested with the same default address (0x%02X).",
          current_addr);
        loop_ret = ESP_ERR_INVALID_ARG;
        goto loop_cleanup;
      }
    }

    ESP_LOGD(TAG, "Board %d: Bus Name='%s', Address=0x%02X", i, loop_bus_name, current_addr);

    /* Check if bus config already exists */
    loop_i2c_config = pstar_bus_manager_find_bus(manager, loop_bus_name);
    if (loop_i2c_config) {
      ESP_LOGW(TAG,
               "Bus config '%s' already exists for board %d. Using existing.",
               loop_bus_name,
               i);
      /* --- CORRECTED ACCESS --- */
      if (loop_i2c_config->type != k_pstar_bus_type_i2c ||
          loop_i2c_config->proto.i2c.port != PCA9685_DEFAULT_I2C_PORT) {
        ESP_LOGE(TAG,
                 "Existing bus config '%s' has wrong type/port for board %d.",
                 loop_bus_name,
                 i);
        loop_ret = ESP_ERR_INVALID_STATE;
        goto loop_cleanup;
      }
      loop_config_added   = true;
      loop_config_created = false;
      /* If the config exists AND is initialized, assume the port driver is installed */
      if (loop_i2c_config->initialized) {
        port_driver_installed = true;
      }
    } else {
      /* 1. Create Bus Configuration */
      loop_i2c_config = pstar_bus_config_create_i2c(loop_bus_name,
                                                    PCA9685_DEFAULT_I2C_PORT,
                                                    current_addr,
                                                    PCA9685_DEFAULT_SDA_PIN,
                                                    PCA9685_DEFAULT_SCL_PIN,
                                                    PCA9685_DEFAULT_I2C_FREQ_HZ);
      if (!loop_i2c_config) {
        ESP_LOGE(TAG, "Failed to create I2C config for board %d", i);
        loop_ret = ESP_ERR_NO_MEM;
        goto loop_cleanup;
      }
      loop_config_created = true;

      /* 2. Add Configuration to Manager */
      loop_ret = pstar_bus_manager_add_bus(manager, loop_i2c_config);
      if (loop_ret != ESP_OK) {
        ESP_LOGE(TAG,
                 "Failed to add config to manager for board %d: %s",
                 i,
                 esp_err_to_name(loop_ret));
        goto loop_cleanup;
      }
      loop_config_added   = true;
      loop_config_created = false;
    }

    /* 3. Initialize Bus Hardware (Driver Install) - ONLY IF NOT ALREADY DONE FOR THIS PORT */
    if (!loop_i2c_config->initialized) {
      if (!port_driver_installed) {
        ESP_LOGI(TAG,
                 "Initializing I2C driver for Port %d (Board %d).",
                 PCA9685_DEFAULT_I2C_PORT,
                 i);
        // *** CORRECTED LINE ***
        loop_ret = pstar_bus_config_init(loop_i2c_config, manager);
        if (loop_ret != ESP_OK) {
          ESP_LOGE(TAG,
                   "Failed to init bus hardware for board %d ('%s'): %s",
                   i,
                   loop_bus_name,
                   esp_err_to_name(loop_ret));
          goto loop_cleanup;
        }
        port_driver_installed = true;
      } else {
        ESP_LOGI(
          TAG,
          "I2C driver for Port %d already installed. Marking config '%s' as initialized without calling i2c_driver_install again.",
          PCA9685_DEFAULT_I2C_PORT,
          loop_bus_name);
        loop_i2c_config->initialized = true;
        loop_ret                     = ESP_OK;
      }
    } else {
      ESP_LOGI(TAG, "Bus config '%s' already marked as initialized.", loop_bus_name);
      port_driver_installed = true;
      loop_ret              = ESP_OK;
    }

    /* 4. Initialize HAL (using the specific bus config) */
    pstar_pca9685_hal_config_t hal_config = {.bus_name = loop_bus_name};
    loop_ret                              = priv_pca9685_hal_init_internal(manager,
                                              &hal_config,
                                              PCA9685_DEFAULT_PWM_FREQ_HZ,
                                              common_oe_pin,
                                              common_oe_active_low,
                                              &out_handles[i]); /* Store handle */
    if (loop_ret != ESP_OK) {
      ESP_LOGE(TAG,
               "Failed to init PCA9685 HAL for board %d ('%s'): %s",
               i,
               loop_bus_name,
               esp_err_to_name(loop_ret));
      goto loop_cleanup;
    }

    ESP_LOGI(TAG, "Successfully initialized default PCA9685 board %d ('%s')", i, loop_bus_name);

  loop_cleanup:
    if (loop_ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to initialize default PCA9685 board %d.", i);
      if (first_error == ESP_OK) {
        first_error = loop_ret;
      }
      if (loop_config_created && !loop_config_added) {
        ESP_LOGW(TAG,
                 "Destroying config '%s' due to initialization failure for board %d",
                 loop_bus_name,
                 i);
        pstar_bus_config_destroy(loop_i2c_config);
      }
      out_handles[i] = NULL;
    }
  } /* End of loop */

  return first_error;
#endif /* CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED */
}

esp_err_t pstar_pca9685_hal_create_custom(pstar_bus_manager_t*        manager,
                                          const char*                 bus_name,
                                          i2c_port_t                  port,
                                          uint8_t                     addr,
                                          int                         sda_pin,
                                          int                         scl_pin,
                                          uint32_t                    i2c_freq_hz,
                                          float                       initial_pwm_freq_hz,
                                          gpio_num_t                  oe_pin,
                                          bool                        oe_active_low,
                                          pstar_pca9685_hal_handle_t* out_handle)
{
  ESP_RETURN_ON_FALSE(manager && bus_name && out_handle,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid arguments");
  ESP_RETURN_ON_FALSE(strlen(bus_name) > 0, ESP_ERR_INVALID_ARG, TAG, "Bus name cannot be empty");

  *out_handle                            = NULL;
  esp_err_t           ret                = ESP_OK;
  bool                config_created     = false;
  bool                config_added       = false;
  pstar_bus_config_t* pca9685_i2c_config = NULL;

  ESP_LOGI(TAG, "Creating PCA9685 with custom configuration for bus '%s'", bus_name);

  /* 1. Create Bus Configuration using provided values */
  pca9685_i2c_config =
    pstar_bus_config_create_i2c(bus_name, port, addr, sda_pin, scl_pin, i2c_freq_hz);
  ESP_GOTO_ON_FALSE(pca9685_i2c_config != NULL,
                    ESP_ERR_NO_MEM,
                    cleanup,
                    TAG,
                    "Failed to create custom PCA9685 I2C config");
  config_created = true;

  /* 2. Add Configuration to the Manager */
  ret = pstar_bus_manager_add_bus(manager, pca9685_i2c_config);
  if (ret == ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG,
             "Custom bus config '%s' already exists in manager, attempting to find it.",
             bus_name);
    pca9685_i2c_config = pstar_bus_manager_find_bus(manager, bus_name);
    if (!pca9685_i2c_config) {
      ESP_LOGE(TAG, "Custom bus config '%s' reported existing but not found.", bus_name);
      ret = ESP_FAIL;
      goto cleanup;
    }
    ESP_LOGI(TAG, "Found existing custom bus config '%s'.", bus_name);
    config_added   = true;
    config_created = false;
    ret            = ESP_OK;
  } else {
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed to add custom PCA9685 config to manager: %s",
                      esp_err_to_name(ret));
    config_added   = true;
    config_created = false;
  }

  /* 3. Initialize the bus hardware (if not already initialized) */
  if (!pca9685_i2c_config->initialized) {
    // *** CORRECTED LINE ***
    ret = pstar_bus_config_init(pca9685_i2c_config, manager);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed to initialize custom bus hardware for '%s': %s",
                      bus_name,
                      esp_err_to_name(ret));
  }

  /* 4. Initialize the PCA9685 HAL Component using internal init */
  pstar_pca9685_hal_config_t hal_config = {.bus_name = bus_name};
  ret                                   = priv_pca9685_hal_init_internal(manager,
                                       &hal_config,
                                       initial_pwm_freq_hz,
                                       oe_pin,
                                       oe_active_low,
                                       out_handle);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to initialize custom PCA9685 HAL: %s",
                    esp_err_to_name(ret));

  ESP_LOGI(TAG,
           "PCA9685 created and initialized successfully with custom config for bus '%s'",
           bus_name);
  return ESP_OK;

cleanup:
  if (config_created && !config_added) {
    ESP_LOGW(TAG, "Destroying custom config that was created but not added to manager");
    pstar_bus_config_destroy(pca9685_i2c_config);
  }
  if (out_handle)
    *out_handle = NULL;
  return ret;
}

/* --- Pin Registration Functions --- */

esp_err_t pstar_pca9685_register_kconfig_pins(void)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering *first* default PCA9685 I2C/OE pins with validator (from KConfig)...");

  /* Register SDA pin */
  const char* sda_desc = "PCA9685 Default SDA";
  ret = pstar_register_pin(PCA9685_DEFAULT_SDA_PIN, sda_desc, true); /* I2C pins are often shared */
  ESP_RETURN_ON_ERROR(ret,
                      TAG,
                      "Failed to register PCA9685 default SDA pin (%d)",
                      PCA9685_DEFAULT_SDA_PIN);

  /* Register SCL pin */
  const char* scl_desc = "PCA9685 Default SCL";
  ret = pstar_register_pin(PCA9685_DEFAULT_SCL_PIN, scl_desc, true); /* I2C pins are often shared */
  ESP_RETURN_ON_ERROR(ret,
                      TAG,
                      "Failed to register PCA9685 default SCL pin (%d)",
                      PCA9685_DEFAULT_SCL_PIN);

  /* Register OE pin if enabled and valid */
#if PCA9685_DEFAULT_USE_OE_PIN
  if (PCA9685_DEFAULT_OE_PIN >= 0 && PCA9685_DEFAULT_OE_PIN < GPIO_NUM_MAX) {
    const char* oe_desc = "PCA9685 Default OE";
    /* OE pin is typically NOT shared if controlling multiple boards independently, */
    /* but if multiple boards share the same OE line, set can_be_shared=true. */
    /* Assuming shared OE for default config. */
    ret = pstar_register_pin(PCA9685_DEFAULT_OE_PIN, oe_desc, true);
    ESP_RETURN_ON_ERROR(ret,
                        TAG,
                        "Failed to register PCA9685 default OE pin (%d)",
                        PCA9685_DEFAULT_OE_PIN);
    ESP_LOGI(TAG, "Registered default OE pin: %d", PCA9685_DEFAULT_OE_PIN);
  } else {
    ESP_LOGW(TAG,
             "Default OE pin enabled but pin number (%d) is invalid. Skipping registration.",
             PCA9685_DEFAULT_OE_PIN);
  }
#endif

  ESP_LOGI(TAG, "PCA9685 default KConfig pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(
    TAG,
    "Pin validator or PCA9685 component disabled, skipping default PCA9685 pin registration.");
  return ESP_OK; /* Not an error if validator or component is disabled */
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED */
}

esp_err_t pstar_pca9685_register_custom_pins(int sda_pin, int scl_pin, int oe_pin)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG,
           "Registering custom PCA9685 pins (SDA:%d, SCL:%d, OE:%d) with validator...",
           sda_pin,
           scl_pin,
           oe_pin);

  /* Register SDA pin */
  const char* sda_desc = "PCA9685 Custom SDA";
  ret = pstar_register_pin(sda_pin, sda_desc, true); /* I2C pins are often shared */
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register PCA9685 custom SDA pin (%d)", sda_pin);

  /* Register SCL pin */
  const char* scl_desc = "PCA9685 Custom SCL";
  ret = pstar_register_pin(scl_pin, scl_desc, true); /* I2C pins are often shared */
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register PCA9685 custom SCL pin (%d)", scl_pin);

  /* Register OE pin if valid */
  if (oe_pin >= 0 && oe_pin < GPIO_NUM_MAX) {
    const char* oe_desc = "PCA9685 Custom OE";
    /* Assume custom OE pin might not be shared unless explicitly stated otherwise */
    ret = pstar_register_pin(oe_pin, oe_desc, false);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register PCA9685 custom OE pin (%d)", oe_pin);
    ESP_LOGI(TAG, "Registered custom OE pin: %d", oe_pin);
  } else {
    ESP_LOGD(TAG, "Custom OE pin (%d) is invalid or not used. Skipping registration.", oe_pin);
  }

  ESP_LOGI(TAG, "PCA9685 custom pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator disabled, skipping custom PCA9685 pin registration.");
  return ESP_OK; /* Not an error if validator is disabled */
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED */
}