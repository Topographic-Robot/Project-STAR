/* components/pstar_pca9685_hal/pstar_pca9685_hal.c */

#include "pstar_pca9685_hal.h"

#include "pstar_bus_config.h"
#include "pstar_bus_i2c.h"
#include "pstar_bus_manager.h"
#include "pstar_pin_validator.h"

#include "driver/gpio.h" /* Include GPIO driver for OE pin */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" /* For vTaskDelay */

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
#define PCA9685_LED0_ON_L 0x06  /**< LED0 output and brightness control byte 0 */
#define PCA9685_LED0_ON_H 0x07  /**< LED0 output and brightness control byte 1 */
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 output and brightness control byte 2 */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 output and brightness control byte 3 */
/* Registers for LED1_*, LED2_*, ... LED15_* follow sequentially (4 bytes each) */
#define PCA9685_ALL_LED_ON_L 0xFA  /**< load all the LEDn_ON registers, byte 0 */
#define PCA9685_ALL_LED_ON_H 0xFB  /**< load all the LEDn_ON registers, byte 1 */
#define PCA9685_ALL_LED_OFF_L 0xFC /**< load all the LEDn_OFF registers, byte 0 */
#define PCA9685_ALL_LED_OFF_H 0xFD /**< load all the LEDn_OFF registers, byte 1 */
#define PCA9685_PRE_SCALE 0xFE     /**< prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF      /**< defines the test mode to be entered */

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
#define PCA9685_OSC_CLOCK_MHZ 25.0f /**< Internal oscillator frequency in MHz */
#define PCA9685_WAKEUP_DELAY_US 500 /**< Delay needed after waking from sleep (datasheet) */
#define MAX_DEFAULT_BOARDS 16       /**< Sanity limit for default init count */
#define BUS_NAME_MAX_LEN 32         /**< Max length for generated bus names */

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
#endif /* CONFIG_PSTAR_KCONFIG_PCA9685_ENABLED */

/* --- Internal Handle Structure --- */
typedef struct pstar_pca9685_hal_dev_t {
  const pstar_bus_manager_t* bus_manager;     /**< Pointer to the bus manager */
  char*                      bus_name;        /**< Name of the bus config (copied) */
  float                      current_freq_hz; /**< Currently configured PWM frequency */
  gpio_num_t                 oe_pin;          /**< GPIO number for OE pin, or < 0 if not used */
  bool                       oe_active_low;   /**< True if OE pin is active low */
  bool                       output_enabled;  /**< Current state of the output enable */
} pstar_pca9685_hal_dev_t;

/* --- Helper Functions --- */

/**
 * @brief Write a single byte to a PCA9685 register.
 *
 * @param handle PCA9685 HAL handle.
 * @param reg_addr Register address to write to.
 * @param value Byte value to write.
 * @return esp_err_t ESP_OK on success, or error from pstar_bus_i2c_write.
 */
static esp_err_t
priv_pca9685_write_byte(pstar_pca9685_hal_handle_t handle, uint8_t reg_addr, uint8_t value)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  /* Use the bus manager's write function, writing 1 byte (value) to the specified register (reg_addr) */
  return pstar_bus_i2c_write(handle->bus_manager, handle->bus_name, &value, 1, reg_addr, NULL);
}

/**
 * @brief Read a single byte from a PCA9685 register.
 *
 * @param handle PCA9685 HAL handle.
 * @param reg_addr Register address to read from.
 * @param[out] value Pointer to store the read byte value.
 * @return esp_err_t ESP_OK on success, or error from pstar_bus_i2c_read.
 */
static esp_err_t
priv_pca9685_read_byte(pstar_pca9685_hal_handle_t handle, uint8_t reg_addr, uint8_t* value)
{
  ESP_RETURN_ON_FALSE(handle && value,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Device handle or value pointer is NULL");
  /* Use the bus manager's read function, reading 1 byte into 'value' from the specified register (reg_addr) */
  return pstar_bus_i2c_read(handle->bus_manager, handle->bus_name, value, 1, reg_addr, NULL);
}

/**
 * @brief Configure the OE GPIO pin.
 *
 * @param handle PCA9685 HAL handle containing OE pin info.
 * @return esp_err_t ESP_OK on success, or error from GPIO configuration.
 */
static esp_err_t priv_pca9685_configure_oe_pin(pstar_pca9685_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");

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

  /* Store OE pin configuration */
  dev->oe_pin         = oe_pin;
  dev->oe_active_low  = oe_active_low;
  dev->output_enabled = false; /* Start disabled */

  /* --- Configure OE Pin (if applicable) --- */
  ret = priv_pca9685_configure_oe_pin(dev);
  ESP_GOTO_ON_ERROR(ret, init_fail, TAG, "Failed to configure OE pin: %s", esp_err_to_name(ret));

  /* --- PCA9685 Initialization Sequence --- */
  ESP_LOGI(TAG, "Initializing PCA9685 on bus '%s'...", dev->bus_name);

  /* 1. Reset MODE1 register to default value (clears SLEEP bit, enables auto-increment) */
  ret = priv_pca9685_write_byte(dev, PCA9685_MODE1, PCA9685_MODE1_AI); /* Enable auto-increment */
  ESP_GOTO_ON_ERROR(ret, init_fail, TAG, "Failed to reset MODE1: %s", esp_err_to_name(ret));
  vTaskDelay(pdMS_TO_TICKS(5)); /* Allow oscillator to stabilize after potential reset/power-on */

  /* 2. Set MODE2 register to default (totem pole output) */
  ret = priv_pca9685_write_byte(dev, PCA9685_MODE2, PCA9685_MODE2_OUTDRV);
  ESP_GOTO_ON_ERROR(ret, init_fail, TAG, "Failed to set MODE2: %s", esp_err_to_name(ret));

  /* 3. Set initial PWM frequency */
  ret = pstar_pca9685_hal_set_pwm_freq(dev, initial_freq_hz);
  ESP_GOTO_ON_ERROR(ret,
                    init_fail,
                    TAG,
                    "Failed to set initial PWM frequency: %s",
                    esp_err_to_name(ret));

  /* 4. Set all PWM channels OFF initially */
  ret = pstar_pca9685_hal_set_all_pwm_values(dev, 0, 0x1000); /* Full OFF */
  ESP_GOTO_ON_ERROR(ret,
                    init_fail,
                    TAG,
                    "Failed to set all channels OFF: %s",
                    esp_err_to_name(ret));

  /* 5. Enable output if OE pin is configured (optional initial state, could leave disabled) */
  /* ret = pstar_pca9685_hal_output_enable(dev); */
  /* ESP_GOTO_ON_ERROR(ret, init_fail, TAG, "Failed to enable output initially: %s", esp_err_to_name(ret)); */

  ESP_LOGI(TAG,
           "PCA9685 initialized successfully on bus '%s' at %.2f Hz (OE Pin: %d)",
           dev->bus_name,
           dev->current_freq_hz,
           dev->oe_pin);
  *out_handle = dev;
  return ESP_OK;

init_fail:
  ESP_LOGE(TAG, "PCA9685 initialization failed on bus '%s'", dev->bus_name);
  if (dev) {
    if (dev->bus_name) {
      free(dev->bus_name);
    }
    /* No need to deconfigure GPIO here, system handles it. */
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

  if (disable_output) {
    esp_err_t disable_ret = pstar_pca9685_hal_output_disable(handle);
    if (disable_ret != ESP_OK &&
        disable_ret != ESP_ERR_INVALID_STATE) { /* Ignore if OE not configured */
      ESP_LOGE(TAG, "Failed to disable output during deinit: %s", esp_err_to_name(disable_ret));
    } else if (disable_ret == ESP_OK) {
      ESP_LOGD(TAG, "Output disabled via OE pin.");
    }
  }

  if (sleep) {
    esp_err_t sleep_ret = pstar_pca9685_hal_sleep(handle);
    if (sleep_ret != ESP_OK) {
      /* Log error but continue deinit */
      ESP_LOGE(TAG, "Failed to send Sleep command during deinit: %s", esp_err_to_name(sleep_ret));
    } else {
      ESP_LOGD(TAG, "Sent Sleep command.");
    }
  }

  /* No need to explicitly deconfigure GPIO, driver handles it. */

  if (handle->bus_name) {
    free(handle->bus_name);
  }
  free(handle);
  return ESP_OK;
}

esp_err_t pstar_pca9685_hal_set_pwm_freq(pstar_pca9685_hal_handle_t handle, float freq_hz)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGD(TAG, "Setting PWM frequency to %.2f Hz for '%s'", freq_hz, handle->bus_name);

  /* Clamp frequency to datasheet limits */
  if (freq_hz < 24.0f)
    freq_hz = 24.0f;
  if (freq_hz > 1526.0f)
    freq_hz = 1526.0f;

  /* Calculate prescale value */
  float   prescale_float = (PCA9685_OSC_CLOCK_MHZ * 1000000.0f) / (4096.0f * freq_hz) - 1.0f;
  uint8_t prescale       = (uint8_t)floor(prescale_float + 0.5f); /* Round to nearest integer */

  ESP_LOGD(TAG, "Calculated prescale value: %d for %.2f Hz", prescale, freq_hz);

  esp_err_t ret;
  uint8_t   old_mode1;

  /* 1. Read current MODE1 register */
  ret = priv_pca9685_read_byte(handle, PCA9685_MODE1, &old_mode1);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read MODE1 before setting frequency");

  /* 2. Set SLEEP bit in MODE1 to allow changing the prescaler */
  uint8_t new_mode1 = (old_mode1 & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
  ret               = priv_pca9685_write_byte(handle, PCA9685_MODE1, new_mode1);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to enter sleep mode before setting frequency");
  vTaskDelay(pdMS_TO_TICKS(1)); /* Wait >500ns for oscillator stop */

  /* 3. Write the prescale value */
  ret = priv_pca9685_write_byte(handle, PCA9685_PRE_SCALE, prescale);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write prescale value");

  /* 4. Restore original MODE1 (clearing SLEEP) */
  ret = priv_pca9685_write_byte(handle,
                                PCA9685_MODE1,
                                old_mode1 & ~PCA9685_MODE1_SLEEP); /* Ensure sleep is cleared */
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to restore MODE1 after setting frequency");
  vTaskDelay(pdMS_TO_TICKS(1)); /* Wait >500us for oscillator to wake up */

  /* 5. Set RESTART bit to restart PWM channels with the new frequency */
  ret = priv_pca9685_write_byte(handle, PCA9685_MODE1, old_mode1 | PCA9685_MODE1_RESTART);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set RESTART bit after setting frequency");

  /* Store the actual frequency set (based on rounded prescale) */
  handle->current_freq_hz = (PCA9685_OSC_CLOCK_MHZ * 1000000.0f) / (4096.0f * (prescale + 1));
  ESP_LOGD(TAG,
           "Frequency set for '%s'. Actual frequency: %.2f Hz",
           handle->bus_name,
           handle->current_freq_hz);

  return ESP_OK;
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

  /* Mask values to 13 bits (12 bits timer + 1 bit full ON/OFF) */
  on_value &= 0x1FFF;
  off_value &= 0x1FFF;

  ESP_LOGD(TAG,
           "Setting channel %d on '%s' to ON=0x%04X, OFF=0x%04X",
           channel,
           handle->bus_name,
           on_value,
           off_value);

  esp_err_t ret;
  uint8_t   reg_base =
    PCA9685_LED0_ON_L + (4 * channel); /* Calculate base register address for the channel */

  /* Write ON_L, ON_H, OFF_L, OFF_H using auto-increment (AI bit must be set in MODE1) */
  uint8_t data[4] = {
    (uint8_t)(on_value & 0xFF),  /* ON_L */
    (uint8_t)(on_value >> 8),    /* ON_H (includes full ON bit) */
    (uint8_t)(off_value & 0xFF), /* OFF_L */
    (uint8_t)(off_value >> 8)    /* OFF_H (includes full OFF bit) */
  };

  ret = pstar_bus_i2c_write(handle->bus_manager, handle->bus_name, data, 4, reg_base, NULL);
  ESP_RETURN_ON_ERROR(ret,
                      TAG,
                      "Failed to write PWM values for channel %d on '%s'",
                      channel,
                      handle->bus_name);

  return ESP_OK;
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

  uint16_t on_value  = 0;
  uint16_t off_value = 0;

  if (duty_cycle_percent >= 100.0f) {
    /* Special case for 100% duty cycle: Set Full ON bit (bit 12 of ON value) */
    on_value  = 0x1000;
    off_value = 0; /* Off value doesn't matter when Full ON is set */
  } else if (duty_cycle_percent <= 0.0f) {
    /* Special case for 0% duty cycle: Set Full OFF bit (bit 12 of OFF value) */
    on_value  = 0; /* On value doesn't matter when Full OFF is set */
    off_value = 0x1000;
  } else {
    /* Calculate OFF value based on percentage of the 4096 clock cycles */
    /* Ensure calculation uses float division */
    off_value = (uint16_t)roundf((duty_cycle_percent / 100.0f) * 4096.0f);
    /* Clamp to max value (0xFFF = 4095) */
    if (off_value > PCA9685_MAX_PWM_VALUE) {
      off_value = PCA9685_MAX_PWM_VALUE;
    }
    /* Standard PWM: turn ON at time 0, turn OFF at calculated time */
    on_value = 0;
  }
  /* Use the function that handles the full ON/OFF bits correctly */
  return pstar_pca9685_hal_set_channel_pwm_values(handle, channel, on_value, off_value);
}

esp_err_t pstar_pca9685_hal_set_all_pwm_values(pstar_pca9685_hal_handle_t handle,
                                               uint16_t                   on_value,
                                               uint16_t                   off_value)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");

  /* Mask values to 13 bits */
  on_value &= 0x1FFF;
  off_value &= 0x1FFF;

  ESP_LOGD(TAG,
           "Setting all channels on '%s' to ON=0x%04X, OFF=0x%04X",
           handle->bus_name,
           on_value,
           off_value);

  esp_err_t ret;
  uint8_t   reg_base = PCA9685_ALL_LED_ON_L; /* Base register for ALL_LED control */

  /* Write ALL_LED_ON_L, ALL_LED_ON_H, ALL_LED_OFF_L, ALL_LED_OFF_H using auto-increment */
  uint8_t data[4] = {
    (uint8_t)(on_value & 0xFF),  /* ON_L */
    (uint8_t)(on_value >> 8),    /* ON_H */
    (uint8_t)(off_value & 0xFF), /* OFF_L */
    (uint8_t)(off_value >> 8)    /* OFF_H */
  };

  ret = pstar_bus_i2c_write(handle->bus_manager, handle->bus_name, data, 4, reg_base, NULL);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write ALL_LED PWM values for '%s'", handle->bus_name);

  return ESP_OK;
}

esp_err_t pstar_pca9685_hal_restart(pstar_pca9685_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGD(TAG, "Restarting PCA9685 on '%s'", handle->bus_name);

  esp_err_t ret;
  uint8_t   mode1;

  /* Read current MODE1 */
  ret = priv_pca9685_read_byte(handle, PCA9685_MODE1, &mode1);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read MODE1 before restart");

  /* Check if restart bit is already set */
  if (mode1 & PCA9685_MODE1_RESTART) {
    /* If restart is set, it might mean the chip is already restarting or stuck. */
    /* Try clearing sleep first, then set restart. */
    mode1 &= ~PCA9685_MODE1_SLEEP;
    ret = priv_pca9685_write_byte(handle, PCA9685_MODE1, mode1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to clear sleep before restart");
    vTaskDelay(pdMS_TO_TICKS(1)); /* Small delay */
  }

  /* Set the RESTART bit (while keeping sleep clear and AI set) */
  uint8_t restart_mode = (mode1 & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART | PCA9685_MODE1_AI;
  ret                  = priv_pca9685_write_byte(handle, PCA9685_MODE1, restart_mode);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set RESTART bit");

  vTaskDelay(pdMS_TO_TICKS(1)); /* Give time for restart sequence */

  return ESP_OK;
}

esp_err_t pstar_pca9685_hal_sleep(pstar_pca9685_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGD(TAG, "Putting PCA9685 on '%s' to sleep", handle->bus_name);

  esp_err_t ret;
  uint8_t   mode1;

  ret = priv_pca9685_read_byte(handle, PCA9685_MODE1, &mode1);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read MODE1 before sleep");

  /* Set the SLEEP bit (bit 4), keep other bits as they were (except clear RESTART) */
  mode1 = (mode1 & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
  ret   = priv_pca9685_write_byte(handle, PCA9685_MODE1, mode1);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write MODE1 to enter sleep");

  vTaskDelay(pdMS_TO_TICKS(1)); /* Allow oscillator to stop (>500ns) */
  return ESP_OK;
}

esp_err_t pstar_pca9685_hal_wakeup(pstar_pca9685_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGD(TAG, "Waking up PCA9685 on '%s'", handle->bus_name);

  esp_err_t ret;
  uint8_t   mode1;

  ret = priv_pca9685_read_byte(handle, PCA9685_MODE1, &mode1);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read MODE1 before wakeup");

  /* Clear the SLEEP bit (bit 4) */
  uint8_t wakeup_mode = mode1 & ~PCA9685_MODE1_SLEEP;
  ret                 = priv_pca9685_write_byte(handle, PCA9685_MODE1, wakeup_mode);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write MODE1 to wake up");

  /* Wait for oscillator to stabilize (datasheet recommends 500us) */
  vTaskDelay(pdMS_TO_TICKS(1)); /* Minimum delay, usually enough */

  /* After waking up, the RESTART bit might need to be set to resume PWM outputs smoothly. */
  ret = pstar_pca9685_hal_restart(handle);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to restart after wakeup");

  return ESP_OK;
}

esp_err_t pstar_pca9685_hal_output_enable(pstar_pca9685_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  if (handle->oe_pin < 0) {
    ESP_LOGD(TAG, "Output enable requested for '%s', but OE pin not configured.", handle->bus_name);
    return ESP_ERR_INVALID_STATE; /* Indicate OE pin not available */
  }

  uint32_t  active_level = handle->oe_active_low ? 0 : 1;
  esp_err_t ret          = gpio_set_level(handle->oe_pin, active_level);
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
  return ret;
}

esp_err_t pstar_pca9685_hal_output_disable(pstar_pca9685_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  if (handle->oe_pin < 0) {
    ESP_LOGD(TAG,
             "Output disable requested for '%s', but OE pin not configured.",
             handle->bus_name);
    return ESP_ERR_INVALID_STATE; /* Indicate OE pin not available */
  }

  uint32_t  inactive_level = handle->oe_active_low ? 1 : 0;
  esp_err_t ret            = gpio_set_level(handle->oe_pin, inactive_level);
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
  return ret;
}

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
    ret = pstar_bus_config_init(pca9685_i2c_config);
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
      if (loop_i2c_config->type != k_pstar_bus_type_i2c ||
          loop_i2c_config->i2c.port != PCA9685_DEFAULT_I2C_PORT) {
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
    if (!loop_i2c_config
           ->initialized) {         // Check if THIS specific config object is marked initialized
      if (!port_driver_installed) { // Check if WE have installed the driver for this port
        ESP_LOGI(TAG,
                 "Initializing I2C driver for Port %d (Board %d).",
                 PCA9685_DEFAULT_I2C_PORT,
                 i);
        loop_ret = pstar_bus_config_init(loop_i2c_config); // This calls i2c_driver_install
        if (loop_ret != ESP_OK) {
          ESP_LOGE(TAG,
                   "Failed to init bus hardware for board %d ('%s'): %s",
                   i,
                   loop_bus_name,
                   esp_err_to_name(loop_ret));
          goto loop_cleanup;
        }
        port_driver_installed = true; // Mark driver as installed for this port
      } else {
        ESP_LOGI(
          TAG,
          "I2C driver for Port %d already installed. Marking config '%s' as initialized without calling i2c_driver_install again.",
          PCA9685_DEFAULT_I2C_PORT,
          loop_bus_name);
        // Manually mark this config as initialized since the underlying driver is ready
        loop_i2c_config->initialized = true;
        loop_ret                     = ESP_OK; // Pretend init was successful for this config object
      }
    } else {
      ESP_LOGI(TAG, "Bus config '%s' already marked as initialized.", loop_bus_name);
      // If the config object was already initialized, assume the port driver is too
      port_driver_installed = true; // Ensure flag is set if we found an initialized config
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
      /* Note: Bus config remains in manager even if HAL init fails. Cleanup should handle it. */
      goto loop_cleanup;
    }

    ESP_LOGI(TAG, "Successfully initialized default PCA9685 board %d ('%s')", i, loop_bus_name);

  loop_cleanup:
    if (loop_ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to initialize default PCA9685 board %d.", i);
      if (first_error == ESP_OK) {
        first_error = loop_ret; /* Record the first error encountered */
      }
      /* If we created the config but failed to add/init it, destroy it */
      if (loop_config_created && !loop_config_added) {
        ESP_LOGW(TAG,
                 "Destroying config '%s' due to initialization failure for board %d",
                 loop_bus_name,
                 i);
        pstar_bus_config_destroy(loop_i2c_config); // Clean up the created config
      }
      out_handles[i] = NULL; /* Ensure handle is NULL on failure */
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
    ret = pstar_bus_config_init(pca9685_i2c_config);
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