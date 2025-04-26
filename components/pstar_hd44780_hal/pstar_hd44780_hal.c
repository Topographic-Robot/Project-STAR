/* components/pstar_hd44780_hal/pstar_hd44780_hal.c */

#include "pstar_hd44780_hal.h"

#include "pstar_bus_config.h" /* Needed for I2C bus creation */
#include "pstar_bus_i2c.h"    /* Needed for I2C writes */
#include "pstar_pin_validator.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" /* For mutex */
#include "freertos/task.h"   /* For vTaskDelay */

#include <stdarg.h> /* For va_list, va_start, va_end */
#include <stdio.h>  /* For vsnprintf */
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "rom/ets_sys.h" /* For ets_delay_us */

static const char* TAG = "HD44780 HAL";

/* HD44780 Commands (same as before) */
#define LCD_CMD_CLEAR_DISPLAY 0x01
#define LCD_CMD_RETURN_HOME 0x02
#define LCD_CMD_ENTRY_MODE_SET 0x04
#define LCD_CMD_DISPLAY_CONTROL 0x08
#define LCD_CMD_CURSOR_SHIFT 0x10
#define LCD_CMD_FUNCTION_SET 0x20
#define LCD_CMD_SET_CGRAM_ADDR 0x40
#define LCD_CMD_SET_DDRAM_ADDR 0x80

/* Entry Mode Set Flags (same as before) */
#define LCD_ENTRY_RIGHT 0x00
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_INCREMENT 0x01
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

/* Display Control Flags (same as before) */
#define LCD_DISPLAY_ON 0x04
#define LCD_DISPLAY_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_ON 0x01
#define LCD_BLINK_OFF 0x00

/* Function Set Flags (same as before) */
#define LCD_8BIT_MODE 0x10
#define LCD_4BIT_MODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

/* Timing Constants (microseconds) - Apply to both modes where applicable */
#define LCD_DELAY_ENABLE_PULSE 4 /* Min 1us, use slightly more */
#define LCD_DELAY_COMMAND 40     /* Most commands take ~37us, Clear/Home take longer */
#define LCD_DELAY_I2C_COMMAND 50 /* Slightly longer delay for I2C operations */
#define LCD_DELAY_CLEAR 2000     /* Clear/Home command needs ~1.52ms */
#define LCD_DELAY_INIT 5000      /* Delay after power on and during init sequence */
#define LCD_DELAY_I2C_PULSE 5    /* Delay between I2C E pin toggles */

#define HD44780_MUTEX_TIMEOUT_MS 1000 /* Timeout for acquiring HAL mutex */
#define HD44780_PRINTF_BUFFER_SIZE 64 /* Buffer size for printf */

/* --- PCF8574 Backpack Bit Definitions (Typical Mapping) --- */
#define PCF8574_BIT_RS (1 << 0)        /* P0 */
#define PCF8574_BIT_RW (1 << 1)        /* P1 - Often tied to GND, but reserve the bit */
#define PCF8574_BIT_E (1 << 2)         /* P2 */
#define PCF8574_BIT_BACKLIGHT (1 << 3) /* P3 */
#define PCF8574_BIT_D4 (1 << 4)        /* P4 */
#define PCF8574_BIT_D5 (1 << 5)        /* P5 */
#define PCF8574_BIT_D6 (1 << 6)        /* P6 */
#define PCF8574_BIT_D7 (1 << 7)        /* P7 */

/* --- Internal Handle Structure --- */
typedef struct pstar_hd44780_hal_dev_t {
  pstar_hd44780_interface_mode_t mode; /**< Interface mode */
  uint8_t                        rows;
  uint8_t                        cols;
  uint8_t                        display_ctrl_flags;
  uint8_t                        entry_mode_flags;
  SemaphoreHandle_t              mutex;

  /* Parallel Mode Fields */
  gpio_num_t rs_pin;
  gpio_num_t e_pin;
  gpio_num_t d4_pin;
  gpio_num_t d5_pin;
  gpio_num_t d6_pin;
  gpio_num_t d7_pin;

  /* I2C Mode Fields */
  const pstar_bus_manager_t* i2c_bus_manager; /**< Pointer to the bus manager */
  char*                      i2c_bus_name;    /**< Name of the bus config (copied) */
  uint8_t                    i2c_addr;        /**< Backpack I2C Address */
  uint8_t i2c_backpack_state;                 /**< Current state of PCF8574 pins (incl backlight) */
} pstar_hd44780_hal_dev_t;

/* --- Helper Functions: Parallel Mode --- */

/** @brief Pulse the Enable (E) pin (Parallel mode, assumes mutex taken). */
static void priv_hd44780_parallel_pulse_enable_nolock(pstar_hd44780_hal_handle_t handle)
{
  gpio_set_level(handle->e_pin, 0);
  ets_delay_us(LCD_DELAY_ENABLE_PULSE);
  gpio_set_level(handle->e_pin, 1);
  ets_delay_us(LCD_DELAY_ENABLE_PULSE);
  gpio_set_level(handle->e_pin, 0);
  ets_delay_us(LCD_DELAY_COMMAND);
}

/** @brief Send 4 bits (Parallel mode, assumes mutex taken). */
static void priv_hd44780_parallel_write_4bits_nolock(pstar_hd44780_hal_handle_t handle,
                                                     uint8_t                    value)
{
  gpio_set_level(handle->d4_pin, (value >> 0) & 0x01);
  gpio_set_level(handle->d5_pin, (value >> 1) & 0x01);
  gpio_set_level(handle->d6_pin, (value >> 2) & 0x01);
  gpio_set_level(handle->d7_pin, (value >> 3) & 0x01);
  priv_hd44780_parallel_pulse_enable_nolock(handle);
}

/** @brief Send a full byte (Parallel mode, assumes mutex taken). */
static void priv_hd44780_parallel_send_byte_nolock(pstar_hd44780_hal_handle_t handle,
                                                   uint8_t                    value,
                                                   int                        rs_level)
{
  gpio_set_level(handle->rs_pin, rs_level);
  ets_delay_us(1);
  priv_hd44780_parallel_write_4bits_nolock(handle, value >> 4);
  priv_hd44780_parallel_write_4bits_nolock(handle, value & 0x0F);
}

/* --- Helper Functions: I2C Mode --- */

/** @brief Write a byte to the I2C backpack (assumes mutex taken). */
static esp_err_t priv_hd44780_i2c_write_backpack_nolock(pstar_hd44780_hal_handle_t handle,
                                                        uint8_t                    data)
{
  /* Update internal state *before* writing */
  handle->i2c_backpack_state = data;
  /* Use the bus manager's simple write command function */
  return pstar_bus_i2c_write_command(handle->i2c_bus_manager, handle->i2c_bus_name, data);
}

/** @brief Pulse the Enable (E) pin via I2C (assumes mutex taken). */
static esp_err_t priv_hd44780_i2c_pulse_enable_nolock(pstar_hd44780_hal_handle_t handle,
                                                      uint8_t                    data_with_e_low)
{
  esp_err_t ret;
  /* Set E high */
  ret = priv_hd44780_i2c_write_backpack_nolock(handle, data_with_e_low | PCF8574_BIT_E);
  if (ret != ESP_OK)
    return ret;
  ets_delay_us(LCD_DELAY_I2C_PULSE); /* Keep E high briefly */

  /* Set E low */
  ret = priv_hd44780_i2c_write_backpack_nolock(handle, data_with_e_low & ~PCF8574_BIT_E);
  if (ret != ESP_OK)
    return ret;
  ets_delay_us(LCD_DELAY_I2C_COMMAND); /* Command execution time */
  return ESP_OK;
}

/** @brief Send 4 bits via I2C (assumes mutex taken). */
static esp_err_t priv_hd44780_i2c_write_4bits_nolock(pstar_hd44780_hal_handle_t handle,
                                                     uint8_t                    value)
{
  uint8_t data = 0;
  /* Map 4 bits to PCF8574 D4-D7 */
  if ((value >> 0) & 0x01)
    data |= PCF8574_BIT_D4;
  if ((value >> 1) & 0x01)
    data |= PCF8574_BIT_D5;
  if ((value >> 2) & 0x01)
    data |= PCF8574_BIT_D6;
  if ((value >> 3) & 0x01)
    data |= PCF8574_BIT_D7;

  /* Preserve other bits (RS, Backlight) from current state */
  data |= (handle->i2c_backpack_state & (PCF8574_BIT_RS | PCF8574_BIT_RW | PCF8574_BIT_BACKLIGHT));

  /* Pulse E */
  return priv_hd44780_i2c_pulse_enable_nolock(handle, data);
}

/** @brief Send a full byte via I2C (assumes mutex taken). */
static esp_err_t
priv_hd44780_i2c_send_byte_nolock(pstar_hd44780_hal_handle_t handle, uint8_t value, int rs_level)
{
  esp_err_t ret;
  /* Assume RW is low (tied to GND on most backpacks) */

  /* Update internal state for RS bit */
  if (rs_level) {
    handle->i2c_backpack_state |= PCF8574_BIT_RS;
  } else {
    handle->i2c_backpack_state &= ~PCF8574_BIT_RS;
  }

  /* Send high nibble */
  ret = priv_hd44780_i2c_write_4bits_nolock(handle, value >> 4);
  if (ret != ESP_OK)
    return ret;

  /* Send low nibble */
  ret = priv_hd44780_i2c_write_4bits_nolock(handle, value & 0x0F);
  return ret;
}

/**
 * @brief Internal common command sender (handles mutex and mode).
 */
static esp_err_t priv_hd44780_send_command_internal(pstar_hd44780_hal_handle_t handle,
                                                    uint8_t                    command)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(HD44780_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for send_command");
    return ESP_ERR_TIMEOUT;
  }

  if (handle->mode == HD44780_MODE_PARALLEL) {
    priv_hd44780_parallel_send_byte_nolock(handle, command, 0);
  } else { /* HD44780_MODE_I2C */
    ret = priv_hd44780_i2c_send_byte_nolock(handle, command, 0);
  }

  /* Add extra delay for clear/home commands */
  if (command == LCD_CMD_CLEAR_DISPLAY || command == LCD_CMD_RETURN_HOME) {
    ets_delay_us(LCD_DELAY_CLEAR);
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

/**
 * @brief Internal common data sender (handles mutex and mode).
 */
static esp_err_t priv_hd44780_send_data_internal(pstar_hd44780_hal_handle_t handle, uint8_t data)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(HD44780_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for send_data");
    return ESP_ERR_TIMEOUT;
  }

  if (handle->mode == HD44780_MODE_PARALLEL) {
    priv_hd44780_parallel_send_byte_nolock(handle, data, 1);
  } else { /* HD44780_MODE_I2C */
    ret = priv_hd44780_i2c_send_byte_nolock(handle, data, 1);
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

/* --- Public Function Implementations --- */

esp_err_t pstar_hd44780_hal_init(const pstar_bus_manager_t*        manager,
                                 const pstar_hd44780_hal_config_t* config,
                                 pstar_hd44780_hal_handle_t*       out_handle)
{
  ESP_RETURN_ON_FALSE(config && out_handle, ESP_ERR_INVALID_ARG, TAG, "Config or out_handle NULL");
  ESP_RETURN_ON_FALSE(config->rows > 0 && config->cols > 0,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Rows and cols must be > 0");
  if (config->mode == HD44780_MODE_I2C) {
    ESP_RETURN_ON_FALSE(manager && config->i2c_bus_name,
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "I2C mode requires valid manager and bus name");
  }
  *out_handle = NULL;

  /* Allocate memory for the device handle */
  pstar_hd44780_hal_handle_t dev =
    (pstar_hd44780_hal_handle_t)malloc(sizeof(pstar_hd44780_hal_dev_t));
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Failed to allocate device handle");
  memset(dev, 0, sizeof(pstar_hd44780_hal_dev_t));

  dev->mode  = config->mode;
  dev->rows  = config->rows;
  dev->cols  = config->cols;
  dev->mutex = NULL; /* Initialize before creation */

  /* Create mutex */
  dev->mutex = xSemaphoreCreateMutex();
  if (!dev->mutex) {
    ESP_LOGE(TAG, "Failed to create mutex");
    free(dev);
    return ESP_ERR_NO_MEM;
  }

  /* Take mutex for initialization */
  if (xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(HD44780_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex during init");
    vSemaphoreDelete(dev->mutex);
    free(dev);
    return ESP_ERR_TIMEOUT;
  }

  esp_err_t ret = ESP_OK;

  /* --- Mode-Specific Initialization --- */
  if (dev->mode == HD44780_MODE_PARALLEL) {
    ESP_LOGI(TAG, "Initializing in Parallel Mode...");
    dev->rs_pin = config->rs_pin;
    dev->e_pin  = config->e_pin;
    dev->d4_pin = config->d4_pin;
    dev->d5_pin = config->d5_pin;
    dev->d6_pin = config->d6_pin;
    dev->d7_pin = config->d7_pin;

    /* Validate GPIO pins */
    ESP_GOTO_ON_FALSE(
      GPIO_IS_VALID_OUTPUT_GPIO(dev->rs_pin) && GPIO_IS_VALID_OUTPUT_GPIO(dev->e_pin) &&
        GPIO_IS_VALID_OUTPUT_GPIO(dev->d4_pin) && GPIO_IS_VALID_OUTPUT_GPIO(dev->d5_pin) &&
        GPIO_IS_VALID_OUTPUT_GPIO(dev->d6_pin) && GPIO_IS_VALID_OUTPUT_GPIO(dev->d7_pin),
      ESP_ERR_INVALID_ARG,
      init_fail_mutex,
      TAG,
      "One or more parallel GPIO pins are invalid");

    /* Configure GPIO pins */
    gpio_config_t io_conf;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << dev->rs_pin) | (1ULL << dev->e_pin) | (1ULL << dev->d4_pin) |
                           (1ULL << dev->d5_pin) | (1ULL << dev->d6_pin) | (1ULL << dev->d7_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    ret                  = gpio_config(&io_conf);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to configure parallel GPIO pins: %s", esp_err_to_name(ret));
      goto init_fail_mutex;
    }

    /* Parallel Initialization Sequence (Simplified - uses internal function) */
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(dev->rs_pin, 0);
    gpio_set_level(dev->e_pin, 0);
    priv_hd44780_parallel_write_4bits_nolock(dev, 0x03);
    vTaskDelay(pdMS_TO_TICKS(5));
    priv_hd44780_parallel_write_4bits_nolock(dev, 0x03);
    ets_delay_us(150);
    priv_hd44780_parallel_write_4bits_nolock(dev, 0x03);
    ets_delay_us(LCD_DELAY_COMMAND);
    priv_hd44780_parallel_write_4bits_nolock(dev, 0x02); /* Set 4-bit mode */
    ets_delay_us(LCD_DELAY_COMMAND);

  } else { /* HD44780_MODE_I2C */
    ESP_LOGI(TAG, "Initializing in I2C Mode...");
    dev->i2c_bus_manager = manager;
    dev->i2c_bus_name    = strdup(config->i2c_bus_name);
    ESP_GOTO_ON_FALSE(dev->i2c_bus_name, ESP_ERR_NO_MEM, init_fail_mutex, TAG, "strdup failed");

    /* Find the I2C bus config to get the address */
    pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, dev->i2c_bus_name);
    ESP_GOTO_ON_FALSE(bus_config,
                      ESP_ERR_NOT_FOUND,
                      init_fail_mutex,
                      TAG,
                      "Bus '%s' not found",
                      dev->i2c_bus_name);
    ESP_GOTO_ON_FALSE(bus_config->initialized,
                      ESP_ERR_INVALID_STATE,
                      init_fail_mutex,
                      TAG,
                      "Bus '%s' not init",
                      dev->i2c_bus_name);
    ESP_GOTO_ON_FALSE(bus_config->type == k_pstar_bus_type_i2c,
                      ESP_ERR_INVALID_ARG,
                      init_fail_mutex,
                      TAG,
                      "Bus '%s' not I2C",
                      dev->i2c_bus_name);
    /* --- CORRECTED ACCESS --- */
    dev->i2c_addr = bus_config->proto.i2c.address; /* Get address from the found bus config */

    /* Initialize backpack state (Backlight ON/OFF based on config) */
    dev->i2c_backpack_state = config->i2c_backlight_on ? PCF8574_BIT_BACKLIGHT : 0;
    ret =
      priv_hd44780_i2c_write_backpack_nolock(dev, dev->i2c_backpack_state); /* Send initial state */
    ESP_GOTO_ON_ERROR(ret, init_fail_mutex, TAG, "Failed to set initial I2C backpack state");

    /* I2C Initialization Sequence (uses internal functions) */
    vTaskDelay(pdMS_TO_TICKS(50));
    priv_hd44780_i2c_write_4bits_nolock(dev, 0x03); /* RS low is default state */
    vTaskDelay(pdMS_TO_TICKS(5));
    priv_hd44780_i2c_write_4bits_nolock(dev, 0x03);
    ets_delay_us(150);
    priv_hd44780_i2c_write_4bits_nolock(dev, 0x03);
    ets_delay_us(LCD_DELAY_I2C_COMMAND);
    priv_hd44780_i2c_write_4bits_nolock(dev, 0x02); /* Set 4-bit mode */
    ets_delay_us(LCD_DELAY_I2C_COMMAND);
  }

  /* --- Common Initialization Commands (Post 4-bit mode set) --- */
  uint8_t func_set = LCD_4BIT_MODE | LCD_5x8DOTS;
  if (dev->rows > 1) {
    func_set |= LCD_2LINE;
  } else {
    func_set |= LCD_1LINE;
  }
  /* Use internal command sender which handles mode and mutex release/acquire */
  xSemaphoreGive(dev->mutex); /* Release mutex before calling public API functions */
  ret = pstar_hd44780_hal_send_command(dev, LCD_CMD_FUNCTION_SET | func_set);
  if (ret != ESP_OK)
    goto init_fail_no_mutex; /* Mutex already given */

  dev->display_ctrl_flags = LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF;
  ret = pstar_hd44780_hal_send_command(dev, LCD_CMD_DISPLAY_CONTROL | dev->display_ctrl_flags);
  if (ret != ESP_OK)
    goto init_fail_no_mutex;

  ret = pstar_hd44780_hal_clear(dev);
  if (ret != ESP_OK)
    goto init_fail_no_mutex;

  dev->entry_mode_flags = LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT;
  ret = pstar_hd44780_hal_send_command(dev, LCD_CMD_ENTRY_MODE_SET | dev->entry_mode_flags);
  if (ret != ESP_OK)
    goto init_fail_no_mutex;

  /* Initialization complete */
  ESP_LOGI(TAG,
           "HD44780 Initialized in %s Mode",
           dev->mode == HD44780_MODE_I2C ? "I2C" : "Parallel");
  *out_handle = dev;
  return ESP_OK;

init_fail_mutex:
  xSemaphoreGive(dev->mutex); /* Release mutex on failure */
init_fail_no_mutex:
  if (dev) {
    if (dev->mutex) {
      vSemaphoreDelete(dev->mutex);
    }
    if (dev->mode == HD44780_MODE_I2C && dev->i2c_bus_name) {
      free(dev->i2c_bus_name);
    }
    free(dev);
  }
  *out_handle = NULL;
  return ret;
}

esp_err_t pstar_hd44780_hal_deinit(pstar_hd44780_hal_handle_t handle, bool clear_display)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGI(TAG,
           "Deinitializing HD44780 HAL (%s Mode)...",
           handle->mode == HD44780_MODE_I2C ? "I2C" : "Parallel");

  if (clear_display) {
    esp_err_t clear_ret = pstar_hd44780_hal_clear(handle); /* Uses public func (mutex safe) */
    if (clear_ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to clear display during deinit: %s", esp_err_to_name(clear_ret));
    }
  }

  if (handle->mode == HD44780_MODE_I2C) {
    /* Optionally turn off backlight */
    esp_err_t bl_ret = pstar_hd44780_hal_backlight_off(handle); /* Uses public func (mutex safe) */
    if (bl_ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to turn off backlight during deinit: %s", esp_err_to_name(bl_ret));
    }
  }

  /* Take mutex for final cleanup */
  if (handle->mutex &&
      xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(HD44780_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for final deinit cleanup");
    /* Try to continue cleanup anyway */
  }

  if (handle->mode == HD44780_MODE_PARALLEL) {
    /* Reset GPIO pins */
    gpio_reset_pin(handle->rs_pin);
    gpio_reset_pin(handle->e_pin);
    gpio_reset_pin(handle->d4_pin);
    gpio_reset_pin(handle->d5_pin);
    gpio_reset_pin(handle->d6_pin);
    gpio_reset_pin(handle->d7_pin);
  } else {
    if (handle->i2c_bus_name) {
      free(handle->i2c_bus_name);
      handle->i2c_bus_name = NULL;
    }
  }

  /* Delete mutex */
  if (handle->mutex) {
    vSemaphoreDelete(handle->mutex);
    handle->mutex = NULL;
  }

  free(handle);
  return ESP_OK;
}

/* --- Core Public Functions --- */
/* These now call the internal helpers that handle the mode switching */

esp_err_t pstar_hd44780_hal_send_command(pstar_hd44780_hal_handle_t handle, uint8_t command)
{
  return priv_hd44780_send_command_internal(handle, command);
}

esp_err_t pstar_hd44780_hal_send_data(pstar_hd44780_hal_handle_t handle, uint8_t data)
{
  return priv_hd44780_send_data_internal(handle, data);
}

esp_err_t pstar_hd44780_hal_clear(pstar_hd44780_hal_handle_t handle)
{
  return priv_hd44780_send_command_internal(handle, LCD_CMD_CLEAR_DISPLAY);
}

esp_err_t pstar_hd44780_hal_home(pstar_hd44780_hal_handle_t handle)
{
  return priv_hd44780_send_command_internal(handle, LCD_CMD_RETURN_HOME);
}

esp_err_t pstar_hd44780_hal_set_cursor(pstar_hd44780_hal_handle_t handle, uint8_t col, uint8_t row)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(row < handle->rows && col < handle->cols,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Cursor position (%d, %d) out of bounds (%dx%d)",
                      col,
                      row,
                      handle->cols,
                      handle->rows);

  /* Use existing row offsets logic */
  static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  uint8_t              ddram_addr;

  if (row >= sizeof(row_offsets) / sizeof(row_offsets[0])) {
    ESP_LOGE(TAG,
             "Row %d exceeds known offsets (max %zu)",
             row,
             sizeof(row_offsets) / sizeof(row_offsets[0]) - 1);
    return ESP_ERR_INVALID_ARG;
  }

  ddram_addr = row_offsets[row] + col;

  /* Use internal command sender */
  return priv_hd44780_send_command_internal(handle, LCD_CMD_SET_DDRAM_ADDR | ddram_addr);
}

esp_err_t pstar_hd44780_hal_print_string(pstar_hd44780_hal_handle_t handle, const char* str)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(str, ESP_ERR_INVALID_ARG, TAG, "String pointer is NULL");

  esp_err_t ret = ESP_OK;
  while (*str && ret == ESP_OK) {
    ret = priv_hd44780_send_data_internal(handle, *str++); /* Use internal sender */
  }
  return ret;
}

esp_err_t pstar_hd44780_hal_printf(pstar_hd44780_hal_handle_t handle, const char* format, ...)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(format, ESP_ERR_INVALID_ARG, TAG, "Format string is NULL");

  char      buffer[HD44780_PRINTF_BUFFER_SIZE];
  va_list   args;
  esp_err_t ret = ESP_OK;

  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (len < 0) {
    ESP_LOGE(TAG, "printf formatting error");
    return ESP_FAIL;
  }
  if (len >= sizeof(buffer)) {
    ESP_LOGW(TAG, "printf output truncated (%d vs %d)", len, sizeof(buffer) - 1);
  }

  /* Use print_string which now uses the internal data sender */
  ret = pstar_hd44780_hal_print_string(handle, buffer);

  return ret;
}

esp_err_t pstar_hd44780_hal_backlight_on(pstar_hd44780_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  if (handle->mode != HD44780_MODE_I2C) {
    ESP_LOGD(TAG, "Backlight control only available in I2C mode.");
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(HD44780_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for backlight_on");
    return ESP_ERR_TIMEOUT;
  }

  handle->i2c_backpack_state |= PCF8574_BIT_BACKLIGHT;
  ret = priv_hd44780_i2c_write_backpack_nolock(handle, handle->i2c_backpack_state);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to turn backlight ON: %s", esp_err_to_name(ret));
  } else {
    ESP_LOGD(TAG, "Backlight turned ON");
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_hd44780_hal_backlight_off(pstar_hd44780_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  if (handle->mode != HD44780_MODE_I2C) {
    ESP_LOGD(TAG, "Backlight control only available in I2C mode.");
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(HD44780_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for backlight_off");
    return ESP_ERR_TIMEOUT;
  }

  handle->i2c_backpack_state &= ~PCF8574_BIT_BACKLIGHT;
  ret = priv_hd44780_i2c_write_backpack_nolock(handle, handle->i2c_backpack_state);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to turn backlight OFF: %s", esp_err_to_name(ret));
  } else {
    ESP_LOGD(TAG, "Backlight turned OFF");
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

/* --- Convenience Creation Functions --- */

esp_err_t pstar_hd44780_hal_create_kconfig_default(pstar_bus_manager_t*        manager,
                                                   pstar_hd44780_hal_handle_t* out_handle)
{
#if !CONFIG_PSTAR_KCONFIG_HD44780_ENABLED
  ESP_LOGE(TAG, "HD44780 component is disabled in Kconfig.");
  if (out_handle)
    *out_handle = NULL;
  return ESP_ERR_NOT_SUPPORTED;
#else
  ESP_RETURN_ON_FALSE(manager && out_handle,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Manager or out_handle NULL");
  *out_handle   = NULL;
  esp_err_t ret = ESP_OK;

#if CONFIG_PSTAR_KCONFIG_HD44780_MODE_I2C
  ESP_LOGI(TAG, "Creating HD44780 (I2C Mode) with KConfig default configuration");

  /* I2C Mode Specific Setup */
  pstar_bus_config_t* lcd_i2c_config = NULL;
  bool                config_created = false;
  bool                config_added   = false;
  const char*         bus_name       = CONFIG_PSTAR_KCONFIG_HD44780_I2C_BUS_NAME;

#ifdef CONFIG_PSTAR_KCONFIG_HD44780_I2C_PORT_0
  const i2c_port_t port = I2C_NUM_0;
#else
  const i2c_port_t port = I2C_NUM_1;
#endif

  lcd_i2c_config = pstar_bus_config_create_i2c(bus_name,
                                               port,
                                               CONFIG_PSTAR_KCONFIG_HD44780_I2C_ADDR,
                                               CONFIG_PSTAR_KCONFIG_HD44780_SDA_PIN,
                                               CONFIG_PSTAR_KCONFIG_HD44780_SCL_PIN,
                                               CONFIG_PSTAR_KCONFIG_HD44780_I2C_FREQ_HZ);
  ESP_GOTO_ON_FALSE(lcd_i2c_config != NULL,
                    ESP_ERR_NO_MEM,
                    cleanup_i2c,
                    TAG,
                    "Failed to create LCD I2C config");
  config_created = true;

  ret = pstar_bus_manager_add_bus(manager, lcd_i2c_config);
  if (ret == ESP_ERR_INVALID_STATE) { /* Already exists */
    ESP_LOGW(TAG, "Bus config '%s' already exists, finding...", bus_name);
    lcd_i2c_config = pstar_bus_manager_find_bus(manager, bus_name);
    ESP_GOTO_ON_FALSE(lcd_i2c_config,
                      ESP_FAIL,
                      cleanup_i2c,
                      TAG,
                      "Failed to find existing bus '%s'",
                      bus_name);
    config_added   = true;
    config_created = false;
    ret            = ESP_OK;
  } else {
    ESP_GOTO_ON_ERROR(ret,
                      cleanup_i2c,
                      TAG,
                      "Failed to add LCD config to manager: %s",
                      esp_err_to_name(ret));
    config_added   = true;
    config_created = false;
  }

  if (!lcd_i2c_config->initialized) {
    // *** CORRECTED LINE ***
    ret = pstar_bus_config_init(lcd_i2c_config, manager);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup_i2c,
                      TAG,
                      "Failed to initialize LCD bus '%s': %s",
                      bus_name,
                      esp_err_to_name(ret));
  }

  /* Create HAL config struct for I2C mode */
  pstar_hd44780_hal_config_t hal_config = {
    .mode             = HD44780_MODE_I2C,
    .rows             = CONFIG_PSTAR_KCONFIG_HD44780_ROWS,
    .cols             = CONFIG_PSTAR_KCONFIG_HD44780_COLS,
    .i2c_bus_name     = bus_name,
    .i2c_backlight_on = true, /* Default backlight on */
  };
  ret = pstar_hd44780_hal_init(manager, &hal_config, out_handle);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup_i2c,
                    TAG,
                    "Failed to init HD44780 HAL (I2C): %s",
                    esp_err_to_name(ret));

  ESP_LOGI(TAG, "HD44780 (I2C) created and initialized successfully");
  return ESP_OK;

cleanup_i2c:
  if (config_created && !config_added) {
    pstar_bus_config_destroy(lcd_i2c_config);
  }
  *out_handle = NULL;
  return ret;

#else  /* Parallel Mode */
  ESP_LOGI(TAG, "Creating HD44780 (Parallel Mode) with KConfig default configuration");
  pstar_hd44780_hal_config_t hal_config = {
    .mode   = HD44780_MODE_PARALLEL,
    .rows   = CONFIG_PSTAR_KCONFIG_HD44780_ROWS,
    .cols   = CONFIG_PSTAR_KCONFIG_HD44780_COLS,
    .rs_pin = CONFIG_PSTAR_KCONFIG_HD44780_RS_PIN,
    .e_pin  = CONFIG_PSTAR_KCONFIG_HD44780_E_PIN,
    .d4_pin = CONFIG_PSTAR_KCONFIG_HD44780_D4_PIN,
    .d5_pin = CONFIG_PSTAR_KCONFIG_HD44780_D5_PIN,
    .d6_pin = CONFIG_PSTAR_KCONFIG_HD44780_D6_PIN,
    .d7_pin = CONFIG_PSTAR_KCONFIG_HD44780_D7_PIN,
  };
  ret = pstar_hd44780_hal_init(manager,
                               &hal_config,
                               out_handle); /* Manager unused here but required by signature */
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init HD44780 HAL (Parallel): %s", esp_err_to_name(ret));
    *out_handle = NULL;
    return ret;
  }
  ESP_LOGI(TAG, "HD44780 (Parallel) created and initialized successfully");
  return ESP_OK;
#endif /* CONFIG_PSTAR_KCONFIG_HD44780_MODE_I2C */

#endif /* CONFIG_PSTAR_KCONFIG_HD44780_ENABLED */
}

esp_err_t pstar_hd44780_hal_create_custom_parallel(const pstar_bus_manager_t*  manager,
                                                   gpio_num_t                  rs_pin,
                                                   gpio_num_t                  e_pin,
                                                   gpio_num_t                  d4_pin,
                                                   gpio_num_t                  d5_pin,
                                                   gpio_num_t                  d6_pin,
                                                   gpio_num_t                  d7_pin,
                                                   uint8_t                     rows,
                                                   uint8_t                     cols,
                                                   pstar_hd44780_hal_handle_t* out_handle)
{
  ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_INVALID_ARG, TAG, "Output handle is NULL");
  ESP_LOGI(TAG, "Creating HD44780 with custom parallel configuration");

  pstar_hd44780_hal_config_t config = {
    .mode   = HD44780_MODE_PARALLEL,
    .rows   = rows,
    .cols   = cols,
    .rs_pin = rs_pin,
    .e_pin  = e_pin,
    .d4_pin = d4_pin,
    .d5_pin = d5_pin,
    .d6_pin = d6_pin,
    .d7_pin = d7_pin,
    /* I2C fields are ignored in parallel mode */
    .i2c_bus_name     = NULL,
    .i2c_backlight_on = false,
  };

  /* Pass manager, although it won't be used by init in parallel mode */
  return pstar_hd44780_hal_init(manager, &config, out_handle);
}

esp_err_t pstar_hd44780_hal_create_custom_i2c(pstar_bus_manager_t*        manager,
                                              const char*                 i2c_bus_name,
                                              i2c_port_t                  port,
                                              uint8_t                     addr,
                                              gpio_num_t                  sda_pin,
                                              gpio_num_t                  scl_pin,
                                              uint32_t                    i2c_freq_hz,
                                              uint8_t                     rows,
                                              uint8_t                     cols,
                                              bool                        initial_backlight_on,
                                              pstar_hd44780_hal_handle_t* out_handle)
{
  ESP_RETURN_ON_FALSE(manager && i2c_bus_name && out_handle,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid arguments for custom I2C");
  *out_handle                        = NULL;
  esp_err_t           ret            = ESP_OK;
  pstar_bus_config_t* lcd_i2c_config = NULL;
  bool                config_created = false;
  bool                config_added   = false;

  ESP_LOGI(TAG, "Creating HD44780 with custom I2C configuration for bus '%s'", i2c_bus_name);

  /* 1. Create I2C Bus Config */
  lcd_i2c_config =
    pstar_bus_config_create_i2c(i2c_bus_name, port, addr, sda_pin, scl_pin, i2c_freq_hz);
  ESP_GOTO_ON_FALSE(lcd_i2c_config != NULL,
                    ESP_ERR_NO_MEM,
                    cleanup,
                    TAG,
                    "Failed create custom LCD I2C config");
  config_created = true;

  /* 2. Add Config to Manager */
  ret = pstar_bus_manager_add_bus(manager, lcd_i2c_config);
  if (ret == ESP_ERR_INVALID_STATE) { /* Already exists */
    ESP_LOGW(TAG, "Custom bus config '%s' already exists, finding...", i2c_bus_name);
    lcd_i2c_config = pstar_bus_manager_find_bus(manager, i2c_bus_name);
    ESP_GOTO_ON_FALSE(lcd_i2c_config,
                      ESP_FAIL,
                      cleanup,
                      TAG,
                      "Failed find existing custom bus '%s'",
                      i2c_bus_name);
    config_added   = true;
    config_created = false;
    ret            = ESP_OK;
  } else {
    ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed add custom LCD config: %s", esp_err_to_name(ret));
    config_added   = true;
    config_created = false;
  }

  /* 3. Initialize Bus Hardware */
  if (!lcd_i2c_config->initialized) {
    // *** CORRECTED LINE ***
    ret = pstar_bus_config_init(lcd_i2c_config, manager);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed init custom LCD bus '%s': %s",
                      i2c_bus_name,
                      esp_err_to_name(ret));
  }

  /* 4. Initialize HAL */
  pstar_hd44780_hal_config_t hal_config = {
    .mode             = HD44780_MODE_I2C,
    .rows             = rows,
    .cols             = cols,
    .i2c_bus_name     = i2c_bus_name,
    .i2c_backlight_on = initial_backlight_on,
  };
  ret = pstar_hd44780_hal_init(manager, &hal_config, out_handle);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed init custom HD44780 HAL (I2C): %s",
                    esp_err_to_name(ret));

  ESP_LOGI(TAG, "HD44780 (I2C Custom) created and initialized successfully");
  return ESP_OK;

cleanup:
  if (config_created && !config_added) {
    pstar_bus_config_destroy(lcd_i2c_config);
  }
  *out_handle = NULL;
  return ret;
}

/* --- Pin Registration Functions --- */

esp_err_t pstar_hd44780_register_kconfig_pins(void)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_HD44780_ENABLED
  esp_err_t ret = ESP_OK;

#if CONFIG_PSTAR_KCONFIG_HD44780_MODE_I2C
  ESP_LOGI(TAG, "Registering HD44780 I2C pins with validator (from KConfig)...");
  ret = pstar_hd44780_register_custom_i2c_pins(CONFIG_PSTAR_KCONFIG_HD44780_SDA_PIN,
                                               CONFIG_PSTAR_KCONFIG_HD44780_SCL_PIN);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register HD44780 KConfig I2C pins");
#else /* Parallel Mode */
  ESP_LOGI(TAG, "Registering HD44780 Parallel pins with validator (from KConfig)...");
  ret = pstar_hd44780_register_custom_parallel_pins(CONFIG_PSTAR_KCONFIG_HD44780_RS_PIN,
                                                    CONFIG_PSTAR_KCONFIG_HD44780_E_PIN,
                                                    CONFIG_PSTAR_KCONFIG_HD44780_D4_PIN,
                                                    CONFIG_PSTAR_KCONFIG_HD44780_D5_PIN,
                                                    CONFIG_PSTAR_KCONFIG_HD44780_D6_PIN,
                                                    CONFIG_PSTAR_KCONFIG_HD44780_D7_PIN);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register HD44780 KConfig Parallel pins");
#endif

  ESP_LOGI(TAG, "HD44780 KConfig pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator or HD44780 disabled, skipping HD44780 KConfig pin registration.");
  return ESP_OK;
#endif
}

esp_err_t pstar_hd44780_register_custom_parallel_pins(int rs_pin,
                                                      int e_pin,
                                                      int d4_pin,
                                                      int d5_pin,
                                                      int d6_pin,
                                                      int d7_pin)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering HD44780 custom parallel pins with validator...");

  ret = pstar_register_pin(rs_pin, "HD44780 Custom RS", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom RS pin (%d)", rs_pin);
  ret = pstar_register_pin(e_pin, "HD44780 Custom E", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom E pin (%d)", e_pin);
  ret = pstar_register_pin(d4_pin, "HD44780 Custom D4", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom D4 pin (%d)", d4_pin);
  ret = pstar_register_pin(d5_pin, "HD44780 Custom D5", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom D5 pin (%d)", d5_pin);
  ret = pstar_register_pin(d6_pin, "HD44780 Custom D6", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom D6 pin (%d)", d6_pin);
  ret = pstar_register_pin(d7_pin, "HD44780 Custom D7", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom D7 pin (%d)", d7_pin);

  ESP_LOGI(TAG, "HD44780 custom parallel pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator disabled, skipping HD44780 custom parallel pin registration.");
  return ESP_OK;
#endif
}

esp_err_t pstar_hd44780_register_custom_i2c_pins(int sda_pin, int scl_pin)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering HD44780 custom I2C pins with validator...");

  ret = pstar_register_pin(sda_pin, "HD44780 Custom I2C SDA", true); /* I2C pins shareable */
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom SDA pin (%d)", sda_pin);
  ret = pstar_register_pin(scl_pin, "HD44780 Custom I2C SCL", true); /* I2C pins shareable */
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom SCL pin (%d)", scl_pin);

  ESP_LOGI(TAG, "HD44780 custom I2C pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator disabled, skipping HD44780 custom I2C pin registration.");
  return ESP_OK;
#endif
}