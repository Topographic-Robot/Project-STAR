/* components/pstar_hd44780_hal/pstar_hd44780_hal.c */

#include "pstar_hd44780_hal.h"

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

/* HD44780 Commands */
#define LCD_CMD_CLEAR_DISPLAY 0x01
#define LCD_CMD_RETURN_HOME 0x02
#define LCD_CMD_ENTRY_MODE_SET 0x04
#define LCD_CMD_DISPLAY_CONTROL 0x08
#define LCD_CMD_CURSOR_SHIFT 0x10
#define LCD_CMD_FUNCTION_SET 0x20
#define LCD_CMD_SET_CGRAM_ADDR 0x40
#define LCD_CMD_SET_DDRAM_ADDR 0x80

/* Entry Mode Set Flags */
#define LCD_ENTRY_RIGHT 0x00
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_INCREMENT 0x01
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

/* Display Control Flags */
#define LCD_DISPLAY_ON 0x04
#define LCD_DISPLAY_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_ON 0x01
#define LCD_BLINK_OFF 0x00

/* Function Set Flags */
#define LCD_8BIT_MODE 0x10
#define LCD_4BIT_MODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

/* Timing Constants (microseconds) */
#define LCD_DELAY_ENABLE_PULSE 4 /* Min 1us, use slightly more */
#define LCD_DELAY_COMMAND 40     /* Most commands take ~37us, Clear/Home take longer */
#define LCD_DELAY_CLEAR 2000     /* Clear/Home command needs ~1.52ms */
#define LCD_DELAY_INIT 5000      /* Delay after power on and during init sequence */

#define HD44780_MUTEX_TIMEOUT_MS 1000 /* Timeout for acquiring HAL mutex */
#define HD44780_PRINTF_BUFFER_SIZE 64 /* Buffer size for printf */

/* --- Internal Handle Structure --- */
typedef struct pstar_hd44780_hal_dev_t {
  gpio_num_t        rs_pin;
  gpio_num_t        e_pin;
  gpio_num_t        d4_pin;
  gpio_num_t        d5_pin;
  gpio_num_t        d6_pin;
  gpio_num_t        d7_pin;
  uint8_t           rows;
  uint8_t           cols;
  uint8_t           display_ctrl_flags;
  uint8_t           entry_mode_flags;
  SemaphoreHandle_t mutex;
} pstar_hd44780_hal_dev_t;

/* --- Helper Functions --- */

/**
 * @brief Pulse the Enable (E) pin to latch data.
 * Assumes mutex is already taken.
 */
static void priv_hd44780_pulse_enable_nolock(pstar_hd44780_hal_handle_t handle)
{
  gpio_set_level(handle->e_pin, 0);
  ets_delay_us(LCD_DELAY_ENABLE_PULSE);
  gpio_set_level(handle->e_pin, 1);
  ets_delay_us(LCD_DELAY_ENABLE_PULSE);
  gpio_set_level(handle->e_pin, 0);
  ets_delay_us(LCD_DELAY_COMMAND); /* Wait for command execution */
}

/**
 * @brief Send 4 bits of data/command to the LCD data lines (D4-D7).
 * Assumes mutex is already taken.
 */
static void priv_hd44780_write_4bits_nolock(pstar_hd44780_hal_handle_t handle, uint8_t value)
{
  gpio_set_level(handle->d4_pin, (value >> 0) & 0x01);
  gpio_set_level(handle->d5_pin, (value >> 1) & 0x01);
  gpio_set_level(handle->d6_pin, (value >> 2) & 0x01);
  gpio_set_level(handle->d7_pin, (value >> 3) & 0x01);
  priv_hd44780_pulse_enable_nolock(handle);
}

/**
 * @brief Send a full byte (command or data) in 4-bit mode.
 * Assumes mutex is already taken.
 * @param rs_level 0 for command, 1 for data.
 */
static void
priv_hd44780_send_byte_nolock(pstar_hd44780_hal_handle_t handle, uint8_t value, int rs_level)
{
  gpio_set_level(handle->rs_pin, rs_level);
  ets_delay_us(1); /* Short delay after setting RS */

  /* Send high nibble */
  priv_hd44780_write_4bits_nolock(handle, value >> 4);
  /* Send low nibble */
  priv_hd44780_write_4bits_nolock(handle, value & 0x0F);
}

/**
 * @brief Send a command byte (internal, assumes mutex taken).
 */
static inline void priv_hd44780_send_command_nolock(pstar_hd44780_hal_handle_t handle,
                                                    uint8_t                    command)
{
  priv_hd44780_send_byte_nolock(handle, command, 0); /* RS=0 for command */
}

/**
 * @brief Send a data byte (internal, assumes mutex taken).
 */
static inline void priv_hd44780_send_data_nolock(pstar_hd44780_hal_handle_t handle, uint8_t data)
{
  priv_hd44780_send_byte_nolock(handle, data, 1); /* RS=1 for data */
}

/* --- Public Function Implementations --- */

esp_err_t pstar_hd44780_hal_init(const pstar_hd44780_hal_config_t* config,
                                 pstar_hd44780_hal_handle_t*       out_handle)
{
  ESP_RETURN_ON_FALSE(config && out_handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
  ESP_RETURN_ON_FALSE(
    GPIO_IS_VALID_OUTPUT_GPIO(config->rs_pin) && GPIO_IS_VALID_OUTPUT_GPIO(config->e_pin) &&
      GPIO_IS_VALID_OUTPUT_GPIO(config->d4_pin) && GPIO_IS_VALID_OUTPUT_GPIO(config->d5_pin) &&
      GPIO_IS_VALID_OUTPUT_GPIO(config->d6_pin) && GPIO_IS_VALID_OUTPUT_GPIO(config->d7_pin),
    ESP_ERR_INVALID_ARG,
    TAG,
    "One or more GPIO pins are invalid");
  ESP_RETURN_ON_FALSE(config->rows > 0 && config->cols > 0,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Rows and cols must be > 0");
  *out_handle = NULL;

  /* Allocate memory for the device handle */
  pstar_hd44780_hal_handle_t dev =
    (pstar_hd44780_hal_handle_t)malloc(sizeof(pstar_hd44780_hal_dev_t));
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Failed to allocate device handle");

  dev->rs_pin = config->rs_pin;
  dev->e_pin  = config->e_pin;
  dev->d4_pin = config->d4_pin;
  dev->d5_pin = config->d5_pin;
  dev->d6_pin = config->d6_pin;
  dev->d7_pin = config->d7_pin;
  dev->rows   = config->rows;
  dev->cols   = config->cols;
  dev->mutex  = NULL; /* Initialize before creation */

  /* Create mutex */
  dev->mutex = xSemaphoreCreateMutex();
  if (!dev->mutex) {
    ESP_LOGE(TAG, "Failed to create mutex for HD44780 handle");
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

  /* Configure GPIO pins */
  gpio_config_t io_conf;
  io_conf.intr_type    = GPIO_INTR_DISABLE;
  io_conf.mode         = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << dev->rs_pin) | (1ULL << dev->e_pin) | (1ULL << dev->d4_pin) |
                         (1ULL << dev->d5_pin) | (1ULL << dev->d6_pin) | (1ULL << dev->d7_pin);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
  esp_err_t ret        = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure GPIO pins: %s", esp_err_to_name(ret));
    goto init_fail_mutex;
  }

  /* --- HD44780 Initialization Sequence (4-bit mode) --- */
  ESP_LOGI(TAG, "Initializing HD44780 (%dx%d) in 4-bit mode...", dev->cols, dev->rows);

  /* 1. Wait >15ms after VCC rises to 4.5V */
  vTaskDelay(pdMS_TO_TICKS(50)); /* Wait 50ms to be safe */

  /* 2. Special Function Set sequence */
  gpio_set_level(dev->rs_pin, 0);
  gpio_set_level(dev->e_pin, 0);
  priv_hd44780_write_4bits_nolock(dev, 0x03); /* Send 0x3 */
  vTaskDelay(pdMS_TO_TICKS(5));               /* Wait >4.1ms */
  priv_hd44780_write_4bits_nolock(dev, 0x03); /* Send 0x3 */
  ets_delay_us(150);                          /* Wait >100us */
  priv_hd44780_write_4bits_nolock(dev, 0x03); /* Send 0x3 */
  ets_delay_us(LCD_DELAY_COMMAND);

  /* 3. Set 4-bit mode */
  priv_hd44780_write_4bits_nolock(dev, 0x02); /* Send 0x2 */
  ets_delay_us(LCD_DELAY_COMMAND);

  /* 4. Function Set: 4-bit, N lines (N>1 ? 2 lines : 1 line), 5x8 dots */
  uint8_t func_set = LCD_4BIT_MODE | LCD_5x8DOTS;
  if (dev->rows > 1) {
    func_set |= LCD_2LINE;
  } else {
    func_set |= LCD_1LINE;
  }
  priv_hd44780_send_command_nolock(dev, LCD_CMD_FUNCTION_SET | func_set);

  /* 5. Display Control: Display ON, Cursor OFF, Blink OFF */
  dev->display_ctrl_flags = LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF;
  priv_hd44780_send_command_nolock(dev, LCD_CMD_DISPLAY_CONTROL | dev->display_ctrl_flags);

  /* 6. Clear Display */
  priv_hd44780_send_command_nolock(dev, LCD_CMD_CLEAR_DISPLAY);
  vTaskDelay(pdMS_TO_TICKS(3)); /* Clear needs extra delay */

  /* 7. Entry Mode Set: Increment cursor, no shift */
  dev->entry_mode_flags = LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT;
  priv_hd44780_send_command_nolock(dev, LCD_CMD_ENTRY_MODE_SET | dev->entry_mode_flags);

  /* Initialization complete */
  ESP_LOGI(TAG, "HD44780 Initialized");
  xSemaphoreGive(dev->mutex);
  *out_handle = dev;
  return ESP_OK;

init_fail_mutex:
  xSemaphoreGive(dev->mutex);
  vSemaphoreDelete(dev->mutex);
  free(dev);
  *out_handle = NULL;
  return ret;
}

esp_err_t pstar_hd44780_hal_deinit(pstar_hd44780_hal_handle_t handle, bool clear_display)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGI(TAG, "Deinitializing HD44780 HAL...");

  if (clear_display) {
    esp_err_t clear_ret = pstar_hd44780_hal_clear(handle);
    if (clear_ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to clear display during deinit: %s", esp_err_to_name(clear_ret));
    }
  }

  /* Reset GPIO pins */
  gpio_reset_pin(handle->rs_pin);
  gpio_reset_pin(handle->e_pin);
  gpio_reset_pin(handle->d4_pin);
  gpio_reset_pin(handle->d5_pin);
  gpio_reset_pin(handle->d6_pin);
  gpio_reset_pin(handle->d7_pin);

  /* Delete mutex */
  if (handle->mutex) {
    vSemaphoreDelete(handle->mutex);
    handle->mutex = NULL;
  }

  free(handle);
  return ESP_OK;
}

esp_err_t pstar_hd44780_hal_send_command(pstar_hd44780_hal_handle_t handle, uint8_t command)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(HD44780_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for send_command");
    return ESP_ERR_TIMEOUT;
  }

  priv_hd44780_send_command_nolock(handle, command);

  /* Add extra delay for clear/home commands */
  if (command == LCD_CMD_CLEAR_DISPLAY || command == LCD_CMD_RETURN_HOME) {
    ets_delay_us(LCD_DELAY_CLEAR);
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_hd44780_hal_send_data(pstar_hd44780_hal_handle_t handle, uint8_t data)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(HD44780_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for send_data");
    return ESP_ERR_TIMEOUT;
  }

  priv_hd44780_send_data_nolock(handle, data);

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_hd44780_hal_clear(pstar_hd44780_hal_handle_t handle)
{
  return pstar_hd44780_hal_send_command(handle, LCD_CMD_CLEAR_DISPLAY);
}

esp_err_t pstar_hd44780_hal_home(pstar_hd44780_hal_handle_t handle)
{
  return pstar_hd44780_hal_send_command(handle, LCD_CMD_RETURN_HOME);
}

esp_err_t pstar_hd44780_hal_set_cursor(pstar_hd44780_hal_handle_t handle, uint8_t col, uint8_t row)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");
  ESP_RETURN_ON_FALSE(row < handle->rows && col < handle->cols,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Cursor position (%d, %d) out of bounds (%dx%d)",
                      col,
                      row,
                      handle->cols,
                      handle->rows);

  esp_err_t ret = ESP_OK;

  /* DDRAM addresses for common display sizes */
  /* Adjust these offsets if using displays with unusual memory mapping */
  static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54}; /* Offsets for 4 rows */
  uint8_t              ddram_addr;

  if (row >= sizeof(row_offsets) / sizeof(row_offsets[0])) {
    ESP_LOGE(TAG,
             "Row %d exceeds known offsets (max %zu)",
             row,
             sizeof(row_offsets) / sizeof(row_offsets[0]) - 1);
    return ESP_ERR_INVALID_ARG;
  }

  ddram_addr = row_offsets[row] + col;

  ret = pstar_hd44780_hal_send_command(handle, LCD_CMD_SET_DDRAM_ADDR | ddram_addr);

  return ret;
}

esp_err_t pstar_hd44780_hal_print_string(pstar_hd44780_hal_handle_t handle, const char* str)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(str, ESP_ERR_INVALID_ARG, TAG, "String pointer is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(HD44780_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for print_string");
    return ESP_ERR_TIMEOUT;
  }

  while (*str) {
    priv_hd44780_send_data_nolock(handle, *str++);
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_hd44780_hal_printf(pstar_hd44780_hal_handle_t handle, const char* format, ...)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(format, ESP_ERR_INVALID_ARG, TAG, "Format string is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  char      buffer[HD44780_PRINTF_BUFFER_SIZE];
  va_list   args;
  esp_err_t ret = ESP_OK;

  va_start(args, format);
  int len = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (len < 0) {
    ESP_LOGE(TAG, "printf formatting error");
    return ESP_FAIL; /* Or a more specific error */
  }
  if (len >= sizeof(buffer)) {
    ESP_LOGW(TAG, "printf output truncated (%d vs %d)", len, sizeof(buffer) - 1);
    /* String is null-terminated by vsnprintf even on truncation */
  }

  /* Use print_string which handles mutex */
  ret = pstar_hd44780_hal_print_string(handle, buffer);

  return ret;
}

/* --- Convenience Creation Functions --- */

esp_err_t pstar_hd44780_hal_create_kconfig_default(pstar_hd44780_hal_handle_t* out_handle)
{
#if !CONFIG_PSTAR_KCONFIG_HD44780_ENABLED
  ESP_LOGE(TAG, "HD44780 component is disabled in Kconfig.");
  if (out_handle)
    *out_handle = NULL;
  return ESP_ERR_NOT_SUPPORTED;
#else
  ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_INVALID_ARG, TAG, "Output handle pointer is NULL");
  ESP_LOGI(TAG, "Creating HD44780 with KConfig default configuration");

  pstar_hd44780_hal_config_t config = {
    .rs_pin = CONFIG_PSTAR_KCONFIG_HD44780_RS_PIN,
    .e_pin  = CONFIG_PSTAR_KCONFIG_HD44780_E_PIN,
    .d4_pin = CONFIG_PSTAR_KCONFIG_HD44780_D4_PIN,
    .d5_pin = CONFIG_PSTAR_KCONFIG_HD44780_D5_PIN,
    .d6_pin = CONFIG_PSTAR_KCONFIG_HD44780_D6_PIN,
    .d7_pin = CONFIG_PSTAR_KCONFIG_HD44780_D7_PIN,
    .rows   = CONFIG_PSTAR_KCONFIG_HD44780_ROWS,
    .cols   = CONFIG_PSTAR_KCONFIG_HD44780_COLS,
  };

  return pstar_hd44780_hal_init(&config, out_handle);
#endif /* CONFIG_PSTAR_KCONFIG_HD44780_ENABLED */
}

esp_err_t pstar_hd44780_hal_create_custom(gpio_num_t                  rs_pin,
                                          gpio_num_t                  e_pin,
                                          gpio_num_t                  d4_pin,
                                          gpio_num_t                  d5_pin,
                                          gpio_num_t                  d6_pin,
                                          gpio_num_t                  d7_pin,
                                          uint8_t                     rows,
                                          uint8_t                     cols,
                                          pstar_hd44780_hal_handle_t* out_handle)
{
  ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_INVALID_ARG, TAG, "Output handle pointer is NULL");
  ESP_LOGI(TAG, "Creating HD44780 with custom configuration");

  pstar_hd44780_hal_config_t config = {
    .rs_pin = rs_pin,
    .e_pin  = e_pin,
    .d4_pin = d4_pin,
    .d5_pin = d5_pin,
    .d6_pin = d6_pin,
    .d7_pin = d7_pin,
    .rows   = rows,
    .cols   = cols,
  };

  return pstar_hd44780_hal_init(&config, out_handle);
}

/* --- Pin Registration Functions --- */

esp_err_t pstar_hd44780_register_kconfig_pins(void)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_HD44780_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering HD44780 pins with validator (from KConfig)...");

  /* Register RS pin */
  ret = pstar_register_pin(CONFIG_PSTAR_KCONFIG_HD44780_RS_PIN, "HD44780 RS", false);
  ESP_RETURN_ON_ERROR(ret,
                      TAG,
                      "Failed to register HD44780 RS pin (%d)",
                      CONFIG_PSTAR_KCONFIG_HD44780_RS_PIN);

  /* Register E pin */
  ret = pstar_register_pin(CONFIG_PSTAR_KCONFIG_HD44780_E_PIN, "HD44780 E", false);
  ESP_RETURN_ON_ERROR(ret,
                      TAG,
                      "Failed to register HD44780 E pin (%d)",
                      CONFIG_PSTAR_KCONFIG_HD44780_E_PIN);

  /* Register D4 pin */
  ret = pstar_register_pin(CONFIG_PSTAR_KCONFIG_HD44780_D4_PIN, "HD44780 D4", false);
  ESP_RETURN_ON_ERROR(ret,
                      TAG,
                      "Failed to register HD44780 D4 pin (%d)",
                      CONFIG_PSTAR_KCONFIG_HD44780_D4_PIN);

  /* Register D5 pin */
  ret = pstar_register_pin(CONFIG_PSTAR_KCONFIG_HD44780_D5_PIN, "HD44780 D5", false);
  ESP_RETURN_ON_ERROR(ret,
                      TAG,
                      "Failed to register HD44780 D5 pin (%d)",
                      CONFIG_PSTAR_KCONFIG_HD44780_D5_PIN);

  /* Register D6 pin */
  ret = pstar_register_pin(CONFIG_PSTAR_KCONFIG_HD44780_D6_PIN, "HD44780 D6", false);
  ESP_RETURN_ON_ERROR(ret,
                      TAG,
                      "Failed to register HD44780 D6 pin (%d)",
                      CONFIG_PSTAR_KCONFIG_HD44780_D6_PIN);

  /* Register D7 pin */
  ret = pstar_register_pin(CONFIG_PSTAR_KCONFIG_HD44780_D7_PIN, "HD44780 D7", false);
  ESP_RETURN_ON_ERROR(ret,
                      TAG,
                      "Failed to register HD44780 D7 pin (%d)",
                      CONFIG_PSTAR_KCONFIG_HD44780_D7_PIN);

  ESP_LOGI(TAG, "HD44780 KConfig pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator or HD44780 disabled, skipping HD44780 KConfig pin registration.");
  return ESP_OK; /* Not an error if validator or component is disabled */
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_HD44780_ENABLED */
}

esp_err_t pstar_hd44780_register_custom_pins(int rs_pin,
                                             int e_pin,
                                             int d4_pin,
                                             int d5_pin,
                                             int d6_pin,
                                             int d7_pin)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering HD44780 custom pins with validator...");

  /* Register RS pin */
  ret = pstar_register_pin(rs_pin, "HD44780 Custom RS", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register HD44780 custom RS pin (%d)", rs_pin);

  /* Register E pin */
  ret = pstar_register_pin(e_pin, "HD44780 Custom E", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register HD44780 custom E pin (%d)", e_pin);

  /* Register D4 pin */
  ret = pstar_register_pin(d4_pin, "HD44780 Custom D4", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register HD44780 custom D4 pin (%d)", d4_pin);

  /* Register D5 pin */
  ret = pstar_register_pin(d5_pin, "HD44780 Custom D5", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register HD44780 custom D5 pin (%d)", d5_pin);

  /* Register D6 pin */
  ret = pstar_register_pin(d6_pin, "HD44780 Custom D6", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register HD44780 custom D6 pin (%d)", d6_pin);

  /* Register D7 pin */
  ret = pstar_register_pin(d7_pin, "HD44780 Custom D7", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register HD44780 custom D7 pin (%d)", d7_pin);

  ESP_LOGI(TAG, "HD44780 custom pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator disabled, skipping HD44780 custom pin registration.");
  return ESP_OK; /* Not an error if validator is disabled */
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED */
}