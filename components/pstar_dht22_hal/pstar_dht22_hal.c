/* components/pstar_dht22_hal/pstar_dht22_hal.c */

#include "pstar_dht22_hal.h"

#include "pstar_pin_validator.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h" /* For high-resolution timing if needed */
#include "rom/ets_sys.h"

static const char* TAG = "DHT22 HAL";

/* DHT22 Timing Constants (microseconds) */
#define DHT22_START_SIGNAL_LOW_US 1100 /* Min 1ms, typ 1.1ms */
#define DHT22_START_SIGNAL_HIGH_US 30  /* 20-40us */
#define DHT22_RESPONSE_WAIT_US 100     /* Max wait for sensor response */
#define DHT22_RESPONSE_LOW_US 90       /* Typ 80us */
#define DHT22_RESPONSE_HIGH_US 90      /* Typ 80us */
#define DHT22_DATA_START_LOW_US 60     /* Typ 50us */
#define DHT22_DATA_BIT_LOW_TIME_US 28  /* Typ 26-28us */
#define DHT22_DATA_BIT_HIGH_TIME_US 75 /* Typ 70us */
#define DHT22_DATA_TIMEOUT_US 100      /* Max wait for a bit transition */
#define DHT22_POST_DATA_DELAY_MS 2000  /* Min delay between reads is 2s */
#define DHT22_MAX_READ_ATTEMPTS 3      /* Number of attempts on timeout/CRC error */
#define DHT22_RETRY_DELAY_MS 50        /* Short delay between retries */

/* --- Internal Handle Structure --- */
typedef struct pstar_dht22_hal_dev_t {
  gpio_num_t gpio_pin;       /**< GPIO pin used for communication */
  int64_t    last_read_time; /**< Timestamp of the last successful read */
} pstar_dht22_hal_dev_t;

/* --- Helper Functions --- */

/**
 * @brief Waits for the GPIO pin to reach a specific level or timeout.
 * @param pin GPIO number.
 * @param level Target level (0 or 1).
 * @param timeout_us Timeout duration in microseconds.
 * @return Time spent waiting in microseconds, or -1 on timeout.
 */
static int priv_dht22_wait_for_level(gpio_num_t pin, int level, uint32_t timeout_us)
{
  int64_t start_time = esp_timer_get_time();
  while (gpio_get_level(pin) != level) {
    if ((esp_timer_get_time() - start_time) > timeout_us) {
      return -1; /* Timeout */
    }
    /* Small delay to prevent busy-waiting too aggressively */
    /* ets_delay_us(1); // Might be too short, esp_timer is better */
  }
  return (int)(esp_timer_get_time() - start_time);
}

/**
 * @brief Reads a single byte (8 bits) from the DHT22 data line.
 * @param handle DHT22 HAL handle.
 * @param[out] byte Pointer to store the read byte.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on bit read timeout.
 */
static esp_err_t priv_dht22_read_byte(pstar_dht22_hal_handle_t handle, uint8_t* byte)
{
  *byte = 0;
  for (int i = 0; i < 8; ++i) {
    /* Wait for the 50us low period before the bit */
    if (priv_dht22_wait_for_level(handle->gpio_pin, 1, DHT22_DATA_TIMEOUT_US) < 0) {
      ESP_LOGD(TAG, "Timeout waiting for data bit %d start high", 7 - i);
      return ESP_ERR_TIMEOUT;
    }

    /* Wait for the high period of the bit */
    int high_duration_us = priv_dht22_wait_for_level(handle->gpio_pin, 0, DHT22_DATA_TIMEOUT_US);
    if (high_duration_us < 0) {
      ESP_LOGD(TAG, "Timeout waiting for data bit %d end low", 7 - i);
      return ESP_ERR_TIMEOUT;
    }

    /* Shift previous bits left */
    *byte <<= 1;

    /* Determine bit value based on high duration */
    if (high_duration_us > DHT22_DATA_BIT_LOW_TIME_US) {
      *byte |= 1; /* It's a '1' bit */
    }
    /* Else it's a '0' bit, no need to change *byte */
  }
  return ESP_OK;
}

/* --- Public Function Implementations --- */

esp_err_t pstar_dht22_hal_init(const pstar_dht22_hal_config_t* config,
                               pstar_dht22_hal_handle_t*       out_handle)
{
  ESP_RETURN_ON_FALSE(config && out_handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
  ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(config->gpio_pin),
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid GPIO pin: %d",
                      config->gpio_pin);
  *out_handle = NULL;

  /* Allocate memory for the device handle */
  pstar_dht22_hal_handle_t dev = (pstar_dht22_hal_handle_t)malloc(sizeof(pstar_dht22_hal_dev_t));
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Failed to allocate device handle");

  dev->gpio_pin       = config->gpio_pin;
  dev->last_read_time = -DHT22_POST_DATA_DELAY_MS * 1000LL; /* Allow immediate first read */

  /* Configure GPIO pin */
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << dev->gpio_pin),
    .mode         = GPIO_MODE_INPUT_OUTPUT_OD, /* Open-drain for DHT protocol */
    .pull_up_en   = GPIO_PULLUP_ENABLE,        /* External pull-up usually required */
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE,
  };
  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure GPIO pin %d: %s", dev->gpio_pin, esp_err_to_name(ret));
    free(dev);
    return ret;
  }

  /* Set initial state (high impedance input, relying on pull-up) */
  ret = gpio_set_level(dev->gpio_pin, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG,
             "Failed to set initial GPIO level for pin %d: %s",
             dev->gpio_pin,
             esp_err_to_name(ret));
    free(dev);
    return ret;
  }

  ESP_LOGI(TAG, "DHT22 HAL initialized on GPIO %d", dev->gpio_pin);
  *out_handle = dev;
  return ESP_OK;
}

esp_err_t pstar_dht22_hal_deinit(pstar_dht22_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGI(TAG, "Deinitializing DHT22 HAL for GPIO %d", handle->gpio_pin);

  /* Reset GPIO configuration */
  gpio_reset_pin(handle->gpio_pin);

  free(handle);
  return ESP_OK;
}

esp_err_t
pstar_dht22_hal_read_data(pstar_dht22_hal_handle_t handle, float* humidity, float* temperature)
{
  ESP_RETURN_ON_FALSE(handle && humidity && temperature,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Handle or output pointers are NULL");

  *humidity    = NAN; /* Default to Not a Number on error */
  *temperature = NAN;

  esp_err_t ret = ESP_FAIL; /* Default to failure */

  /* Check minimum delay between reads */
  int64_t current_time = esp_timer_get_time();
  if (current_time - handle->last_read_time < (DHT22_POST_DATA_DELAY_MS * 1000LL)) {
    ESP_LOGD(TAG, "Read interval too short. Min %dms required.", DHT22_POST_DATA_DELAY_MS);
    return ESP_ERR_INVALID_STATE; /* Indicate too soon */
  }

  /* --- Start Communication --- */
  /* Disable interrupts during timing-critical section */
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&mux);

  /* 1. Send start signal: Pull low for >1ms, then high for 20-40us */
  gpio_set_direction(handle->gpio_pin, GPIO_MODE_OUTPUT_OD);
  gpio_set_level(handle->gpio_pin, 0);
  ets_delay_us(DHT22_START_SIGNAL_LOW_US);
  gpio_set_level(handle->gpio_pin, 1); /* Go high (via pull-up) */
  ets_delay_us(DHT22_START_SIGNAL_HIGH_US);
  gpio_set_direction(handle->gpio_pin, GPIO_MODE_INPUT); /* Set back to input */

  /* 2. Wait for sensor response: Low for ~80us, then High for ~80us */
  if (priv_dht22_wait_for_level(handle->gpio_pin, 0, DHT22_RESPONSE_WAIT_US) < 0) {
    ESP_LOGD(TAG, "Timeout waiting for sensor response (low)");
    ret = ESP_ERR_TIMEOUT;
    goto read_fail;
  }
  if (priv_dht22_wait_for_level(handle->gpio_pin, 1, DHT22_RESPONSE_LOW_US) < 0) {
    ESP_LOGD(TAG, "Timeout waiting for sensor response (high)");
    ret = ESP_ERR_TIMEOUT;
    goto read_fail;
  }
  if (priv_dht22_wait_for_level(handle->gpio_pin, 0, DHT22_RESPONSE_HIGH_US) < 0) {
    ESP_LOGD(TAG, "Timeout waiting for sensor response end (low)");
    ret = ESP_ERR_TIMEOUT;
    goto read_fail;
  }

  /* 3. Read 40 bits (5 bytes) of data */
  uint8_t data[5] = {0};
  for (int i = 0; i < 5; ++i) {
    ret = priv_dht22_read_byte(handle, &data[i]);
    if (ret != ESP_OK) {
      ESP_LOGD(TAG, "Failed reading byte %d: %s", i, esp_err_to_name(ret));
      goto read_fail;
    }
  }

  /* Re-enable interrupts */
  portEXIT_CRITICAL(&mux);

  /* 4. Verify Checksum */
  uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
  if (checksum != data[4]) {
    ESP_LOGE(TAG, "Checksum error: Calculated=0x%02X, Received=0x%02X", checksum, data[4]);
    ESP_LOGE(TAG,
             "Raw data: %02X %02X %02X %02X %02X",
             data[0],
             data[1],
             data[2],
             data[3],
             data[4]);
    return ESP_ERR_INVALID_CRC;
  }

  /* 5. Decode Data */
  uint16_t raw_humidity    = (data[0] << 8) | data[1];
  uint16_t raw_temperature = (data[2] << 8) | data[3];

  *humidity = (float)raw_humidity / 10.0f;

  /* Handle negative temperatures (MSB of raw_temperature is sign bit) */
  if (raw_temperature & 0x8000) {
    *temperature = -((float)(raw_temperature & 0x7FFF) / 10.0f);
  } else {
    *temperature = (float)raw_temperature / 10.0f;
  }

  /* Clamp humidity to 0-100% */
  if (*humidity > 100.0f) {
    *humidity = 100.0f;
  } else if (*humidity < 0.0f) {
    *humidity = 0.0f;
  }

  handle->last_read_time = esp_timer_get_time(); /* Update last read time on success */
  ESP_LOGD(TAG, "Read success: Temp=%.1fC, Hum=%.1f%%", *temperature, *humidity);
  return ESP_OK;

read_fail:
  /* Re-enable interrupts on failure */
  portEXIT_CRITICAL(&mux);
  ESP_LOGD(TAG, "Read failed: %s", esp_err_to_name(ret));
  return ret;
}

/* --- Convenience Creation Functions --- */

esp_err_t pstar_dht22_hal_create_kconfig_default(pstar_dht22_hal_handle_t* out_handle)
{
#if !CONFIG_PSTAR_KCONFIG_DHT22_ENABLED
  ESP_LOGE(TAG, "DHT22 component is disabled in Kconfig.");
  if (out_handle)
    *out_handle = NULL;
  return ESP_ERR_NOT_SUPPORTED;
#else
  ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_INVALID_ARG, TAG, "Output handle pointer is NULL");
  ESP_LOGI(TAG, "Creating DHT22 with KConfig default configuration");

  pstar_dht22_hal_config_t config = {
    .gpio_pin = CONFIG_PSTAR_KCONFIG_DHT22_GPIO_PIN,
  };

  return pstar_dht22_hal_init(&config, out_handle);
#endif /* CONFIG_PSTAR_KCONFIG_DHT22_ENABLED */
}

esp_err_t pstar_dht22_hal_create_custom(gpio_num_t gpio_pin, pstar_dht22_hal_handle_t* out_handle)
{
  ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_INVALID_ARG, TAG, "Output handle pointer is NULL");
  ESP_LOGI(TAG, "Creating DHT22 with custom configuration on GPIO %d", gpio_pin);

  pstar_dht22_hal_config_t config = {
    .gpio_pin = gpio_pin,
  };

  return pstar_dht22_hal_init(&config, out_handle);
}

/* --- Pin Registration Functions --- */

esp_err_t pstar_dht22_register_kconfig_pins(void)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_DHT22_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering DHT22 pin with validator (from KConfig)...");

  const char* desc = "DHT22 Data Pin";
  /* DHT22 pin is dedicated, cannot be shared */
  ret = pstar_register_pin(CONFIG_PSTAR_KCONFIG_DHT22_GPIO_PIN, desc, false);
  ESP_RETURN_ON_ERROR(ret,
                      TAG,
                      "Failed to register DHT22 pin (%d)",
                      CONFIG_PSTAR_KCONFIG_DHT22_GPIO_PIN);

  ESP_LOGI(TAG, "DHT22 KConfig pin registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator or DHT22 disabled, skipping DHT22 KConfig pin registration.");
  return ESP_OK; /* Not an error if validator or component is disabled */
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_DHT22_ENABLED */
}

esp_err_t pstar_dht22_register_custom_pin(int gpio_pin)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering DHT22 custom pin %d with validator...", gpio_pin);

  const char* desc = "DHT22 Custom Data Pin";
  /* DHT22 pin is dedicated, cannot be shared */
  ret = pstar_register_pin(gpio_pin, desc, false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register DHT22 custom pin (%d)", gpio_pin);

  ESP_LOGI(TAG, "DHT22 custom pin registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator disabled, skipping DHT22 custom pin registration.");
  return ESP_OK; /* Not an error if validator is disabled */
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED */
}