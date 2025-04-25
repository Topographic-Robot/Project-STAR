/* components/pstar_pin_validator/pstar_pin_validator.c */

#include "pstar_pin_validator.h"

#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define TAG ("PSTAR PIN VALIDATOR")

/* --- Global Variables (Static) --- */

static pstar_pin_validator_t s_pin_validator = {0};

/* --- Private Function Declarations --- */

/**
 * @brief Initialize mutex if not already initialized
 *
 * @return ESP_OK if successful, otherwise an error code
 */
static esp_err_t priv_init_mutex(void);

/* --- Functions --- */

static esp_err_t priv_init_mutex(void)
{
/* Only create mutex if validator is enabled */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  if (s_pin_validator.mutex == NULL) {
    s_pin_validator.mutex = xSemaphoreCreateMutex();
    if (s_pin_validator.mutex == NULL) {
      ESP_LOGE(TAG, "Failed to create pin validator mutex");
      return ESP_ERR_NO_MEM;
    }
    ESP_LOGD(TAG, "Pin validator mutex created");
  }
#endif
  return ESP_OK;
}

esp_err_t pstar_register_pin(gpio_num_t gpio_num, const char* desc, bool can_be_shared)
{
/* --- Add Top-Level Guard --- */
#if !CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  // ESP_LOGD(TAG, "Pin Validator disabled. Skipping registration for GPIO %d.", gpio_num);
  return ESP_OK; /* Silently succeed if disabled */
#else
  esp_err_t ret;

  /* validate inputs before taking mutex */
  if (gpio_num >= GPIO_NUM_MAX || gpio_num < 0) {
    ESP_LOGE(TAG, "Invalid GPIO number: %d (max allowed: %d)", gpio_num, GPIO_NUM_MAX - 1);
    return ESP_ERR_INVALID_ARG;
  }

  if (desc == NULL) {
    ESP_LOGE(TAG, "Description cannot be NULL");
    return ESP_ERR_INVALID_ARG;
  }

  if (strlen(desc) >= PIN_VALIDATOR_DESC_MAX_LEN) {
    ESP_LOGE(TAG, "Description too long (max: %d characters)", PIN_VALIDATOR_DESC_MAX_LEN - 1);
    return ESP_ERR_INVALID_ARG;
  }

  /* Initialize mutex if needed */
  ret = priv_init_mutex();
  if (ret != ESP_OK) {
    return ret;
  }

  /* Take mutex */
  if (xSemaphoreTake(s_pin_validator.mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take pin validator mutex");
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGD(TAG, "Registering pin %d, desc: '%s', can_be_shared: %d", gpio_num, desc, can_be_shared);

  s_pin_validator.initialized = true;

  /* get current pin */
  pstar_pin_info_t* pin = &s_pin_validator.pins[gpio_num];

  /* if the pin is already in use check if it can be shared, error otherwise */
  if (pin->usage_count > 0) {
    ESP_LOGD(TAG, "Pin %d already in use (count: %d)", gpio_num, pin->usage_count);
    if (!pin->can_be_shared || !can_be_shared) {
      ESP_LOGE(TAG,
               "Pin %d cannot be shared (current config: %d, requested: %d)",
               gpio_num,
               pin->can_be_shared,
               can_be_shared);
      xSemaphoreGive(s_pin_validator.mutex);
      return ESP_ERR_INVALID_STATE;
    }
  } else {
    /* first time registering */
    ESP_LOGD(TAG, "First registration of pin %d", gpio_num); /* Changed to debug */
    pin->can_be_shared = can_be_shared;
  }

  /* increase the usage count */
  pin->usage_count += 1;

  /* allocate/reallocate memory for user descriptions */
  if (pin->users == NULL) {
    pin->users = (char**)malloc(sizeof(char*));
    if (pin->users == NULL) {
      ESP_LOGE(TAG, "Failed to allocate memory for pin users");
      pin->usage_count -= 1;
      xSemaphoreGive(s_pin_validator.mutex);
      return ESP_ERR_NO_MEM;
    }
  } else {
    char** new_users = (char**)realloc(pin->users, sizeof(char*) * pin->usage_count);
    if (new_users == NULL) {
      ESP_LOGE(TAG, "Failed to reallocate memory for pin users");
      pin->usage_count -= 1;
      xSemaphoreGive(s_pin_validator.mutex);
      return ESP_ERR_NO_MEM;
    }
    pin->users = new_users;
  }

  /* allocate and copy the description */
  pin->users[pin->usage_count - 1] = (char*)malloc(strlen(desc) + 1);
  if (pin->users[pin->usage_count - 1] == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for description");
    pin->usage_count -= 1;
    xSemaphoreGive(s_pin_validator.mutex);
    return ESP_ERR_NO_MEM;
  }

  strcpy(pin->users[pin->usage_count - 1], desc);

  ESP_LOGD(TAG,
           "Pin %d registered successfully (usage count: %d)",
           gpio_num,
           pin->usage_count); /* Changed to debug */

  /* Release mutex */
  xSemaphoreGive(s_pin_validator.mutex);

  return ESP_OK;
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED */
}

esp_err_t pstar_validate_pins(void)
{
/* --- Add Top-Level Guard --- */
#if !CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  // ESP_LOGD(TAG, "Pin Validator disabled. Skipping validation.");
  return ESP_OK; /* Silently succeed if disabled */
#else
  esp_err_t ret;
  bool      conflicts_found = false;

  /* Initialize mutex if needed */
  ret = priv_init_mutex();
  if (ret != ESP_OK) {
    return ret;
  }

  /* Take mutex */
  if (xSemaphoreTake(s_pin_validator.mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take pin validator mutex");
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGI(TAG, "Validating all pin configurations");

  if (!s_pin_validator.initialized) {
    ESP_LOGW(TAG, "Pin validator not initialized, no pins registered");
    xSemaphoreGive(s_pin_validator.mutex);
    return ESP_OK;
  }

  /* Check each pin for conflicts */
  for (int i = 0; i < GPIO_NUM_MAX; i++) {
    pstar_pin_info_t* pin = &s_pin_validator.pins[i];

    if (pin->usage_count > 1 && !pin->can_be_shared) {
      conflicts_found = true;
      ESP_LOGE(TAG, "Conflict on GPIO %d: Used %d times but not shareable", i, pin->usage_count);

      /* Log all users of this conflicting pin */
      for (int j = 0; j < pin->usage_count; j++) {
        ESP_LOGE(TAG, "  User %d: %s", j + 1, pin->users[j]);
      }
    } else if (pin->usage_count > 0) {
      ESP_LOGD(TAG,
               "GPIO %d: Used %d times, shareable: %d",
               i,
               pin->usage_count,
               pin->can_be_shared);
    }
  }

  /* Release mutex */
  xSemaphoreGive(s_pin_validator.mutex);

  if (conflicts_found) {
    ESP_LOGE(TAG, "Pin validation failed: conflicts detected");
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGI(TAG, "Pin validation successful, no conflicts found");
  return ESP_OK;
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED */
}

esp_err_t pstar_free_pin_validator(void)
{
/* --- Add Top-Level Guard --- */
#if !CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  return ESP_OK; /* Silently succeed if disabled */
#else
  esp_err_t ret;

  /* Initialize mutex if needed */
  ret = priv_init_mutex();
  if (ret != ESP_OK) {
    return ret; /* Cannot proceed without mutex initialized */
  }
  /* Check if mutex actually exists before trying to take/delete */
  if (s_pin_validator.mutex == NULL) {
    ESP_LOGW(
      TAG,
      "Attempting to free validator, but mutex was NULL (already freed or never initialized?).");
    s_pin_validator.initialized = false; /* Ensure state is reset */
    return ESP_OK;
  }

  /* Take mutex */
  if (xSemaphoreTake(s_pin_validator.mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to take pin validator mutex for free");
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGI(TAG, "Freeing pin validator resources");

  if (!s_pin_validator.initialized) {
    ESP_LOGW(TAG, "Pin validator not initialized, nothing to free");
    xSemaphoreGive(s_pin_validator.mutex);
    return ESP_OK;
  }

  /* Free all allocated memory for pin descriptions */
  for (int i = 0; i < GPIO_NUM_MAX; i++) {
    pstar_pin_info_t* pin = &s_pin_validator.pins[i];

    if (pin->users != NULL) {
      for (int j = 0; j < pin->usage_count; j++) {
        if (pin->users[j] != NULL) {
          free(pin->users[j]);
          pin->users[j] = NULL;
        }
      }

      free(pin->users);
      pin->users = NULL;
    }

    /* Reset pin data */
    pin->usage_count   = 0;
    pin->can_be_shared = false;
  }

  s_pin_validator.initialized = false;

  /* Grab mutex handle before releasing and deleting */
  SemaphoreHandle_t mutex_to_delete = s_pin_validator.mutex;
  s_pin_validator.mutex             = NULL; /* Null out global handle */

  /* Release mutex before deleting it */
  xSemaphoreGive(mutex_to_delete);

  /* Delete the mutex */
  vSemaphoreDelete(mutex_to_delete);

  ESP_LOGI(TAG, "Pin validator resources freed successfully");
  return ESP_OK;
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED */
}
