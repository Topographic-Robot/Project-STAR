/* components/pstar_pin_validator/pin_validator.c */

#include "pin_validator.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

#define PIN_VALIDATOR_TAG ("Pin Validator")

/* Global pin registry */
static pin_usage_info_t  s_pin_registry[PIN_VALIDATOR_MAX_PINS] = {0};
static SemaphoreHandle_t s_validator_mutex                      = NULL;
static bool              s_initialized                          = false;

/* Helper function to safely release mutex */
static void release_mutex(void) 
{
  if (s_validator_mutex != NULL) {
    xSemaphoreGive(s_validator_mutex);
  }
}

esp_err_t pin_validator_init(void) 
{
  if (s_initialized) {
    ESP_LOGW(PIN_VALIDATOR_TAG, "Pin validator already initialized");
    return ESP_OK;
  }
  
  /* Initialize the mutex for thread safety */
  s_validator_mutex = xSemaphoreCreateMutex();
  if (s_validator_mutex == NULL) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Failed to create mutex");
    return ESP_ERR_NO_MEM;
  }
  
  /* Initialize the pin registry */
  memset(s_pin_registry, 0, sizeof(s_pin_registry));
  
  s_initialized = true;
  ESP_LOGI(PIN_VALIDATOR_TAG, "Pin validator initialized");
  return ESP_OK;
}

esp_err_t pin_validator_register_pin(gpio_num_t  pin, 
                                     const char* component_name,
                                     const char* usage_desc,
                                     bool        can_share) 
{
  if (!s_initialized) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (pin < 0 || pin >= PIN_VALIDATOR_MAX_PINS) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Invalid pin number: %d", pin);
    return ESP_ERR_INVALID_ARG;
  }
  
  if (component_name == NULL || usage_desc == NULL) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Component name or usage description is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  
  /* Check if pin is already registered */
  if (s_pin_registry[pin].in_use) {
    /* If pin is already registered by this component, just increment usage count */
    if (strcmp(s_pin_registry[pin].component_name, component_name) == 0) {
      s_pin_registry[pin].usage_count++;
      ESP_LOGD(PIN_VALIDATOR_TAG, 
               "Incremented usage count for pin %d by %s: %s (count: %d)",
              pin, 
              component_name, 
              usage_desc, 
              s_pin_registry[pin].usage_count);
      release_mutex();
      return ESP_OK;
    }
    
    /* If pin can't be shared, flag a conflict */
    if (!s_pin_registry[pin].can_share || !can_share) {
      ESP_LOGW(PIN_VALIDATOR_TAG, 
               "Pin %d already registered by %s for %s, new request from %s for %s",
              pin, 
              s_pin_registry[pin].component_name, 
              s_pin_registry[pin].usage_desc,
              component_name, usage_desc);
      
      /* Still register it to track the conflict */
      s_pin_registry[pin].can_share = false; /* Mark as conflict */
    } else {
      ESP_LOGI(PIN_VALIDATOR_TAG, 
               "Pin %d will be shared between %s and %s",
                pin, 
                s_pin_registry[pin].component_name, 
                component_name);
    }
  } else {
    /* First registration for this pin */
    s_pin_registry[pin].in_use = true;
    s_pin_registry[pin].usage_count = 1;
    s_pin_registry[pin].can_share = can_share;
    
    ESP_LOGI(PIN_VALIDATOR_TAG, 
             "Registered pin %d for %s: %s",
             pin, 
             component_name, 
             usage_desc);
  }
  
  /* Update component name by concatenating or replacing */
  if (s_pin_registry[pin].usage_count > 1) {
    /* Append new component name if not already in the string */
    if (strstr(s_pin_registry[pin].component_name, component_name) == NULL) {
      size_t current_len = strlen(s_pin_registry[pin].component_name);
      size_t avail_space = PIN_VALIDATOR_MAX_USAGE_DESC_LEN - current_len - 1;
      
      if (avail_space > strlen(component_name) + 2) { /* +2 for ", " */
        strcat(s_pin_registry[pin].component_name, ", ");
        strcat(s_pin_registry[pin].component_name, component_name);
      } else {
        strncat(s_pin_registry[pin].component_name, "...", avail_space);
      }
    }
    
    /* Append new usage description if not already in the string */
    if (strstr(s_pin_registry[pin].usage_desc, usage_desc) == NULL) {
      size_t current_len = strlen(s_pin_registry[pin].usage_desc);
      size_t avail_space = PIN_VALIDATOR_MAX_USAGE_DESC_LEN - current_len - 1;
      
      if (avail_space > strlen(usage_desc) + 2) { /* +2 for ", " */
        strcat(s_pin_registry[pin].usage_desc, ", ");
        strcat(s_pin_registry[pin].usage_desc, usage_desc);
      } else {
        strncat(s_pin_registry[pin].usage_desc, "...", avail_space);
      }
    }
  } else {
    /* First usage, just copy the strings */
    strncpy(s_pin_registry[pin].component_name, component_name, PIN_VALIDATOR_MAX_USAGE_DESC_LEN - 1);
    s_pin_registry[pin].component_name[PIN_VALIDATOR_MAX_USAGE_DESC_LEN - 1] = '\0';
    
    strncpy(s_pin_registry[pin].usage_desc, usage_desc, PIN_VALIDATOR_MAX_USAGE_DESC_LEN - 1);
    s_pin_registry[pin].usage_desc[PIN_VALIDATOR_MAX_USAGE_DESC_LEN - 1] = '\0';
  }
  
  release_mutex();
  return ESP_OK;
}

esp_err_t pin_validator_register_pins(const gpio_num_t* pins,
                                     size_t num_pins,
                                     const char* component_name,
                                     const char* usage_desc,
                                     bool can_share) {
  if (!s_initialized) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (pins == NULL || num_pins == 0) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Invalid pins array or num_pins == 0");
    return ESP_ERR_INVALID_ARG;
  }
  
  esp_err_t result = ESP_OK;
  
  for (size_t i = 0; i < num_pins; i++) {
    if (pins[i] != -1) { /* Skip -1 pins (not used) */
      esp_err_t pin_result = pin_validator_register_pin(pins[i], component_name, usage_desc, can_share);
      if (pin_result != ESP_OK && result == ESP_OK) {
        /* Remember the first error but continue registering other pins */
        result = pin_result;
      }
    }
  }
  
  return result;
}

esp_err_t pin_validator_validate_all(bool halt_on_conflict) {
  if (!s_initialized) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  
  bool has_conflicts = false;
  
  ESP_LOGI(PIN_VALIDATOR_TAG, "Validating all pin assignments...");
  
  /* Check each pin for multiple non-shareable registrations */
  for (int pin = 0; pin < PIN_VALIDATOR_MAX_PINS; pin++) {
    if (s_pin_registry[pin].in_use && s_pin_registry[pin].usage_count > 1 && !s_pin_registry[pin].can_share) {
      ESP_LOGE(PIN_VALIDATOR_TAG, "PIN CONFLICT: GPIO %d used by multiple components: %s",
              pin, s_pin_registry[pin].component_name);
      ESP_LOGE(PIN_VALIDATOR_TAG, "    Usage: %s", s_pin_registry[pin].usage_desc);
      has_conflicts = true;
    }
  }
  
  if (has_conflicts) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "PIN VALIDATION FAILED: Conflicts detected!");
    if (halt_on_conflict) {
      ESP_LOGE(PIN_VALIDATOR_TAG, "System halted due to pin conflicts. Please fix your configuration.");
      release_mutex();
      while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
    }
    release_mutex();
    return ESP_ERR_INVALID_STATE;
  } else {
    ESP_LOGI(PIN_VALIDATOR_TAG, "PIN VALIDATION PASSED: No conflicts detected.");
    release_mutex();
    return ESP_OK;
  }
}

esp_err_t pin_validator_print_assignments(void) {
  if (!s_initialized) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  
  ESP_LOGI(PIN_VALIDATOR_TAG, "=== PIN ASSIGNMENT TABLE ===");
  ESP_LOGI(PIN_VALIDATOR_TAG, "| GPIO | Component      | Usage                  | Shared | Count |");
  ESP_LOGI(PIN_VALIDATOR_TAG, "|------|----------------|------------------------|--------|-------|");
  
  bool any_pins_used = false;
  
  for (int pin = 0; pin < PIN_VALIDATOR_MAX_PINS; pin++) {
    if (s_pin_registry[pin].in_use) {
      any_pins_used = true;
      ESP_LOGI(PIN_VALIDATOR_TAG, "| %-4d | %-14s | %-22s | %-6s | %-5d |",
              pin, 
              s_pin_registry[pin].component_name,
              s_pin_registry[pin].usage_desc,
              s_pin_registry[pin].can_share ? "Yes" : "No",
              s_pin_registry[pin].usage_count);
    }
  }
  
  if (!any_pins_used) {
    ESP_LOGI(PIN_VALIDATOR_TAG, "No pins registered yet.");
  }
  
  ESP_LOGI(PIN_VALIDATOR_TAG, "=========================");
  
  release_mutex();
  return ESP_OK;
}

esp_err_t pin_validator_clear_all(void) {
  if (!s_initialized) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  
  memset(s_pin_registry, 0, sizeof(s_pin_registry));
  ESP_LOGI(PIN_VALIDATOR_TAG, "Cleared all pin assignments");
  
  release_mutex();
  return ESP_OK;
}

esp_err_t pin_validator_unregister_pin(gpio_num_t pin, const char* component_name) {
  if (!s_initialized) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (pin < 0 || pin >= PIN_VALIDATOR_MAX_PINS) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Invalid pin number: %d", pin);
    return ESP_ERR_INVALID_ARG;
  }
  
  if (component_name == NULL) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Component name is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE(PIN_VALIDATOR_TAG, "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  
  if (!s_pin_registry[pin].in_use) {
    ESP_LOGW(PIN_VALIDATOR_TAG, "Pin %d not registered, cannot unregister", pin);
    release_mutex();
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this component has registered this pin */
  if (strstr(s_pin_registry[pin].component_name, component_name) == NULL) {
    ESP_LOGW(PIN_VALIDATOR_TAG, "Pin %d not registered by %s, cannot unregister", pin, component_name);
    release_mutex();
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Decrement usage count */
  s_pin_registry[pin].usage_count--;
  
  /* If pin is no longer used, clear its entry */
  if (s_pin_registry[pin].usage_count <= 0) {
    memset(&s_pin_registry[pin], 0, sizeof(pin_usage_info_t));
    ESP_LOGI(PIN_VALIDATOR_TAG, "Unregistered pin %d completely", pin);
  } else {
    ESP_LOGI(PIN_VALIDATOR_TAG, "Decremented usage count for pin %d (count: %d)", 
            pin, s_pin_registry[pin].usage_count);
    
    /* This is simplified and does not update the component_name and usage_desc strings
     * A more complete implementation would rebuild these strings */
  }
  
  release_mutex();
  return ESP_OK;
}
