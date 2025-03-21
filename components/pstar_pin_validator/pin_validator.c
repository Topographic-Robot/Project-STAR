/* components/pstar_pin_validator/pin_validator.c */

#include "pin_validator.h"
#include "log_handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

/* Constants ******************************************************************/

#define PIN_VALIDATOR_TAG ("Pin Validator")

/* Globals (Static) ***********************************************************/

static pin_usage_info_t  s_pin_registry[PIN_VALIDATOR_MAX_PINS] = {0};
static SemaphoreHandle_t s_validator_mutex                      = NULL;
static bool              s_initialized                          = false;

/* Private Function Prototypes ************************************************/

static void priv_release_mutex_if_taken(bool* mutex_taken);

/* Public Functions ***********************************************************/

esp_err_t pin_validator_init(void) 
{
  if (s_initialized) {
    log_warn(PIN_VALIDATOR_TAG, 
             "Already Init", 
             "Pin validator already initialized");
    return ESP_OK;
  }
  
  /* Initialize the mutex for thread safety */
  s_validator_mutex = xSemaphoreCreateMutex();
  if (s_validator_mutex == NULL) {
    log_error(PIN_VALIDATOR_TAG, 
              "Init Error", 
              "Failed to create mutex");
    return ESP_ERR_NO_MEM;
  }
  
  /* Initialize the pin registry */
  memset(s_pin_registry, 0, sizeof(s_pin_registry));
  
  s_initialized = true;
  log_info(PIN_VALIDATOR_TAG, 
           "Init Complete", 
           "Pin validator initialized successfully");
  return ESP_OK;
}

esp_err_t pin_validator_register_pin(gpio_num_t  pin, 
                                     const char* component_name,
                                     const char* usage_desc,
                                     bool        can_share) 
{
  if (!s_initialized) {
    log_error(PIN_VALIDATOR_TAG, 
              "Not Init", 
              "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (pin < 0 || pin >= PIN_VALIDATOR_MAX_PINS) {
    log_error(PIN_VALIDATOR_TAG, 
              "Pin Error", 
              "Invalid pin number: %d", 
              pin);
    return ESP_ERR_INVALID_ARG;
  }
  
  if (component_name == NULL || usage_desc == NULL) {
    log_error(PIN_VALIDATOR_TAG, 
              "Param Error", 
              "Component name or usage description is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  bool mutex_taken = false;
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(PIN_VALIDATOR_TAG, 
              "Mutex Error", 
              "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  mutex_taken = true;
  
  /* Check if pin is already registered */
  if (s_pin_registry[pin].in_use) {
    /* If pin is already registered by this component, just increment usage count */
    if (strcmp(s_pin_registry[pin].component_name, component_name) == 0) {
      s_pin_registry[pin].usage_count++;
      log_debug(PIN_VALIDATOR_TAG, 
                "Usage Count", 
                "Incremented usage count for pin %d by %s: %s (count: %d)",
                pin, 
                component_name, 
                usage_desc, 
                s_pin_registry[pin].usage_count);
      priv_release_mutex_if_taken(&mutex_taken);
      return ESP_OK;
    }
    
    /* If pin can't be shared, flag a conflict */
    if (!s_pin_registry[pin].can_share || !can_share) {
      log_warn(PIN_VALIDATOR_TAG, 
               "Pin Conflict", 
               "Pin %d already registered by %s for %s, new request from %s for %s",
               pin, 
               s_pin_registry[pin].component_name, 
               s_pin_registry[pin].usage_desc,
               component_name, 
               usage_desc);
      
      /* Still register it to track the conflict */
      s_pin_registry[pin].can_share = false; /* Mark as conflict */
    } else {
      log_info(PIN_VALIDATOR_TAG, 
               "Pin Shared", 
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
    
    log_info(PIN_VALIDATOR_TAG, 
             "Pin Registered", 
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
  
  priv_release_mutex_if_taken(&mutex_taken);
  return ESP_OK;
}

esp_err_t pin_validator_register_pins(const gpio_num_t* pins,
                                     size_t            num_pins,
                                     const char*       component_name,
                                     const char*       usage_desc,
                                     bool              can_share) 
{
  if (!s_initialized) {
    log_error(PIN_VALIDATOR_TAG, 
              "Not Init", 
              "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (pins == NULL || num_pins == 0) {
    log_error(PIN_VALIDATOR_TAG, 
              "Param Error", 
              "Invalid pins array or num_pins == 0");
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

esp_err_t pin_validator_validate_all(bool halt_on_conflict) 
{
  if (!s_initialized) {
    log_error(PIN_VALIDATOR_TAG, 
              "Not Init", 
              "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  
  bool mutex_taken = false;
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(PIN_VALIDATOR_TAG, 
              "Mutex Error", 
              "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  mutex_taken = true;
  
  bool has_conflicts = false;
  
  log_info(PIN_VALIDATOR_TAG, 
           "Validation Start", 
           "Validating all pin assignments...");
  
  /* Check each pin for multiple non-shareable registrations */
  for (int pin = 0; pin < PIN_VALIDATOR_MAX_PINS; pin++) {
    if (s_pin_registry[pin].in_use && s_pin_registry[pin].usage_count > 1 && !s_pin_registry[pin].can_share) {
      log_error(PIN_VALIDATOR_TAG, 
                "Pin Conflict", 
                "GPIO %d used by multiple components: %s",
                pin, 
                s_pin_registry[pin].component_name);
      log_error(PIN_VALIDATOR_TAG, 
                "Usage Details", 
                "Usage: %s", 
                s_pin_registry[pin].usage_desc);
      has_conflicts = true;
    }
  }
  
  if (has_conflicts) {
    log_error(PIN_VALIDATOR_TAG, 
              "Validation Failed", 
              "PIN VALIDATION FAILED: Conflicts detected!");
    if (halt_on_conflict) {
      log_error(PIN_VALIDATOR_TAG, 
                "System Halted", 
                "System halted due to pin conflicts. Please fix your configuration.");
      priv_release_mutex_if_taken(&mutex_taken);
      while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    }
    priv_release_mutex_if_taken(&mutex_taken);
    return ESP_ERR_INVALID_STATE;
  } else {
    log_info(PIN_VALIDATOR_TAG, 
             "Validation Passed", 
             "No conflicts detected in pin assignments");
    priv_release_mutex_if_taken(&mutex_taken);
    return ESP_OK;
  }
}

esp_err_t pin_validator_print_assignments(void) 
{
  if (!s_initialized) {
    log_error(PIN_VALIDATOR_TAG, 
              "Not Init", 
              "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  
  bool mutex_taken = false;
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(PIN_VALIDATOR_TAG, 
              "Mutex Error", 
              "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  mutex_taken = true;
  
  log_info(PIN_VALIDATOR_TAG, 
           "Assignment Table", 
           "=== PIN ASSIGNMENT TABLE ===");
  log_info(PIN_VALIDATOR_TAG, 
           "Table Header", 
           "| GPIO | Component      | Usage                  | Shared | Count |");
  log_info(PIN_VALIDATOR_TAG, 
           "Table Separator", 
           "|------|----------------|------------------------|--------|-------|");
  
  bool any_pins_used = false;
  
  for (int pin = 0; pin < PIN_VALIDATOR_MAX_PINS; pin++) {
    if (s_pin_registry[pin].in_use) {
      any_pins_used = true;
      log_info(PIN_VALIDATOR_TAG, 
               "Pin Info", 
               "| %-4d | %-14s | %-22s | %-6s | %-5d |",
               pin, 
               s_pin_registry[pin].component_name,
               s_pin_registry[pin].usage_desc,
               s_pin_registry[pin].can_share ? "Yes" : "No",
               s_pin_registry[pin].usage_count);
    }
  }
  
  if (!any_pins_used) {
    log_info(PIN_VALIDATOR_TAG, 
             "No Pins", 
             "No pins registered yet.");
  }
  
  log_info(PIN_VALIDATOR_TAG, 
           "Table Footer", 
           "=========================");
  
  priv_release_mutex_if_taken(&mutex_taken);
  return ESP_OK;
}

esp_err_t pin_validator_clear_all(void) 
{
  if (!s_initialized) {
    log_error(PIN_VALIDATOR_TAG, 
              "Not Init", 
              "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  
  bool mutex_taken = false;
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(PIN_VALIDATOR_TAG, 
              "Mutex Error", 
              "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  mutex_taken = true;
  
  memset(s_pin_registry, 0, sizeof(s_pin_registry));
  log_info(PIN_VALIDATOR_TAG, 
           "Registry Cleared", 
           "Cleared all pin assignments");
  
  priv_release_mutex_if_taken(&mutex_taken);
  return ESP_OK;
}

esp_err_t pin_validator_unregister_pin(gpio_num_t pin, const char* component_name) 
{
  if (!s_initialized) {
    log_error(PIN_VALIDATOR_TAG, 
              "Not Init", 
              "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (pin < 0 || pin >= PIN_VALIDATOR_MAX_PINS) {
    log_error(PIN_VALIDATOR_TAG, 
              "Pin Error", 
              "Invalid pin number: %d", 
              pin);
    return ESP_ERR_INVALID_ARG;
  }
  
  if (component_name == NULL) {
    log_error(PIN_VALIDATOR_TAG, 
              "Param Error", 
              "Component name is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  bool mutex_taken = false;
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(PIN_VALIDATOR_TAG, 
              "Mutex Error", 
              "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  mutex_taken = true;
  
  if (!s_pin_registry[pin].in_use) {
    log_warn(PIN_VALIDATOR_TAG, 
             "Not Registered", 
             "Pin %d not registered, cannot unregister", 
             pin);
    priv_release_mutex_if_taken(&mutex_taken);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this component has registered this pin */
  if (strstr(s_pin_registry[pin].component_name, component_name) == NULL) {
    log_warn(PIN_VALIDATOR_TAG, 
             "Component Mismatch", 
             "Pin %d not registered by %s, cannot unregister", 
             pin, 
             component_name);
    priv_release_mutex_if_taken(&mutex_taken);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Decrement usage count */
  s_pin_registry[pin].usage_count--;
  
  /* If pin is no longer used, clear its entry */
  if (s_pin_registry[pin].usage_count <= 0) {
    memset(&s_pin_registry[pin], 0, sizeof(pin_usage_info_t));
    log_info(PIN_VALIDATOR_TAG, 
             "Pin Unregistered", 
             "Unregistered pin %d completely", 
             pin);
  } else {
    log_info(PIN_VALIDATOR_TAG, 
             "Usage Decremented", 
             "Decremented usage count for pin %d (count: %d)", 
             pin, 
             s_pin_registry[pin].usage_count);
    
    /* This is simplified and does not update the component_name and usage_desc strings
     * A more complete implementation would rebuild these strings */
  }
  
  priv_release_mutex_if_taken(&mutex_taken);
  return ESP_OK;
}

/* Private Functions **********************************************************/

/**
 * @brief Safely releases mutex if it was taken
 *
 * @param[in,out] mutex_taken Pointer to boolean indicating if mutex is taken
 */
static void priv_release_mutex_if_taken(bool* mutex_taken)
{
  if (s_validator_mutex != NULL && *mutex_taken) {
    xSemaphoreGive(s_validator_mutex);
    *mutex_taken = false;
  }
}