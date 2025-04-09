/* components/pstar_pin_validator/pstar_pin_validator.c */

#include "pstar_pin_validator.h"

#include "pstar_log_handler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdatomic.h> // Include for atomic operations
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h> // Needed for malloc/free
#include <string.h>

/* Constants ******************************************************************/

static const char* TAG = "PIN_VALIDATOR";

/* Globals (Static) ***********************************************************/

static pin_usage_info_t  s_pin_registry[PIN_VALIDATOR_MAX_PINS] = {0};
static SemaphoreHandle_t s_validator_mutex                      = NULL;
static _Atomic bool      s_initialized = false; // Use atomic for thread safety

/* Private Function Prototypes ************************************************/

static void priv_release_mutex_if_taken(bool* mutex_taken);
static void priv_rebuild_aggregated_strings(int pin);
static void priv_update_overall_shareability(int pin);

/* Public Functions ***********************************************************/

esp_err_t pin_validator_init(void)
{
  // Atomically check if already initialized
  if (atomic_load(&s_initialized)) {
    log_warn(TAG, "Already Init", "Pin validator already initialized");
    return ESP_OK;
  }

  /* Initialize the mutex for thread safety */
  // Check if mutex handle is unexpectedly non-NULL before creating
  if (s_validator_mutex != NULL) {
    log_warn(TAG, "Mutex Exists", "Mutex handle non-NULL before init, potential issue?");
    // Decide recovery strategy: delete existing? return error? For now, log and continue.
  }
  s_validator_mutex = xSemaphoreCreateMutex();
  if (s_validator_mutex == NULL) {
    log_error(TAG, "Mutex Error", "Failed to create mutex");
    return ESP_ERR_NO_MEM;
  }

  /* Initialize the pin registry */
  memset(s_pin_registry, 0, sizeof(s_pin_registry));

  // Atomically set initialized flag *after* successful initialization
  atomic_store(&s_initialized, true);
  log_info(TAG, "Init Complete", "Pin validator initialized successfully");
  return ESP_OK;
}

// --- NEW FUNCTION ---
/**
 * @brief Cleans up the pin validator resources, including deleting the mutex.
 *
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized.
 */
esp_err_t pin_validator_cleanup(void)
{
  // Atomically check if initialized and set to false to prevent races
  if (!atomic_exchange(&s_initialized, false)) {
    log_warn(TAG, "Cleanup Skip", "Pin validator not initialized or already cleaned up.");
    return ESP_OK; // Not an error if already cleaned up
  }

  log_info(TAG, "Cleanup Start", "Cleaning up pin validator...");

  /* Delete the mutex */
  if (s_validator_mutex != NULL) {
    SemaphoreHandle_t temp_mutex = s_validator_mutex;
    s_validator_mutex            = NULL; // Set to NULL *before* deleting
    vSemaphoreDelete(temp_mutex);
    log_info(TAG, "Cleanup", "Pin validator mutex deleted.");
  } else {
    log_warn(TAG, "Cleanup Warning", "Mutex was already NULL during cleanup.");
  }

  /* Optionally clear the registry (good practice) */
  memset(s_pin_registry, 0, sizeof(s_pin_registry));

  log_info(TAG, "Cleanup Complete", "Pin validator cleanup finished.");
  return ESP_OK;
}

esp_err_t pin_validator_register_pin(gpio_num_t  pin,
                                     const char* component_name,
                                     const char* usage_desc,
                                     bool        can_share)

{
  if (!atomic_load(&s_initialized)) { // Use atomic load
    log_error(TAG, "Not Init", "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (pin < 0 || pin >= PIN_VALIDATOR_MAX_PINS) {
    log_error(TAG, "Pin Error", "Invalid pin number: %d", pin);
    return ESP_ERR_INVALID_ARG;
  }

  if (component_name == NULL || usage_desc == NULL || strlen(component_name) == 0 ||
      strlen(usage_desc) == 0) {
    log_error(TAG, "Param Error", "Component name or usage description is NULL or empty");
    return ESP_ERR_INVALID_ARG;
  }

  bool mutex_taken = false;
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(TAG, "Mutex Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  mutex_taken = true;

  pin_usage_info_t* pin_info = &s_pin_registry[pin];

  if (!pin_info->in_use) {
    /* First registration for this pin */
    pin_info->in_use            = true;
    pin_info->num_registrations = 1;
    pin_info->usage_count       = 1;
    /* Overall shareability will be set by priv_update_overall_shareability below */

    /* Initialize first registration */
    strncpy(pin_info->registrations[0].component_name,
            component_name,
            PIN_VALIDATOR_MAX_USAGE_DESC_LEN - 1);
    pin_info->registrations[0].component_name[PIN_VALIDATOR_MAX_USAGE_DESC_LEN - 1] = '\0';
    strncpy(pin_info->registrations[0].usage_desc,
            usage_desc,
            PIN_VALIDATOR_MAX_USAGE_DESC_LEN - 1);
    pin_info->registrations[0].usage_desc[PIN_VALIDATOR_MAX_USAGE_DESC_LEN - 1] = '\0';
    pin_info->registrations[0].count                                            = 1;
    pin_info->registrations[0].can_share =
      can_share; /* Store shareability of this specific registration */

    /* Update overall shareability based on this first registration */
    priv_update_overall_shareability(pin);
    /* Rebuild aggregated strings (will just copy the first one) */
    priv_rebuild_aggregated_strings(pin);

    log_info(TAG,
             "Pin Registered",
             "Registered pin %d for %s: %s (Shareable: %s)",
             pin,
             component_name,
             usage_desc,
             can_share ? "Yes" : "No");
  } else {
    /* Pin already registered: check if this component/usage combo is already present */
    bool found = false;
    for (uint32_t i = 0; i < pin_info->num_registrations; i++) {
      /* Check both component name and usage description for uniqueness */
      if (strcmp(pin_info->registrations[i].component_name, component_name) == 0 &&
          strcmp(pin_info->registrations[i].usage_desc, usage_desc) == 0) {
        /* Found exact existing registration; increment count */
        pin_info->registrations[i].count++;
        pin_info->usage_count++;
        /* Update shareability for this specific registration ONLY if the new one is FALSE */
        if (!can_share) {
          pin_info->registrations[i].can_share = false;
        }
        log_debug(TAG,
                  "Usage Count",
                  "Incremented usage count for pin %d by %s: %s (count: %ld, Shareable: %s)",
                  pin,
                  component_name,
                  pin_info->registrations[i].usage_desc,
                  pin_info->registrations[i].count,
                  pin_info->registrations[i].can_share ? "Yes" : "No");
        found = true;
        break;
      }
    }
    if (!found) {
      /* New registration (different component or different usage by same component) */
      if (pin_info->num_registrations < PIN_VALIDATOR_MAX_REGISTRATIONS) {
        uint32_t idx = pin_info->num_registrations;
        strncpy(pin_info->registrations[idx].component_name,
                component_name,
                PIN_VALIDATOR_MAX_USAGE_DESC_LEN - 1);
        pin_info->registrations[idx].component_name[PIN_VALIDATOR_MAX_USAGE_DESC_LEN - 1] = '\0';
        strncpy(pin_info->registrations[idx].usage_desc,
                usage_desc,
                PIN_VALIDATOR_MAX_USAGE_DESC_LEN - 1);
        pin_info->registrations[idx].usage_desc[PIN_VALIDATOR_MAX_USAGE_DESC_LEN - 1] = '\0';
        pin_info->registrations[idx].count                                            = 1;
        pin_info->registrations[idx].can_share                                        = can_share;
        pin_info->num_registrations++;
        pin_info->usage_count++;

        log_info(TAG,
                 "Pin Shared/Added",
                 "Pin %d now also used by %s: %s (Shareable: %s)",
                 pin,
                 component_name,
                 usage_desc,
                 can_share ? "Yes" : "No");
      } else {
        log_error(TAG, "Registration Error", "Exceeded max registrations for pin %d", pin);
        priv_release_mutex_if_taken(&mutex_taken);
        // --- Use the corrected error code ---
        return ESP_ERR_PSTAR_PIN_VALIDATOR_MAX_REGISTRATIONS_REACHED;
      }
    }
    /* Update overall pin shareability based on all registrations */
    priv_update_overall_shareability(pin);
    /* Rebuild the aggregated strings from all registrations */
    priv_rebuild_aggregated_strings(pin);
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
  if (!atomic_load(&s_initialized)) { // Use atomic load
    log_error(TAG, "Not Init", "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (pins == NULL || num_pins == 0) {
    log_error(TAG, "Param Error", "Invalid pins array or num_pins == 0");
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t result = ESP_OK;

  for (size_t i = 0; i < num_pins; i++) {
    if (pins[i] != -1) { /* Skip -1 pins (not used) */
      esp_err_t pin_result =
        pin_validator_register_pin(pins[i], component_name, usage_desc, can_share);
      if (pin_result != ESP_OK && result == ESP_OK) {
        result = pin_result; /* Record the first error */
      }
    }
  }

  return result;
}

esp_err_t pin_validator_validate_all(bool halt_on_conflict)
{
  if (!atomic_load(&s_initialized)) { // Use atomic load
    log_error(TAG, "Not Init", "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  bool mutex_taken = false;
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(TAG, "Mutex Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  mutex_taken = true;

  bool has_conflicts = false;

  log_info(TAG, "Validation Start", "Validating all pin assignments...");

  /* Check each pin for multiple registrations when overall shareability is false */
  for (int pin = 0; pin < PIN_VALIDATOR_MAX_PINS; pin++) {
    pin_usage_info_t* pin_info = &s_pin_registry[pin];
    /* Conflict condition: More than one *unique registration* AND overall shareability is false */
    if (pin_info->in_use && pin_info->num_registrations > 1 && !pin_info->can_share) {
      /* Conflict: Pin used by multiple components/usages but not shareable */
      log_error(TAG,
                "Pin Conflict",
                "GPIO %d used by multiple components but marked non-shareable by at least one:",
                pin);
      /* Log details of each registration for the conflicting pin */
      for (uint32_t i = 0; i < pin_info->num_registrations; i++) {
        log_error(TAG,
                  "Conflict Detail",
                  "  - %s: '%s' (Shareable: %s, Count: %ld)",
                  pin_info->registrations[i].component_name,
                  pin_info->registrations[i].usage_desc,
                  pin_info->registrations[i].can_share ? "Yes" : "No",
                  pin_info->registrations[i].count);
      }
      has_conflicts = true;
    }
  }

  priv_release_mutex_if_taken(&mutex_taken); /* Release mutex before potentially halting */

  if (has_conflicts) {
    log_error(TAG, "Validation Failed", "PIN VALIDATION FAILED: Conflicts detected!");
    /* The function now just reports the error state. */
    // --- Use the corrected error code ---
    return ESP_ERR_PSTAR_PIN_VALIDATOR_CONFLICT; /* Indicate conflicts were found */
  } else {
    log_info(TAG, "Validation Passed", "No conflicts detected in pin assignments");
    return ESP_OK;
  }
}

esp_err_t pin_validator_print_assignments(void)
{
  if (!atomic_load(&s_initialized)) { // Use atomic load
    log_error(TAG, "Not Init", "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  bool mutex_taken = false;
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(TAG, "Mutex Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  mutex_taken = true;

  log_info(TAG, "Assignment Table", "Building pin assignment table...");

  /* Define column widths */
  const int gpio_width   = 4;
  const int comp_width   = 50;
  const int usage_width  = 50;
  const int shared_width = 7;
  const int count_width  = 5;
  const int max_line_len = gpio_width + comp_width + usage_width + shared_width + count_width +
                           15; // Base width + separators + newline + null

  /* Estimate buffer size: Header(3) + Separators(3) + MaxPins + Footer(1) lines */
  size_t buffer_size  = (3 + 3 + PIN_VALIDATOR_MAX_PINS + 1) * max_line_len;
  char*  table_buffer = NULL;

  /* Safe allocation with null check */
  table_buffer = (char*)malloc(buffer_size);
  if (table_buffer == NULL) {
    log_error(TAG, "Memory Error", "Failed to allocate buffer for pin assignment table");
    priv_release_mutex_if_taken(&mutex_taken);
    return ESP_ERR_NO_MEM;
  }

  char*  buf_ptr        = table_buffer;
  size_t remaining_size = buffer_size;
  int    written        = 0;

  /* --- Build the table string --- */

  /* Build header row */
  char header_row[256];
  int  header_len = snprintf(header_row,
                            sizeof(header_row),
                            "| %-*s | %-*s | %-*s | %-*s | %-*s |",
                            gpio_width,
                            "GPIO",
                            comp_width,
                            "Component(s)",
                            usage_width,
                            "Usage(s)",
                            shared_width,
                            "Shared?",
                            count_width,
                            "Count");

  /* Build a separator line for the header */
  char sep_line[256];
  if (header_len > 0 && header_len < sizeof(sep_line)) {
    memset(sep_line, '=', header_len);
    sep_line[header_len] = '\0';
    // Add top separator
    written = snprintf(buf_ptr, remaining_size, "%s\n", sep_line);
    if (written < 0 || (size_t)written >= remaining_size)
      goto buffer_full;
    buf_ptr += written;
    remaining_size -= written;
  }

  /* Centered heading text */
  const char* heading_text     = " PIN ASSIGNMENT TABLE ";
  int         heading_text_len = strlen(heading_text);
  if (header_len > 0) {
    if (heading_text_len > header_len) {
      heading_text_len = header_len;
    }
    int  left_equals = (header_len - heading_text_len) / 2;
    char heading_line[256];
    memset(heading_line, '=', header_len);
    heading_line[header_len] = '\0';
    /* Replace the center part with heading text */
    if (left_equals >= 0 && left_equals + heading_text_len <= header_len) {
      memcpy(heading_line + left_equals, heading_text, heading_text_len);
    }
    // Add heading line
    written = snprintf(buf_ptr, remaining_size, "%s\n", heading_line);
    if (written < 0 || (size_t)written >= remaining_size)
      goto buffer_full;
    buf_ptr += written;
    remaining_size -= written;
    // Add bottom separator for heading
    written = snprintf(buf_ptr, remaining_size, "%s\n", sep_line);
    if (written < 0 || (size_t)written >= remaining_size)
      goto buffer_full;
    buf_ptr += written;
    remaining_size -= written;
  }

  /* Add table header row and separator */
  if (header_len > 0) {
    written = snprintf(buf_ptr, remaining_size, "%s\n", header_row);
    if (written < 0 || (size_t)written >= remaining_size)
      goto buffer_full;
    buf_ptr += written;
    remaining_size -= written;

    memset(sep_line, '-', header_len);
    sep_line[header_len] = '\0';
    written              = snprintf(buf_ptr, remaining_size, "%s\n", sep_line);
    if (written < 0 || (size_t)written >= remaining_size)
      goto buffer_full;
    buf_ptr += written;
    remaining_size -= written;
  }

  /* Add each in-use pin row using the same column widths */
  bool any_pins_used = false;
  for (int pin = 0; pin < PIN_VALIDATOR_MAX_PINS; pin++) {
    pin_usage_info_t* pin_info = &s_pin_registry[pin];
    if (pin_info->in_use) {
      any_pins_used = true;
      written       = snprintf(buf_ptr,
                         remaining_size,
                         "| %-*d | %-*.*s | %-*.*s | %-*s | %-*ld |\n",
                         gpio_width,
                         pin,
                         comp_width,
                         comp_width,
                         pin_info->aggregated_component_name,
                         usage_width,
                         usage_width,
                         pin_info->aggregated_usage_desc,
                         shared_width,
                         pin_info->can_share ? "Yes" : "No", /* Use the overall shareability flag */
                         count_width,
                         pin_info->usage_count);
      if (written < 0 || (size_t)written >= remaining_size)
        goto buffer_full;
      buf_ptr += written;
      remaining_size -= written;
    }
  }

  if (!any_pins_used) {
    const char* msg         = "No pins registered yet.";
    int         msg_len     = strlen(msg);
    int         inner_width = header_len > 2 ? header_len - 2 : 1;
    int         pad_before  = (inner_width - msg_len) / 2;
    if (pad_before < 0)
      pad_before = 0;
    int pad_after = inner_width - msg_len - pad_before;
    if (pad_after < 0)
      pad_after = 0;

    written = snprintf(buf_ptr, remaining_size, "|%*s%s%*s|\n", pad_before, "", msg, pad_after, "");
    if (written < 0 || (size_t)written >= remaining_size)
      goto buffer_full;
    buf_ptr += written;
    remaining_size -= written;
  }

  /* Add final separator line */
  if (header_len > 0) {
    memset(sep_line, '-', header_len);
    sep_line[header_len] = '\0';
    written              = snprintf(buf_ptr, remaining_size, "%s\n", sep_line);
    if (written < 0 || (size_t)written >= remaining_size)
      goto buffer_full;
    buf_ptr += written;
    remaining_size -= written;
  }

buffer_full:
  if (remaining_size <= 1) { // Check if buffer became full during construction
    log_warn(TAG, "Buffer Full", "Pin assignment table truncated due to buffer size limit.");
    // Ensure null termination even if truncated
    if (buffer_size > 0) {
      table_buffer[buffer_size - 1] = '\0';
    }
  }

  /* --- Print the entire buffered table --- */
  if (table_buffer != NULL) {
    printf("%s", table_buffer); // Use printf for potentially large string
  }

  /* --- Cleanup --- */
  if (table_buffer != NULL) {
    free(table_buffer);
    table_buffer = NULL; // Set to NULL after freeing
  }

  priv_release_mutex_if_taken(&mutex_taken);
  return ESP_OK;
}

esp_err_t pin_validator_clear_all(void)
{
  if (!atomic_load(&s_initialized)) { // Use atomic load
    log_error(TAG, "Not Init", "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  bool mutex_taken = false;
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(TAG, "Mutex Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  mutex_taken = true;

  memset(s_pin_registry, 0, sizeof(s_pin_registry));
  log_info(TAG, "Registry Cleared", "Cleared all pin assignments");

  priv_release_mutex_if_taken(&mutex_taken);
  return ESP_OK;
}

esp_err_t pin_validator_unregister_pin(gpio_num_t pin, const char* component_name)
{
  if (!atomic_load(&s_initialized)) { // Use atomic load
    log_error(TAG, "Not Init", "Pin validator not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (pin < 0 || pin >= PIN_VALIDATOR_MAX_PINS) {
    log_error(TAG, "Pin Error", "Invalid pin number: %d", pin);
    return ESP_ERR_INVALID_ARG;
  }

  if (component_name == NULL || strlen(component_name) == 0) {
    log_error(TAG, "Param Error", "Component name is NULL or empty");
    return ESP_ERR_INVALID_ARG;
  }

  bool mutex_taken = false;
  if (xSemaphoreTake(s_validator_mutex, portMAX_DELAY) != pdTRUE) {
    log_error(TAG, "Mutex Error", "Failed to take mutex");
    return ESP_ERR_TIMEOUT;
  }
  mutex_taken = true;

  pin_usage_info_t* pin_info = &s_pin_registry[pin];
  if (!pin_info->in_use) {
    log_warn(TAG, "Not Registered", "Pin %d not registered, cannot unregister", pin);
    priv_release_mutex_if_taken(&mutex_taken);
    return ESP_ERR_NOT_FOUND;
  }

  /* Find the registration corresponding to component_name */
  int reg_index = -1; /* Use int for index */
  for (uint32_t i = 0; i < pin_info->num_registrations; i++) {
    if (strcmp(pin_info->registrations[i].component_name, component_name) == 0) {
      reg_index = i;
      break;
    }
  }
  if (reg_index == -1) {
    log_warn(TAG,
             "Component Mismatch",
             "Pin %d not registered by %s, cannot unregister",
             pin,
             component_name);
    priv_release_mutex_if_taken(&mutex_taken);
    return ESP_ERR_NOT_FOUND;
  }

  /* Decrement usage count for the registration and overall usage count */
  if (pin_info->registrations[reg_index].count > 0) {
    pin_info->registrations[reg_index].count--;
    pin_info->usage_count--;
  }

  if (pin_info->registrations[reg_index].count == 0) {
    /* Remove this registration by shifting subsequent registrations */
    for (uint32_t i = reg_index; i < pin_info->num_registrations - 1; i++) {
      pin_info->registrations[i] = pin_info->registrations[i + 1];
    }
    /* Clear the last (now unused) registration slot */
    if (pin_info->num_registrations > 0) {
      memset(&pin_info->registrations[pin_info->num_registrations - 1],
             0,
             sizeof(pin_registration_t));
    }
    pin_info->num_registrations--;
    log_info(TAG,
             "Registration Removed",
             "Removed registration for %s on pin %d",
             component_name,
             pin);
  } else {
    log_info(TAG,
             "Usage Decremented",
             "Decremented usage count for pin %d by %s (remaining count: %ld)",
             pin,
             component_name,
             pin_info->registrations[reg_index].count);
  }

  /* Recalculate overall shareability from remaining registrations */
  priv_update_overall_shareability(pin);

  /* Rebuild aggregated strings from remaining registrations */
  priv_rebuild_aggregated_strings(pin);

  /* If no registrations remain, clear the pin info completely */
  if (pin_info->usage_count == 0) {
    memset(pin_info, 0, sizeof(pin_usage_info_t));
    log_info(TAG, "Pin Unregistered", "Unregistered pin %d completely", pin);
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

/**
 * @brief Rebuild aggregated component and usage strings for a given pin
 *
 * This function concatenates the component names and usage descriptions
 * from all registrations of a pin into aggregated strings.
 *
 * Optimized using pointer arithmetic and snprintf to avoid repeated scanning.
 * Assumes mutex is held.
 *
 * @param pin Pin index to rebuild aggregated strings for.
 */
static void priv_rebuild_aggregated_strings(int pin)
{
  pin_usage_info_t* pin_info        = &s_pin_registry[pin];
  char*             comp_ptr        = pin_info->aggregated_component_name;
  char*             usage_ptr       = pin_info->aggregated_usage_desc;
  size_t            comp_remaining  = PIN_VALIDATOR_MAX_USAGE_DESC_LEN;
  size_t            usage_remaining = PIN_VALIDATOR_MAX_USAGE_DESC_LEN;
  int               written         = 0;

  /* Clear aggregated strings */
  if (comp_remaining > 0) {
    *comp_ptr = '\0';
    comp_remaining--; /* Account for null terminator */
  } else {
    comp_ptr = NULL; /* Avoid using if buffer size is 0 */
  }

  if (usage_remaining > 0) {
    *usage_ptr = '\0';
    usage_remaining--; /* Account for null terminator */
  } else {
    usage_ptr = NULL;
  }

  for (uint32_t i = 0; i < pin_info->num_registrations; i++) {
    /* Add separator if not the first item */
    if (i > 0) {
      if (comp_ptr && comp_remaining >= 2) {
        written = snprintf(comp_ptr, comp_remaining + 1, ", ");
        if (written > 0 && (size_t)written <= comp_remaining) {
          comp_ptr += written;
          comp_remaining -= written;
        } else {
          comp_ptr = NULL; /* Buffer full */
        }
      } else {
        comp_ptr = NULL;
      }

      if (usage_ptr && usage_remaining >= 2) {
        written = snprintf(usage_ptr, usage_remaining + 1, ", ");
        if (written > 0 && (size_t)written <= usage_remaining) {
          usage_ptr += written;
          usage_remaining -= written;
        } else {
          usage_ptr = NULL; /* Buffer full */
        }
      } else {
        usage_ptr = NULL;
      }
    }

    /* Append component name */
    if (comp_ptr) {
      written =
        snprintf(comp_ptr, comp_remaining + 1, "%s", pin_info->registrations[i].component_name);
      if (written > 0 && (size_t)written <= comp_remaining) {
        comp_ptr += written;
        comp_remaining -= written;
      } else {
        comp_ptr = NULL; /* Buffer full or error */
      }
    }

    /* Append usage description */
    if (usage_ptr) {
      written =
        snprintf(usage_ptr, usage_remaining + 1, "%s", pin_info->registrations[i].usage_desc);
      if (written > 0 && (size_t)written <= usage_remaining) {
        usage_ptr += written;
        usage_remaining -= written;
      } else {
        usage_ptr = NULL; /* Buffer full or error */
      }
    }
    /* Stop if buffers are full */
    if (!comp_ptr && !usage_ptr) {
      break;
    }
  }
}

/**
 * @brief Update overall shareability of a pin based on its registrations.
 * Assumes mutex is held.
 *
 * @param pin Pin index to update.
 */
static void priv_update_overall_shareability(int pin)
{
  pin_usage_info_t* pin_info = &s_pin_registry[pin];
  pin_info->can_share        = true;                           /* Assume shareable initially */
  for (uint32_t i = 0; i < pin_info->num_registrations; i++) { /* Iterate up to num_registrations */
    if (!pin_info->registrations[i].can_share) {
      pin_info->can_share = false; /* If *any* registration is not shareable, the pin isn't */
      break;                       /* No need to check further */
    }
  }
}
