/* components/pstar_bus/bus_manager.c */

#include "pstar_bus_manager.h"
#include "pstar_log_handler.h"
#include "pstar_bus_config.h"
#include <stdlib.h>
#include <string.h>
/* Removed esp_log.h - Use log_xxx now */

/* Constants ******************************************************************/

static const char* TAG = "Bus Manager";

/* Public Functions ***********************************************************/

esp_err_t pstar_bus_manager_init(pstar_bus_manager_t* manager, const char* tag)
{
  /* Validate input */
  if (!manager) {
    /* Use log_error now - assumes minimal logger is initialized before this. */
    log_error(TAG, "Init Fail", "Initialization failed - Bus manager pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  /* Initialize buses list to NULL */
  manager->buses = NULL;

  /* Set logging tag, use default if NULL or empty */
  if (tag && strlen(tag) > 0) {
      /* Use strdup for safety, requires free later */
      manager->tag = strdup(tag);
      if (!manager->tag) {
          /* Use log_error now */
          log_error(TAG, "Init Fail", "Initialization failed - Memory allocation for tag failed");
          return ESP_ERR_NO_MEM;
      }
  } else {
      /* Use a static default tag to avoid allocation */
      manager->tag = TAG; /* Use the component's default tag */
  }

  /* Create the mutex for thread-safe operations */
  manager->mutex = xSemaphoreCreateMutex();
  if (!manager->mutex) { /* Use ! comparison for handles */
    /* Use log_error now */
    log_error(manager->tag, "Init Fail", "Initialization failed - Failed to create mutex for bus manager");
    /* Free tag only if it was dynamically allocated (strdup was used) */
    if (manager->tag != TAG) {
      free((void*)manager->tag); /* Cast needed for free */
      manager->tag = NULL;
    }
    return ESP_ERR_NO_MEM;
  }

  /* Use log_info now */
  log_info(manager->tag, "Init Success", "Bus manager initialized successfully with thread safety");
  return ESP_OK;
}

esp_err_t pstar_bus_manager_add_bus(pstar_bus_manager_t* manager,
                                    pstar_bus_config_t*  config)
{
  /* Validate input */
  if (!manager || !config) {
    /* Fix: Use TAG for initial check */
    log_error(TAG,
              "Bus addition failed",
              "Manager or configuration pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  /* Validate bus configuration */
  /* Fix: Check config->name validity */
  if (!config->name || strlen(config->name) == 0) {
    log_error(manager->tag, /* Use manager's tag once manager is validated */
              "Bus addition failed",
              "Bus configuration lacks a valid name");
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread safety */
  /* Fix: Check mutex validity before taking */
  if (manager->mutex == NULL) {
      log_error(manager->tag, "Bus addition failed", "Mutex not initialized");
      return ESP_ERR_INVALID_STATE;
  }
  if (xSemaphoreTake(manager->mutex, pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_BUS_MUTEX_TIMEOUT_MS)) != pdTRUE) { /* Use Kconfig timeout */
    log_error(manager->tag,
              "Bus addition failed",
              "Failed to acquire mutex for bus addition");
    return ESP_ERR_TIMEOUT;
  }

  /* Check if a bus with the same name already exists */
  for (pstar_bus_config_t* curr = manager->buses; curr; curr = curr->next) {
    /* Fix: Add NULL check for curr->name */
    if (curr->name && config->name && strcmp(curr->name, config->name) == 0) {
      /* Found existing bus with same name */
      xSemaphoreGive(manager->mutex);
      log_error(manager->tag,
                "Bus addition failed",
                "Bus with name '%s' already exists",
                config->name);
      return ESP_ERR_INVALID_STATE; /* Fix: Use specific error for duplicate */
    }
  }

  /* Link the new configuration at the start of the list */
  config->next   = manager->buses;
  manager->buses = config;

  /* Release mutex */
  xSemaphoreGive(manager->mutex);

  log_info(manager->tag,
           "Bus added",
           "Added bus: %s (type: %s)",
           config->name,
           pstar_bus_type_to_string(config->type));
  return ESP_OK;
}

pstar_bus_config_t* pstar_bus_manager_find_bus(const pstar_bus_manager_t* manager,
                                               const char*                name)
{
  /* Validate input */
  if (!manager || !name || strlen(name) == 0) { /* Fix: Check for empty name */
    /* Avoid logging if manager is NULL or name is invalid */
    return NULL;
  }

  pstar_bus_config_t* found_bus = NULL;

  /* Take mutex for thread safety */
  /* Fix: Check mutex handle before taking */
  if (manager->mutex == NULL) {
       log_error(manager->tag, "Bus search failed", "Mutex not initialized");
       return NULL;
  }
  if (xSemaphoreTake(manager->mutex, pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_BUS_MUTEX_TIMEOUT_MS)) != pdTRUE) { /* Use Kconfig timeout */
    log_error(manager->tag,
              "Bus search failed",
              "Failed to acquire mutex for bus search");
    return NULL;
  }

  /* Search through the list of buses */
  for (pstar_bus_config_t* curr = manager->buses; curr; curr = curr->next) {
    /* Fix: Add NULL check for curr->name */
    if (curr->name && strcmp(curr->name, name) == 0) {
      found_bus = curr;
      break;
    }
  }

  /* Release mutex */
  xSemaphoreGive(manager->mutex);

  return found_bus;
}

esp_err_t pstar_bus_manager_remove_bus(pstar_bus_manager_t* manager,
                                       const char*          name)
{
  /* Validate input */
  if (!manager || !name || strlen(name) == 0) { /* Fix: Check for empty name */
    /* Fix: Use TAG for initial check */
    log_error(TAG,
              "Bus removal failed",
              "Manager pointer is NULL or name is NULL/empty");
    return ESP_ERR_INVALID_ARG;
  }

  pstar_bus_config_t* to_remove = NULL;
  pstar_bus_config_t* prev      = NULL; /* Keep track of previous node */
  esp_err_t result = ESP_OK; /* Overall result, prioritize deinit error */

  /* Take mutex for thread safety */
  /* Fix: Check mutex handle before taking */
  if (manager->mutex == NULL) {
       log_error(manager->tag, "Bus removal failed", "Mutex not initialized");
       return ESP_ERR_INVALID_STATE;
  }
  if (xSemaphoreTake(manager->mutex, pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_BUS_MUTEX_TIMEOUT_MS)) != pdTRUE) { /* Use Kconfig timeout */
    log_error(manager->tag,
              "Bus removal failed",
              "Failed to acquire mutex for bus removal");
    return ESP_ERR_TIMEOUT;
  }

  /* Search for the bus in the list */
  for (pstar_bus_config_t* curr = manager->buses; curr; prev = curr, curr = curr->next) {
    /* Fix: Add NULL check for curr->name */
    if (curr->name && strcmp(curr->name, name) == 0) {
      to_remove = curr;
      if (prev == NULL) {
          /* Removing the head of the list */
          manager->buses = curr->next;
      } else {
          /* Removing from middle or end */
          prev->next = curr->next;
      }
      to_remove->next = NULL; /* Break the link */
      break;
    }
  }


  /* Release mutex before potentially time-consuming operations */
  xSemaphoreGive(manager->mutex);

  /* If bus was not found */
  if (!to_remove) {
    log_warn(manager->tag, "Bus removal failed", "Bus '%s' not found", name);
    return ESP_ERR_NOT_FOUND;
  }

  /* Deinitialize the bus if it's initialized */
  if (to_remove->initialized) {
    result = pstar_bus_config_deinit(to_remove);
    if (result != ESP_OK) {
      log_warn(manager->tag,
               "Bus deinit warning",
               "Failed to deinitialize bus '%s' during removal: %s",
               to_remove->name ? to_remove->name : "UNKNOWN", /* Fix: Check name before logging */
               esp_err_to_name(result));
      /* Continue with destruction despite warning */
    }
  }

  /* Destroy the removed bus configuration */
  esp_err_t destroy_result = pstar_bus_config_destroy(to_remove); /* Fix: Use separate variable */
  if (destroy_result != ESP_OK) {
    log_warn(manager->tag,
             "Bus destroy warning",
             "Failed to destroy bus configuration '%s' during removal: %s",
             name, /* Original name is safe here */
             esp_err_to_name(destroy_result));
    /* We've already removed it from the list, so return the deinit result
       or ESP_OK if deinit was skipped/successful */
  }

  log_info(manager->tag, "Bus removed", "Removed and destroyed bus: %s", name);
  /* Return the deinit result if it failed, otherwise the destroy result (or OK) */
  return (result != ESP_OK) ? result : destroy_result;
}

esp_err_t pstar_bus_manager_deinit(pstar_bus_manager_t* manager)
{
  /* Validate input */
  if (!manager) {
    /* Use log_error now, assuming minimal logger is up if deinit is called. */
    log_error(TAG, "Deinit Fail", "Deinitialization failed - Bus manager pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread safety */
  bool mutex_taken = false;
  if (manager->mutex != NULL) {
    /* Use a longer timeout for deinit */
    if (xSemaphoreTake(manager->mutex, pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_BUS_MUTEX_TIMEOUT_MS * 2)) == pdTRUE) {
        mutex_taken = true;
    } else {
        /* Use log_error here as logger *should* be initialized if deinit is called reasonably */
        log_error(manager->tag, "Deinitialization warning", "Failed to acquire mutex for cleanup - attempting anyway (RISKY)");
        /* Continue with cleanup even without mutex, but state is risky */
    }
  } else {
      /* Use log_warn here as logger *should* be initialized */
      log_warn(manager->tag, "Deinitialization warning", "Mutex was NULL during deinit");
  }


  /* Safety: Call deinit and destroy on each bus in the list */
  pstar_bus_config_t* curr = manager->buses;
  manager->buses = NULL; /* Detach list head immediately */

  /* Release mutex before iterating and destroying, prevents holding lock during long operations */
  if (mutex_taken) {
      xSemaphoreGive(manager->mutex);
      mutex_taken = false; /* Mutex is released */
  }

  esp_err_t first_error = ESP_OK; /* Track first error encountered */

  while (curr) {
    pstar_bus_config_t* next = curr->next; /* Store next pointer before destroying curr */

    /* First deinitialize the bus (release hardware resources) */
    esp_err_t result = pstar_bus_config_deinit(curr);
    if (result != ESP_OK) {
      log_warn(manager->tag,
               "Bus deinit warning",
               "Failed to deinitialize bus '%s': %s",
               curr->name ? curr->name : "UNKNOWN", /* Fix: Check name */
               esp_err_to_name(result));
      if (first_error == ESP_OK) first_error = result; /* Record first error */
      /* Continue with cleanup despite the warning */
    }

    /* Then destroy the bus configuration (free memory) */
    result = pstar_bus_config_destroy(curr); /* curr pointer is still valid here */
    if (result != ESP_OK) {
      log_warn(manager->tag,
               "Bus destroy warning",
               "Failed to destroy bus configuration for '%s': %s", /* Log name if possible */
               curr->name ? curr->name : "UNKNOWN",
               esp_err_to_name(result));
      if (first_error == ESP_OK) first_error = result; /* Record first error */
      /* Continue with cleanup despite the warning */
    }
    curr = next; /* Move to the next node */
  }

  /* Free the tag string if it was dynamically allocated */
  /* Fix: Check if tag was allocated with strdup */
  if (manager->tag && manager->tag != TAG) {
    free((void*)manager->tag);
  }
  manager->tag = NULL;


  /* Delete mutex after use */
  if (manager->mutex != NULL) {
    vSemaphoreDelete(manager->mutex);
    manager->mutex = NULL;
  }

  /* Fix: Use TAG for final message. Logger should be available here. */
  log_info(TAG, /* Use component's TAG for the final message */
           "Bus manager deinitialized",
           "Bus manager cleanup %s", (first_error == ESP_OK) ? "successful" : "completed with errors");
  return first_error; /* Return the first error encountered, or ESP_OK */
}
