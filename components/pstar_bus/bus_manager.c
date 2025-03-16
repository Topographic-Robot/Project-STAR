/* components/pstar_bus/bus_manager.c */

#include "bus_manager.h"
#include "log_handler.h"
#include "bus_config.h"
#include <stdlib.h>
#include <string.h>

/* Constants ******************************************************************/

static const char* const bus_manager_tag = "Bus Manager";

/* Public Functions ***********************************************************/

esp_err_t pstar_bus_manager_init(pstar_bus_manager_t* manager, const char* tag)
{
  /* Validate input */
  if (!manager) {
    log_error(bus_manager_tag, 
              "Initialization failed", 
              "Bus manager pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  /* Initialize buses list to NULL */
  manager->buses = NULL;

  /* Set logging tag, use default if NULL */
  if (tag) {
    manager->tag = strdup(tag);
  } else {
    char* temp;
    if (asprintf(&temp, "%s Default Tag", bus_manager_tag) == -1) {
      log_error(bus_manager_tag, 
                "Initialization failed", 
                "Memory allocation for tag failed");
      return ESP_ERR_NO_MEM;
    }
    manager->tag = temp;
  }

  /* Create the mutex for thread-safe operations */
  manager->mutex = xSemaphoreCreateMutex();
  if (manager->mutex == NULL) {
    log_error(bus_manager_tag, 
              "Initialization failed", 
              "Failed to create mutex for bus manager");
    if (manager->tag) {
      free((void*)manager->tag);
      manager->tag = NULL;
    }
    return ESP_ERR_NO_MEM;
  }

  log_info(manager->tag, 
           "Bus manager initialized", 
           "Successfully initialized bus manager with thread safety");
  return ESP_OK;
}

esp_err_t pstar_bus_manager_add_bus(pstar_bus_manager_t* manager, 
                                    pstar_bus_config_t*  config)
{
  /* Validate input */
  if (!manager || !config) {
    log_error(bus_manager_tag, 
              "Bus addition failed", 
              "Manager or configuration pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  /* Validate bus configuration */
  if (!config->name) {
    log_error(bus_manager_tag, 
              "Bus addition failed", 
              "Bus configuration lacks a name");
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread safety */
  if (xSemaphoreTake(manager->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(manager->tag, 
              "Bus addition failed", 
              "Failed to acquire mutex for bus addition");
    return ESP_ERR_TIMEOUT;
  }

  /* Check if a bus with the same name already exists */
  for (pstar_bus_config_t* curr = manager->buses; curr; curr = curr->next) {
    if (curr->name && strcmp(curr->name, config->name) == 0) {
      /* Found existing bus with same name */
      xSemaphoreGive(manager->mutex);
      log_error(manager->tag, 
                "Bus addition failed", 
                "Bus with name '%s' already exists", 
                config->name);
      return ESP_ERR_INVALID_STATE;
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
  if (!manager || !name) {
    log_error(bus_manager_tag, 
              "Bus search failed", 
              "Manager or name pointer is NULL");
    return NULL;
  }

  pstar_bus_config_t* found_bus = NULL;

  /* Take mutex for thread safety */
  if (xSemaphoreTake(manager->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(manager->tag, 
              "Bus search failed", 
              "Failed to acquire mutex for bus search");
    return NULL;
  }

  /* Search through the list of buses */
  for (pstar_bus_config_t* curr = manager->buses; curr; curr = curr->next) {
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
  if (!manager || !name) {
    log_error(bus_manager_tag, 
              "Bus removal failed", 
              "Manager or name pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  pstar_bus_config_t* to_remove = NULL;
  esp_err_t result = ESP_OK;

  /* Take mutex for thread safety */
  if (xSemaphoreTake(manager->mutex, portMAX_DELAY) != pdTRUE) {
    log_error(manager->tag, 
              "Bus removal failed", 
              "Failed to acquire mutex for bus removal");
    return ESP_ERR_TIMEOUT;
  }

  /* Special case: Removing the first bus */
  if (manager->buses && strcmp(manager->buses->name, name) == 0) {
    to_remove      = manager->buses;
    manager->buses = to_remove->next;
    to_remove->next = NULL; /* Break the link */
  } else {
    /* Search for the bus in the rest of the list */
    for (pstar_bus_config_t* curr = manager->buses; curr && curr->next; curr = curr->next) {
      if (strcmp(curr->next->name, name) == 0) {
        to_remove       = curr->next;
        curr->next      = to_remove->next;
        to_remove->next = NULL; /* Break the link */
        break;
      }
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
               to_remove->name,
               esp_err_to_name(result));
      /* Continue with destruction despite warning */
    }
  }
  
  /* Destroy the removed bus configuration */
  result = pstar_bus_config_destroy(to_remove);
  if (result != ESP_OK) {
    log_warn(manager->tag,
             "Bus destroy warning",
             "Failed to destroy bus configuration '%s' during removal: %s",
             name,
             esp_err_to_name(result));
    /* We've already removed it from the list, so return success anyway */
  }

  log_info(manager->tag, "Bus removed", "Removed and destroyed bus: %s", name);
  return ESP_OK;
}

esp_err_t pstar_bus_manager_deinit(pstar_bus_manager_t* manager)
{
  /* Validate input */
  if (!manager) {
    log_error(bus_manager_tag, 
              "Deinitialization failed", 
              "Bus manager pointer is NULL");
    return ESP_ERR_INVALID_ARG;
  }

  /* Take mutex for thread safety */
  if (manager->mutex != NULL) {
    if (xSemaphoreTake(manager->mutex, portMAX_DELAY) != pdTRUE) {
      log_error(manager->tag, 
                "Deinitialization warning", 
                "Failed to acquire mutex for cleanup - attempting anyway");
      /* Continue with cleanup even without mutex */
    }
  }

  /* Safety: Call deinit and destroy on each bus in the list */
  while (manager->buses) {
    pstar_bus_config_t* curr = manager->buses;
    manager->buses           = curr->next;

    /* First deinitialize the bus (release hardware resources) */
    esp_err_t result = pstar_bus_config_deinit(curr);
    if (result != ESP_OK) {
      log_warn(manager->tag,
               "Bus deinit warning",
               "Failed to deinitialize bus '%s': %s",
               curr->name,
               esp_err_to_name(result));
      /* Continue with cleanup despite the warning */
    }
    
    /* Then destroy the bus configuration (free memory) */
    result = pstar_bus_config_destroy(curr);
    if (result != ESP_OK) {
      log_warn(manager->tag,
               "Bus destroy warning",
               "Failed to destroy bus configuration: %s",
               esp_err_to_name(result));
      /* Continue with cleanup despite the warning */
    }
  }

  /* Free the tag string if it was allocated */
  if (manager->tag) {
    free((void*)manager->tag);
    manager->tag = NULL;
  }

  /* Delete mutex after use */
  if (manager->mutex != NULL) {
    vSemaphoreDelete(manager->mutex);
    manager->mutex = NULL;
  }

  log_info(bus_manager_tag, 
           "Bus manager deinitialized", 
           "Successfully cleaned up bus manager resources");
  return ESP_OK;
}