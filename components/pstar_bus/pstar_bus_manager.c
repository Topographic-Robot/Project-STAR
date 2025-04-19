/* components/pstar_bus/pstar_bus_manager.c */

#include "pstar_bus_manager.h"

#include "pstar_bus_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "sdkconfig.h" /* For Kconfig values like timeout */

/* --- Constants --- */

static const char* TAG = "Bus Manager";

/* --- Public Functions --- */

esp_err_t pstar_bus_manager_init(pstar_bus_manager_t* manager, const char* tag)
{
  ESP_RETURN_ON_FALSE(manager, ESP_ERR_INVALID_ARG, TAG, "Manager pointer is NULL");

  manager->buses = NULL;

  /* Set logging tag */
  if (tag && strlen(tag) > 0) {
    /* Use strdup for safety, requires free later */
    /* Cast needed because manager->tag is const char* but strdup returns char* */
    manager->tag = strdup(tag);
    ESP_RETURN_ON_FALSE(manager->tag,
                        ESP_ERR_NO_MEM,
                        TAG,
                        "Failed to allocate memory for manager tag");
  } else {
    manager->tag = TAG; /* Use the component's default static tag */
  }

  manager->mutex = xSemaphoreCreateMutex();
  if (!manager->mutex) {
    ESP_LOGE(manager->tag, "Failed to create mutex");
    /* Free tag only if it was dynamically allocated */
    if (manager->tag != TAG) {
      free((void*)manager->tag); /* Cast needed for free */
      manager->tag = NULL;
    }
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(manager->tag, "Initialized");
  return ESP_OK;
}

esp_err_t pstar_bus_manager_add_bus(pstar_bus_manager_t* manager, pstar_bus_config_t* config)
{
  ESP_RETURN_ON_FALSE(manager && config,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Manager or config pointer is NULL");
  ESP_RETURN_ON_FALSE(config->name && strlen(config->name) > 0,
                      ESP_ERR_INVALID_ARG,
                      manager->tag,
                      "Config name is NULL or empty");
  ESP_RETURN_ON_FALSE(config->type == k_pstar_bus_type_i2c,
                      ESP_ERR_INVALID_ARG,
                      manager->tag,
                      "Only I2C bus type supported currently");
  ESP_RETURN_ON_FALSE(manager->mutex,
                      ESP_ERR_INVALID_STATE,
                      manager->tag,
                      "Manager mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(manager->mutex, pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_BUS_MUTEX_TIMEOUT_MS)) !=
      pdTRUE) {
    ESP_LOGE(manager->tag, "Timeout acquiring mutex for adding bus '%s'", config->name);
    return ESP_ERR_TIMEOUT;
  }

  /* Check for duplicate name */
  for (pstar_bus_config_t* curr = manager->buses; curr; curr = curr->next) {
    if (curr->name && strcmp(curr->name, config->name) == 0) {
      ESP_LOGE(manager->tag, "Bus with name '%s' already exists", config->name);
      ret = ESP_ERR_INVALID_STATE;
      goto add_give_mutex; /* Use goto for single exit point with mutex give */
    }
  }

  /* Add to head of list */
  config->next   = manager->buses;
  manager->buses = config;

  ESP_LOGI(manager->tag,
           "Added bus: %s (Type: %s)",
           config->name,
           pstar_bus_type_to_string(config->type));

add_give_mutex:
  xSemaphoreGive(manager->mutex);
  return ret;
}

pstar_bus_config_t* pstar_bus_manager_find_bus(const pstar_bus_manager_t* manager, const char* name)
{
  /* Allow NULL manager or name, just return NULL without logging error */
  if (!manager || !name || strlen(name) == 0 || !manager->mutex) {
    return NULL;
  }

  pstar_bus_config_t* found_bus = NULL;

  if (xSemaphoreTake(manager->mutex, pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_BUS_MUTEX_TIMEOUT_MS)) !=
      pdTRUE) {
    ESP_LOGE(manager->tag, "Timeout acquiring mutex for finding bus '%s'", name);
    return NULL; /* Return NULL on timeout */
  }

  for (pstar_bus_config_t* curr = manager->buses; curr; curr = curr->next) {
    if (curr->name && strcmp(curr->name, name) == 0) {
      found_bus = curr;
      break;
    }
  }

  xSemaphoreGive(manager->mutex);
  return found_bus;
}

esp_err_t pstar_bus_manager_remove_bus(pstar_bus_manager_t* manager, const char* name)
{
  ESP_RETURN_ON_FALSE(manager && name && strlen(name) > 0,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Manager is NULL or name is invalid");
  ESP_RETURN_ON_FALSE(manager->mutex,
                      ESP_ERR_INVALID_STATE,
                      manager->tag,
                      "Manager mutex not initialized");

  pstar_bus_config_t* to_remove = NULL;
  pstar_bus_config_t* prev      = NULL;
  esp_err_t           ret       = ESP_ERR_NOT_FOUND; /* Default to not found */

  if (xSemaphoreTake(manager->mutex, pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_BUS_MUTEX_TIMEOUT_MS)) !=
      pdTRUE) {
    ESP_LOGE(manager->tag, "Timeout acquiring mutex for removing bus '%s'", name);
    return ESP_ERR_TIMEOUT;
  }

  /* Find and unlink the bus config */
  for (pstar_bus_config_t* curr = manager->buses; curr; prev = curr, curr = curr->next) {
    if (curr->name && strcmp(curr->name, name) == 0) {
      to_remove = curr;
      if (prev == NULL) /* Head of list */
      {
        manager->buses = curr->next;
      } else {
        prev->next = curr->next;
      }
      to_remove->next = NULL;   /* Unlink completely */
      ret             = ESP_OK; /* Found it */
      break;
    }
  }

  /* Release mutex BEFORE potentially long deinit/destroy */
  xSemaphoreGive(manager->mutex);

  if (ret == ESP_ERR_NOT_FOUND) {
    ESP_LOGW(manager->tag, "Bus '%s' not found for removal", name);
    return ESP_ERR_NOT_FOUND;
  }

  /* Now destroy the unlinked config (this calls deinit internally if needed) */
  esp_err_t destroy_result = pstar_bus_config_destroy(to_remove); /* to_remove is valid here */
  /* to_remove pointer is invalid after this call */

  if (destroy_result != ESP_OK) {
    ESP_LOGE(manager->tag,
             "Error destroying bus config '%s' during removal: %s",
             name,
             esp_err_to_name(destroy_result));
    /* Return the destroy error as it's more critical than just finding it */
    return destroy_result;
  }

  ESP_LOGI(manager->tag, "Removed and destroyed bus: %s", name);
  return ESP_OK;
}

esp_err_t pstar_bus_manager_deinit(pstar_bus_manager_t* manager)
{
  ESP_RETURN_ON_FALSE(manager, ESP_ERR_INVALID_ARG, TAG, "Manager pointer is NULL");

  bool mutex_taken = false;
  if (manager->mutex != NULL) {
    /* Use a slightly longer timeout for full cleanup */
    if (xSemaphoreTake(manager->mutex,
                       pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_BUS_MUTEX_TIMEOUT_MS * 2)) == pdTRUE) {
      mutex_taken = true;
    } else {
      ESP_LOGE(manager->tag, "Timeout acquiring mutex for deinit - cleanup may be incomplete!");
      /* Continue cautiously without mutex */
    }
  } else {
    ESP_LOGW(manager->tag, "Manager mutex was NULL during deinit");
  }

  /* Detach list head immediately while holding mutex (if taken) */
  pstar_bus_config_t* curr = manager->buses;
  manager->buses           = NULL;

  /* Release mutex before iterating and destroying */
  if (mutex_taken) {
    xSemaphoreGive(manager->mutex);
    mutex_taken = false;
  }

  esp_err_t first_error = ESP_OK;

  ESP_LOGI(manager->tag, "Deinitializing all managed buses...");
  while (curr) {
    pstar_bus_config_t* next = curr->next; /* Store next pointer before destroying curr */
    const char* current_name = curr->name ? curr->name : "UNKNOWN"; /* Save name for logging */

    /* Destroy calls deinit internally */
    esp_err_t destroy_result = pstar_bus_config_destroy(curr); /* curr is valid here */
    if (destroy_result != ESP_OK) {
      ESP_LOGE(manager->tag,
               "Error destroying bus '%s' during manager deinit: %s",
               current_name,
               esp_err_to_name(destroy_result));
      if (first_error == ESP_OK) {
        first_error = destroy_result; /* Record the first error encountered */
      }
    }
    curr = next; /* Move to the next node (original curr pointer is now invalid) */
  }

  /* Free the tag string if it was dynamically allocated */
  if (manager->tag != NULL && manager->tag != TAG) {
    free((void*)manager->tag); /* Cast needed for free */
  }
  manager->tag = NULL;

  /* Delete mutex */
  if (manager->mutex != NULL) {
    vSemaphoreDelete(manager->mutex);
    manager->mutex = NULL;
  }

  ESP_LOGI(TAG,
           "Manager deinitialized %s",
           (first_error == ESP_OK) ? "successfully" : "with errors");
  return first_error;
}

/* --- Pin Registration --- */

/* Helper function to register a single pin if valid */
static inline esp_err_t register_pin_if_valid(gpio_num_t                  pin,
                                              const char*                 bus_name,
                                              const char*                 usage,
                                              pin_registration_function_t reg_func,
                                              void*                       user_ctx)
{
  if (pin >= 0 && pin < GPIO_NUM_MAX) /* Check if pin number is valid */
  {
    return reg_func(pin, bus_name, usage, user_ctx);
  }
  return ESP_OK; /* Not an error if pin is not used (-1) or invalid */
}

esp_err_t pstar_bus_manager_register_all_pins(const pstar_bus_manager_t*  manager,
                                              pin_registration_function_t reg_func,
                                              void*                       user_ctx)
{
  ESP_RETURN_ON_FALSE(manager && reg_func,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Manager or registration function is NULL");
  ESP_RETURN_ON_FALSE(manager->mutex,
                      ESP_ERR_INVALID_STATE,
                      manager->tag,
                      "Manager mutex not initialized");

  esp_err_t ret        = ESP_OK;
  esp_err_t reg_result = ESP_OK; /* Track registration errors separately */

  if (xSemaphoreTake(manager->mutex, pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_BUS_MUTEX_TIMEOUT_MS)) !=
      pdTRUE) {
    ESP_LOGE(manager->tag, "Timeout acquiring mutex for pin registration");
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGI(manager->tag, "Registering pins for all managed buses...");

  for (pstar_bus_config_t* curr = manager->buses; curr; curr = curr->next) {
    const char* bus_name = curr->name ? curr->name : "UNKNOWN";
    ESP_LOGD(manager->tag,
             "Processing pins for bus '%s' (Type: %s)",
             bus_name,
             pstar_bus_type_to_string(curr->type));

    /* Only handle I2C pins */
    if (curr->type == k_pstar_bus_type_i2c) {
      reg_result =
        register_pin_if_valid(curr->i2c.config.sda_io_num, bus_name, "I2C SDA", reg_func, user_ctx);
      if (reg_result != ESP_OK && ret == ESP_OK) {
        ret = reg_result;
      } /* Record first error */

      reg_result =
        register_pin_if_valid(curr->i2c.config.scl_io_num, bus_name, "I2C SCL", reg_func, user_ctx);
      if (reg_result != ESP_OK && ret == ESP_OK) {
        ret = reg_result;
      }
    } else {
      ESP_LOGW(manager->tag, "Skipping non-I2C bus '%s' during pin registration", bus_name);
    }

    /* Log first error encountered during registration for this bus */
    if (ret != ESP_OK && reg_result != ESP_OK) {
      ESP_LOGE(manager->tag,
               "Pin registration failed for bus '%s' (first error: %s)",
               bus_name,
               esp_err_to_name(ret));
    }
  }

  xSemaphoreGive(manager->mutex);

  if (ret != ESP_OK) {
    ESP_LOGE(manager->tag,
             "Pin registration process completed with errors (first error: %s)",
             esp_err_to_name(ret));
  } else {
    ESP_LOGI(manager->tag, "Pin registration process completed.");
  }

  /* Return the first error encountered during registration calls */
  return ret;
}
