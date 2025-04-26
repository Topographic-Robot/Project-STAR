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

  /* Initialize SPI host tracking */
  for (int i = 0; i < SPI_HOST_MAX; ++i) {
    manager->spi_host_initialized[i] = false;
    manager->spi_device_count[i]     = 0; /* Initialize device count */
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
  ESP_RETURN_ON_FALSE(config->type > k_pstar_bus_type_none && config->type < k_pstar_bus_type_count,
                      ESP_ERR_INVALID_ARG,
                      manager->tag,
                      "Invalid bus type in config");
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

  /* Initialize the bus hardware *before* adding it to the list */
  /* This ensures the manager's SPI host tracking is updated correctly */
  if (!config->initialized) {
    /* Pass the manager to init for SPI host tracking */
    ret = pstar_bus_config_init(config, manager);
    if (ret != ESP_OK) {
      ESP_LOGE(manager->tag,
               "Failed to initialize bus hardware for '%s': %s",
               config->name,
               esp_err_to_name(ret));
      goto add_give_mutex; /* Don't add if init failed */
    }
  }

  /* Add to head of list */
  config->next   = manager->buses;
  manager->buses = config;

  ESP_LOGI(manager->tag,
           "Added bus config: %s (Type: %s)",
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

  pstar_bus_config_t* to_remove         = NULL;
  pstar_bus_config_t* prev              = NULL;
  esp_err_t           ret               = ESP_ERR_NOT_FOUND; /* Default to not found */
  spi_host_device_t   spi_host_to_check = -1; /* Track SPI host if removing an SPI device */
  bool                was_spi_device    = false;

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

      /* If it's an SPI device, note its host and decrement count */
      if (to_remove->type == k_pstar_bus_type_spi) {
        spi_host_to_check = to_remove->proto.spi.host;
        was_spi_device    = true;
        if (spi_host_to_check >= 0 && spi_host_to_check < SPI_HOST_MAX) {
          if (manager->spi_device_count[spi_host_to_check] > 0) {
            manager->spi_device_count[spi_host_to_check]--;
            ESP_LOGD(manager->tag,
                     "Decremented device count for SPI host %d to %d",
                     spi_host_to_check,
                     manager->spi_device_count[spi_host_to_check]);
          } else {
            ESP_LOGW(manager->tag,
                     "SPI device count for host %d was already 0 before decrementing!",
                     spi_host_to_check);
          }
        } else {
          ESP_LOGE(manager->tag,
                   "Invalid SPI host %d found in config '%s'",
                   spi_host_to_check,
                   name);
          spi_host_to_check = -1; /* Mark as invalid */
        }
      }
      break;
    }
  }

  /* Check if we need to free the SPI bus *after* unlinking and decrementing count */
  bool free_spi_bus = false;
  if (ret == ESP_OK && was_spi_device && spi_host_to_check != -1) {
    if (manager->spi_device_count[spi_host_to_check] == 0) {
      if (manager->spi_host_initialized[spi_host_to_check]) {
        free_spi_bus = true;
      } else {
        ESP_LOGW(manager->tag,
                 "Last device on SPI host %d removed, but host wasn't marked as initialized?",
                 spi_host_to_check);
      }
    }
  }

  /* Release mutex BEFORE potentially long deinit/destroy/free */
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
    /* We might still need to free the SPI bus below */
    ret = destroy_result;
  } else {
    ESP_LOGI(manager->tag, "Removed and destroyed bus config: %s", name);
  }

  /* Free the underlying SPI bus if necessary */
  if (free_spi_bus) {
    ESP_LOGI(manager->tag, "Freeing SPI bus driver for host %d...", spi_host_to_check);
    esp_err_t free_ret = spi_bus_free(spi_host_to_check);
    if (free_ret != ESP_OK) {
      ESP_LOGE(manager->tag,
               "Failed to free SPI bus for host %d: %s",
               spi_host_to_check,
               esp_err_to_name(free_ret));
      if (ret == ESP_OK) { /* Only overwrite if previous steps were successful */
        ret = free_ret;
      }
    } else {
      /* Update manager's tracking state - requires mutex again */
      if (xSemaphoreTake(manager->mutex,
                         pdMS_TO_TICKS(CONFIG_PSTAR_KCONFIG_BUS_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        manager->spi_host_initialized[spi_host_to_check] = false;
        /* Device count is already 0 */
        xSemaphoreGive(manager->mutex);
        ESP_LOGI(manager->tag, "SPI bus driver for host %d freed.", spi_host_to_check);
      } else {
        ESP_LOGE(manager->tag,
                 "Timeout acquiring mutex to update SPI host %d freed status.",
                 spi_host_to_check);
        if (ret == ESP_OK) {
          ret = ESP_ERR_TIMEOUT;
        }
      }
    }
  }

  return ret;
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

  /* Free underlying SPI buses that might still be marked as initialized */
  for (int host = 0; host < SPI_HOST_MAX; ++host) {
    if (manager->spi_host_initialized[host]) {
      ESP_LOGI(manager->tag, "Freeing potentially orphaned SPI bus driver for host %d...", host);
      esp_err_t free_ret = spi_bus_free((spi_host_device_t)host);
      if (free_ret != ESP_OK) {
        ESP_LOGE(manager->tag,
                 "Failed to free SPI bus for host %d during manager deinit: %s",
                 host,
                 esp_err_to_name(free_ret));
        if (first_error == ESP_OK) {
          first_error = free_ret;
        }
      }
      manager->spi_host_initialized[host] = false;
      manager->spi_device_count[host]     = 0;
    }
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
                                              void*                       user_ctx,
                                              bool                        can_be_shared)
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

    switch (curr->type) {
      case k_pstar_bus_type_i2c:
        reg_result = register_pin_if_valid(curr->proto.i2c.config.sda_io_num,
                                           bus_name,
                                           "I2C SDA",
                                           reg_func,
                                           user_ctx,
                                           true); /* I2C pins are shareable */
        if (reg_result != ESP_OK && ret == ESP_OK)
          ret = reg_result;

        reg_result = register_pin_if_valid(curr->proto.i2c.config.scl_io_num,
                                           bus_name,
                                           "I2C SCL",
                                           reg_func,
                                           user_ctx,
                                           true); /* I2C pins are shareable */
        if (reg_result != ESP_OK && ret == ESP_OK)
          ret = reg_result;
        break;

      case k_pstar_bus_type_spi:
        /* SPI bus pins (POSI, PISO, SCLK) are shared per host */
        /* Use the renamed internal fields for registration */
        reg_result = register_pin_if_valid(curr->proto.spi.posi_io_num,
                                           bus_name,
                                           "SPI POSI", /* Updated description */
                                           reg_func,
                                           user_ctx,
                                           true);
        if (reg_result != ESP_OK && ret == ESP_OK)
          ret = reg_result;

        reg_result = register_pin_if_valid(curr->proto.spi.piso_io_num,
                                           bus_name,
                                           "SPI PISO", /* Updated description */
                                           reg_func,
                                           user_ctx,
                                           true);
        if (reg_result != ESP_OK && ret == ESP_OK)
          ret = reg_result;

        reg_result = register_pin_if_valid(curr->proto.spi.sclk_io_num,
                                           bus_name,
                                           "SPI SCLK",
                                           reg_func,
                                           user_ctx,
                                           true);
        if (reg_result != ESP_OK && ret == ESP_OK)
          ret = reg_result;

        /* SPI CS pin is device-specific and NOT shared */
        reg_result = register_pin_if_valid(curr->proto.spi.dev_cfg.spics_io_num,
                                           bus_name,
                                           "SPI CS",
                                           reg_func,
                                           user_ctx,
                                           false);
        if (reg_result != ESP_OK && ret == ESP_OK)
          ret = reg_result;

        /* Register DC pin if used (often specific to device, not shared) */
        /* Assuming DC pin info might be stored in user context or flags if needed */
        /* For now, we don't have DC pin explicitly in config, HAL registers it */
        break;

      default:
        ESP_LOGW(manager->tag,
                 "Skipping pin registration for unsupported bus type: %d",
                 curr->type);
        break;
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