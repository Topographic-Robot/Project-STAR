/* components/pstar_bus/bus_gpio.c */

#include "bus_manager.h"
#include "log_handler.h"
#include "bus_types.h"
#include "bus_event.h"
#include "bus_gpio.h"
#include <string.h>

/* Constants ******************************************************************/

#define BUS_GPIO_TAG ("Bus GPIO")

/* Private Function Prototypes ************************************************/

static esp_err_t priv_pstar_bus_gpio_set_level(const pstar_bus_manager_t* manager,
                                               const char*                name, 
                                               gpio_num_t                 pin_num, 
                                               uint32_t                   level);

static esp_err_t priv_pstar_bus_gpio_get_level(const pstar_bus_manager_t* manager,
                                               const char*                name, 
                                               gpio_num_t                 pin_num, 
                                               uint32_t*                  level);

static esp_err_t priv_pstar_bus_gpio_isr_add(const pstar_bus_manager_t* manager,
                                             const char*                name, 
                                             gpio_num_t                 pin_num, 
                                             gpio_isr_t                 isr_handler, 
                                             void*                      args);

static esp_err_t priv_pstar_bus_gpio_isr_remove(const pstar_bus_manager_t* manager,
                                                const char*                name, 
                                                gpio_num_t                 pin_num);

/* Public Functions ***********************************************************/

void pstar_bus_gpio_init_default_ops(pstar_gpio_ops_t* ops)
{
  if (ops == NULL) {
    log_error(BUS_GPIO_TAG, 
              "Init Error", 
              "GPIO operations pointer is NULL");
    return;
  }
  
  ops->set_level  = priv_pstar_bus_gpio_set_level;
  ops->get_level  = priv_pstar_bus_gpio_get_level;
  ops->isr_add    = priv_pstar_bus_gpio_isr_add;
  ops->isr_remove = priv_pstar_bus_gpio_isr_remove;
  
  log_info(BUS_GPIO_TAG, 
           "Default Ops", 
           "Initialized default GPIO operations");
}

esp_err_t pstar_bus_gpio_set_level(const pstar_bus_manager_t* manager,
                                   const char*                name,
                                   gpio_num_t                 pin_num,
                                   uint32_t                   level)
{
  /* Validate input */
  if (!manager || !name) {
    log_error(BUS_GPIO_TAG, 
              "Set Level Error", 
              "Invalid parameters: manager or name is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_GPIO_TAG, 
              "Set Level Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is a GPIO bus */
  if (bus_config->type != k_pstar_bus_type_gpio) {
    log_error(BUS_GPIO_TAG, 
              "Set Level Error", 
              "Bus '%s' is not a GPIO bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Call the set_level operation function */
  if (bus_config->config.gpio.ops.set_level) {
    return bus_config->config.gpio.ops.set_level(manager, name, pin_num, level);
  } else {
    log_error(BUS_GPIO_TAG, 
              "Set Level Error", 
              "No set_level operation defined for GPIO bus '%s'", 
              name);
    return ESP_ERR_NOT_SUPPORTED;
  }
}

esp_err_t pstar_bus_gpio_get_level(const pstar_bus_manager_t* manager,
                                   const char*                name,
                                   gpio_num_t                 pin_num,
                                   uint32_t*                  level)
{
  /* Validate input */
  if (!manager || !name || !level) {
    log_error(BUS_GPIO_TAG, 
              "Get Level Error", 
              "Invalid parameters: manager, name, or level is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_GPIO_TAG, 
              "Get Level Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is a GPIO bus */
  if (bus_config->type != k_pstar_bus_type_gpio) {
    log_error(BUS_GPIO_TAG, 
              "Get Level Error", 
              "Bus '%s' is not a GPIO bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Call the get_level operation function */
  if (bus_config->config.gpio.ops.get_level) {
    return bus_config->config.gpio.ops.get_level(manager, name, pin_num, level);
  } else {
    log_error(BUS_GPIO_TAG, 
              "Get Level Error", 
              "No get_level operation defined for GPIO bus '%s'", 
              name);
    return ESP_ERR_NOT_SUPPORTED;
  }
}

esp_err_t pstar_bus_gpio_isr_add(const pstar_bus_manager_t* manager,
                                 const char*                name,
                                 gpio_num_t                 pin_num,
                                 gpio_isr_t                 isr_handler,
                                 void*                      args)
{
  /* Validate input */
  if (!manager || !name || !isr_handler) {
    log_error(BUS_GPIO_TAG, 
              "ISR Add Error", 
              "Invalid parameters: manager, name, or isr_handler is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_GPIO_TAG, 
              "ISR Add Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is a GPIO bus */
  if (bus_config->type != k_pstar_bus_type_gpio) {
    log_error(BUS_GPIO_TAG, 
              "ISR Add Error", 
              "Bus '%s' is not a GPIO bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Call the isr_add operation function */
  if (bus_config->config.gpio.ops.isr_add) {
    return bus_config->config.gpio.ops.isr_add(manager, 
                                               name, 
                                               pin_num, 
                                               isr_handler, 
                                               args);
  } else {
    log_error(BUS_GPIO_TAG, 
              "ISR Add Error", 
              "No isr_add operation defined for GPIO bus '%s'", 
              name);
    return ESP_ERR_NOT_SUPPORTED;
  }
}

esp_err_t pstar_bus_gpio_isr_remove(const pstar_bus_manager_t* manager,
                                    const char*                name,
                                    gpio_num_t                 pin_num)
{
  /* Validate input */
  if (!manager || !name) {
    log_error(BUS_GPIO_TAG, 
              "ISR Remove Error", 
              "Invalid parameters: manager or name is NULL");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_GPIO_TAG, 
              "ISR Remove Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is a GPIO bus */
  if (bus_config->type != k_pstar_bus_type_gpio) {
    log_error(BUS_GPIO_TAG, 
              "ISR Remove Error", 
              "Bus '%s' is not a GPIO bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Call the isr_remove operation function */
  if (bus_config->config.gpio.ops.isr_remove) {
    return bus_config->config.gpio.ops.isr_remove(manager, name, pin_num);
  } else {
    log_error(BUS_GPIO_TAG, 
              "ISR Remove Error", 
              "No isr_remove operation defined for GPIO bus '%s'", 
              name);
    return ESP_ERR_NOT_SUPPORTED;
  }
}

/* Private Functions **********************************************************/

/**
 * @brief Set GPIO pin level using the default implementation.
 * 
 * @param[in] manager Pointer to the bus manager.
 * @param[in] name    Name of the GPIO bus.
 * @param[in] pin_num GPIO pin number.
 * @param[in] level   Level to set (0 or 1).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
static esp_err_t priv_pstar_bus_gpio_set_level(const pstar_bus_manager_t* manager,
                                               const char*                name, 
                                               gpio_num_t                 pin_num, 
                                               uint32_t                   level)
{
  esp_err_t result = ESP_OK;
  
  /* Validate input */
  if (!manager || !name || pin_num < 0) {
    log_error(BUS_GPIO_TAG, 
              "Set Level Error", 
              "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_GPIO_TAG, 
              "Set Level Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is a GPIO bus */
  if (bus_config->type != k_pstar_bus_type_gpio) {
    log_error(BUS_GPIO_TAG, 
              "Set Level Error", 
              "Bus '%s' is not a GPIO bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(BUS_GPIO_TAG, 
              "Set Level Error", 
              "Bus '%s' is not initialized", 
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Check if the pin is configured (bit is set in pin_bit_mask) */
  uint64_t pin_mask = (1ULL << pin_num);
  if ((bus_config->config.gpio.config.pin_bit_mask & pin_mask) == 0) {
    log_error(BUS_GPIO_TAG, 
              "Set Level Error", 
              "Pin %d is not configured in bus '%s'", 
              pin_num, 
              name);
    return ESP_ERR_INVALID_ARG;
  }

  /* Set the GPIO level */
  result = gpio_set_level(pin_num, level);
  if (result != ESP_OK) {
    log_error(BUS_GPIO_TAG, 
              "Set Level Error", 
              "Failed to set level for pin %d on bus '%s': %s", 
              pin_num, 
              name, 
              esp_err_to_name(result));
    return result;
  }
  
  log_debug(BUS_GPIO_TAG, 
            "Set Level", 
            "Set pin %d to level %lu on bus '%s'", 
            pin_num, 
            level, 
            name);
  
  /* Call callback if available */
  if (bus_config->config.gpio.callbacks.on_level_change) {
    /* Create event structure */
    pstar_bus_event_t event = {
      .bus_type = k_pstar_bus_type_gpio,
      .bus_name = name,
      .data.gpio = {
        .pin_num = pin_num,
        .level   = level
      }
    };
    
    /* Call the callback */
    bus_config->config.gpio.callbacks.on_level_change(&event, 
                                                      level, 
                                                      bus_config->user_ctx);
  }
  
  return ESP_OK;
}

/**
 * @brief Get GPIO pin level using the default implementation.
 * 
 * @param[in]  manager Pointer to the bus manager.
 * @param[in]  name    Name of the GPIO bus.
 * @param[in]  pin_num GPIO pin number.
 * @param[out] level   Pointer to store the pin level.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
static esp_err_t priv_pstar_bus_gpio_get_level(const pstar_bus_manager_t* manager,
                                               const char*                name, 
                                               gpio_num_t                 pin_num, 
                                               uint32_t*                  level)
{
  /* Validate input */
  if (!manager || !name || pin_num < 0 || !level) {
    log_error(BUS_GPIO_TAG, 
              "Get Level Error", 
              "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_GPIO_TAG, 
              "Get Level Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is a GPIO bus */
  if (bus_config->type != k_pstar_bus_type_gpio) {
    log_error(BUS_GPIO_TAG, 
              "Get Level Error", 
              "Bus '%s' is not a GPIO bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(BUS_GPIO_TAG, 
              "Get Level Error", 
              "Bus '%s' is not initialized", 
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Check if the pin is configured (bit is set in pin_bit_mask) */
  uint64_t pin_mask = (1ULL << pin_num);
  if ((bus_config->config.gpio.config.pin_bit_mask & pin_mask) == 0) {
    log_error(BUS_GPIO_TAG, 
              "Get Level Error", 
              "Pin %d is not configured in bus '%s'", 
              pin_num, 
              name);
    return ESP_ERR_INVALID_ARG;
  }

  /* Get the GPIO level */
  int gpio_level = gpio_get_level(pin_num);
  if (gpio_level < 0) {
    log_error(BUS_GPIO_TAG, 
              "Get Level Error", 
              "Failed to get level for pin %d on bus '%s'", 
              pin_num, 
              name);
    return ESP_FAIL;
  }
  
  /* Set the output value */
  *level = (uint32_t)gpio_level;
  
  log_debug(BUS_GPIO_TAG, 
            "Get Level", 
            "Got level %lu from pin %d on bus '%s'", 
            *level, 
            pin_num, 
            name);
  
  return ESP_OK;
}

/**
 * @brief Add an ISR handler for a GPIO pin using the default implementation.
 * 
 * @param[in] manager     Pointer to the bus manager.
 * @param[in] name        Name of the GPIO bus.
 * @param[in] pin_num     GPIO pin number.
 * @param[in] isr_handler ISR handler function.
 * @param[in] args        Arguments to pass to the ISR handler.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
static esp_err_t priv_pstar_bus_gpio_isr_add(const pstar_bus_manager_t* manager,
                                             const char*                name, 
                                             gpio_num_t                 pin_num, 
                                             gpio_isr_t                 isr_handler, 
                                             void*                      args)
{
  esp_err_t result = ESP_OK;
  
  /* Validate input */
  if (!manager || !name || pin_num < 0 || !isr_handler) {
    log_error(BUS_GPIO_TAG, 
              "ISR Add Error", 
              "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_GPIO_TAG, 
              "ISR Add Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is a GPIO bus */
  if (bus_config->type != k_pstar_bus_type_gpio) {
    log_error(BUS_GPIO_TAG, 
              "ISR Add Error", 
              "Bus '%s' is not a GPIO bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(BUS_GPIO_TAG, 
              "ISR Add Error", 
              "Bus '%s' is not initialized", 
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Check if the pin is configured (bit is set in pin_bit_mask) */
  uint64_t pin_mask = (1ULL << pin_num);
  if ((bus_config->config.gpio.config.pin_bit_mask & pin_mask) == 0) {
    log_error(BUS_GPIO_TAG, 
              "ISR Add Error", 
              "Pin %d is not configured in bus '%s'", 
              pin_num, 
              name);
    return ESP_ERR_INVALID_ARG;
  }

  /* Check if GPIO interrupt service is not already installed 
   * Attempt to install the service. If it's already installed, we'll get ESP_ERR_INVALID_STATE
   * which we can safely ignore.
   */
  result = gpio_install_isr_service(0);
  if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
    /* ESP_ERR_INVALID_STATE means the service is already installed, which is OK */
    log_error(BUS_GPIO_TAG, 
              "ISR Service Error", 
              "Failed to install GPIO ISR service for bus '%s': %s", 
              name, 
              esp_err_to_name(result));
    return result;
  }

  /* Add ISR handler for this pin */
  result = gpio_isr_handler_add(pin_num, isr_handler, args);
  if (result != ESP_OK) {
    log_error(BUS_GPIO_TAG, 
              "ISR Add Error", 
              "Failed to add ISR handler for pin %d on bus '%s': %s", 
              pin_num, 
              name, 
              esp_err_to_name(result));
    return result;
  }
  
  log_info(BUS_GPIO_TAG, 
           "ISR Added", 
           "Added ISR handler for pin %d on bus '%s'", 
           pin_num, 
           name);
  
  return ESP_OK;
}

/**
 * @brief Remove an ISR handler for a GPIO pin using the default implementation.
 * 
 * @param[in] manager Pointer to the bus manager.
 * @param[in] name    Name of the GPIO bus.
 * @param[in] pin_num GPIO pin number.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
static esp_err_t priv_pstar_bus_gpio_isr_remove(const pstar_bus_manager_t* manager,
                                                const char*                name, 
                                                gpio_num_t                 pin_num)
{
  esp_err_t result = ESP_OK;
  
  /* Validate input */
  if (!manager || !name || pin_num < 0) {
    log_error(BUS_GPIO_TAG, 
              "ISR Remove Error", 
              "Invalid parameters");
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Find the bus configuration */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, name);
  if (!bus_config) {
    log_error(BUS_GPIO_TAG, 
              "ISR Remove Error", 
              "Bus '%s' not found", 
              name);
    return ESP_ERR_NOT_FOUND;
  }
  
  /* Check if this is a GPIO bus */
  if (bus_config->type != k_pstar_bus_type_gpio) {
    log_error(BUS_GPIO_TAG, 
              "ISR Remove Error", 
              "Bus '%s' is not a GPIO bus (type: %s)", 
              name, 
              pstar_bus_type_to_string(bus_config->type));
    return ESP_ERR_INVALID_ARG;
  }
  
  /* Check if the bus is initialized */
  if (!bus_config->initialized) {
    log_error(BUS_GPIO_TAG, 
              "ISR Remove Error", 
              "Bus '%s' is not initialized", 
              name);
    return ESP_ERR_INVALID_STATE;
  }
  
  /* Remove ISR handler for this pin */
  result = gpio_isr_handler_remove(pin_num);
  if (result != ESP_OK) {
    log_error(BUS_GPIO_TAG, 
              "ISR Remove Error", 
              "Failed to remove ISR handler for pin %d on bus '%s': %s", 
              pin_num, 
              name, 
              esp_err_to_name(result));
    return result;
  }
  
  log_info(BUS_GPIO_TAG, 
           "ISR Removed", 
           "Removed ISR handler for pin %d on bus '%s'", 
           pin_num, 
           name);
  
  return ESP_OK;
}
