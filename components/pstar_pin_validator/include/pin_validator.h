/* components/pstar_pin_validator/include/pin_validator.h */

#ifndef PSTAR_PIN_VALIDATOR_H
#define PSTAR_PIN_VALIDATOR_H

#include "esp_err.h"
#include "driver/gpio.h"
#include <stdbool.h>

#define PIN_VALIDATOR_MAX_USAGE_DESC_LEN (64)           /**< Maximum length of pin usage description */
#define PIN_VALIDATOR_MAX_PINS           (GPIO_NUM_MAX) /**< Maximum number of pins that can be registered */

/* Pin usage information structure */
typedef struct pin_usage_info {
  bool in_use;
  char component_name[PIN_VALIDATOR_MAX_USAGE_DESC_LEN];
  char usage_desc[PIN_VALIDATOR_MAX_USAGE_DESC_LEN];
  bool can_share;
  int  usage_count;
} pin_usage_info_t;

/**
 * @brief Initialize the pin validator system
 * 
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t pin_validator_init(void);

/**
 * @brief Register a pin usage with the validator
 * 
 * @param pin            GPIO pin number
 * @param component_name Name of the component using this pin
 * @param usage_desc     Description of how the pin is used
 * @param can_share      Whether this pin can be shared with other components
 * @return ESP_OK if successfully registered, error code otherwise
 */
esp_err_t pin_validator_register_pin(gpio_num_t  pin, 
                                     const char* component_name,
                                     const char* usage_desc,
                                     bool        can_share);

/**
 * @brief Register multiple pins at once with the validator
 * 
 * @param pins           Array of GPIO pin numbers
 * @param num_pins       Number of pins in the array
 * @param component_name Name of the component using these pins
 * @param usage_desc     Description of how the pins are used
 * @param can_share      Whether these pins can be shared with other components
 * @return ESP_OK if all pins successfully registered, error code otherwise
 */
esp_err_t pin_validator_register_pins(const gpio_num_t* pins,
                                      size_t            num_pins,
                                      const char*       component_name,
                                      const char*       usage_desc,
                                      bool              can_share);

/**
 * @brief Validate all registered pins for conflicts
 * 
 * @param halt_on_conflict If true, system will halt on conflicts
 * @return ESP_OK if no conflicts, ESP_ERR_INVALID_STATE if conflicts found
 */
esp_err_t pin_validator_validate_all(bool halt_on_conflict);

/**
 * @brief Print all registered pin assignments
 * 
 * @return ESP_OK if successful
 */
esp_err_t pin_validator_print_assignments(void);

/**
 * @brief Clear all registered pin assignments
 * 
 * @return ESP_OK if successful
 */
esp_err_t pin_validator_clear_all(void);

/**
 * @brief Unregister a specific pin from the validator
 * 
 * @param pin            GPIO pin number to unregister
 * @param component_name Component name that registered the pin
 * @return ESP_OK if successfully unregistered, error code otherwise
 */
esp_err_t pin_validator_unregister_pin(gpio_num_t  pin, 
                                       const char* component_name);

#endif /* PSTAR_PIN_VALIDATOR_H */
