/* components/pstar_pin_validator/include/pstar_pin_validator.h */

#ifndef PSTAR_PIN_VALIDATOR_H
#define PSTAR_PIN_VALIDATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/gpio.h"

#include <stdbool.h>

#include "esp_err.h"

/* Constants ******************************************************************/

#define PIN_VALIDATOR_MAX_USAGE_DESC_LEN (64) /**< Maximum length of pin usage description */
#define PIN_VALIDATOR_MAX_PINS (GPIO_NUM_MAX) /**< Maximum number of pins that can be registered */
#define PIN_VALIDATOR_MAX_REGISTRATIONS (4)   /**< Maximum number of unique registrations per pin */

/**
 * @brief Error when the maximum number of registrations for a pin has been reached.
 */
#define ESP_ERR_PIN_VALIDATOR_MAX_REGISTRATIONS_REACHED (ESP_ERR_NO_MEM)

/* Structs ********************************************************************/

/**
 * @brief Information about a single registration for a GPIO pin
 */
typedef struct pin_registration {
  char component_name[PIN_VALIDATOR_MAX_USAGE_DESC_LEN]; /**< Name of component using the pin */
  char usage_desc[PIN_VALIDATOR_MAX_USAGE_DESC_LEN];     /**< Description of how the pin is used */
  int  count;     /**< Number of times this component registered the pin */
  bool can_share; /**< Whether this component allows sharing of the pin */
} pin_registration_t;

/**
 * @brief Information about a GPIO pin usage
 */
typedef struct pin_usage_info {
  bool in_use; /**< Whether the pin is in use */
  pin_registration_t
       registrations[PIN_VALIDATOR_MAX_REGISTRATIONS]; /**< Array of registrations for this pin */
  int  num_registrations;                              /**< Number of unique registrations */
  bool can_share; /**< Overall shareability of the pin (true if all registrations allow sharing) */
  int  usage_count; /**< Total usage count for the pin */
  char
    aggregated_component_name[PIN_VALIDATOR_MAX_USAGE_DESC_LEN]; /**< Aggregated component name */
  char aggregated_usage_desc[PIN_VALIDATOR_MAX_USAGE_DESC_LEN]; /**< Aggregated usage description */
} pin_usage_info_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initialize the pin validator system
 *
 * This function initializes the pin validator system, setting up the necessary
 * data structures and mutexes for thread-safe operation. It must be called
 * before any other pin validator functions.
 *
 * @return
 * - ESP_OK if successful
 * - ESP_ERR_NO_MEM if memory allocation for mutex fails
 */
esp_err_t pin_validator_init(void);

/**
 * @brief Register a pin usage with the validator
 *
 * This function registers a GPIO pin as being used by a specific component.
 * It checks for conflicts with existing pin assignments and updates the usage
 * information. If a pin is already registered by the same component, it simply
 * increments the usage count.
 *
 * @param[in] pin            GPIO pin number to register
 * @param[in] component_name Name of the component using this pin
 * @param[in] usage_desc     Description of how the pin is used
 * @param[in] can_share      Whether this pin can be shared with other components
 *
 * @return
 * - ESP_OK if successful
 * - ESP_ERR_INVALID_STATE if pin validator is not initialized
 * - ESP_ERR_INVALID_ARG if pin number is invalid or strings are NULL
 * - ESP_ERR_TIMEOUT if mutex cannot be taken
 * - ESP_ERR_PIN_VALIDATOR_MAX_REGISTRATIONS_REACHED if maximum registrations per pin is reached
 */
esp_err_t pin_validator_register_pin(gpio_num_t  pin,
                                     const char* component_name,
                                     const char* usage_desc,
                                     bool        can_share);

/**
 * @brief Register multiple pins at once with the validator
 *
 * This function registers multiple GPIO pins with the same component and usage
 * description. It calls pin_validator_register_pin for each pin in the array.
 * Pins with value -1 are skipped.
 *
 * @param[in] pins           Array of GPIO pin numbers to register
 * @param[in] num_pins       Number of pins in the array
 * @param[in] component_name Name of the component using these pins
 * @param[in] usage_desc     Description of how the pins are used
 * @param[in] can_share      Whether these pins can be shared with other components
 *
 * @return
 * - ESP_OK if all pins are registered successfully
 * - First error code encountered if any pin registration fails
 */
esp_err_t pin_validator_register_pins(const gpio_num_t* pins,
                                      size_t            num_pins,
                                      const char*       component_name,
                                      const char*       usage_desc,
                                      bool              can_share);

/**
 * @brief Validate all registered pins for conflicts
 *
 * This function checks all registered pins for conflicts, which occur when
 * multiple components use the same pin and at least one of them has marked
 * the pin as non-shareable.
 *
 * @param[in] halt_on_conflict If true, system will halt indefinitely if conflicts are found
 *
 * @return
 * - ESP_OK if no conflicts are found
 * - ESP_ERR_INVALID_STATE if conflicts are found or pin validator is not initialized
 * - ESP_ERR_TIMEOUT if mutex cannot be taken
 */
esp_err_t pin_validator_validate_all(bool halt_on_conflict);

/**
 * @brief Print all registered pin assignments
 *
 * This function prints a table of all registered pin assignments, including
 * the component name, usage description, shareability, and usage count.
 *
 * @return
 * - ESP_OK if successful
 * - ESP_ERR_INVALID_STATE if pin validator is not initialized
 * - ESP_ERR_TIMEOUT if mutex cannot be taken
 */
esp_err_t pin_validator_print_assignments(void);

/**
 * @brief Clear all registered pin assignments
 *
 * This function clears all registered pin assignments, effectively resetting
 * the pin validator to its initial state.
 *
 * @return
 * - ESP_OK if successful
 * - ESP_ERR_INVALID_STATE if pin validator is not initialized
 * - ESP_ERR_TIMEOUT if mutex cannot be taken
 */
esp_err_t pin_validator_clear_all(void);

/**
 * @brief Unregister a specific pin from the validator
 *
 * This function unregisters a pin for a specific component. If the pin is
 * used by multiple components or multiple times by the same component, the
 * usage count is decremented. The pin is fully unregistered only when the
 * usage count reaches zero.
 *
 * @param[in] pin            GPIO pin number to unregister
 * @param[in] component_name Component name that registered the pin
 *
 * @return
 * - ESP_OK if successfully unregistered
 * - ESP_ERR_INVALID_STATE if pin validator is not initialized
 * - ESP_ERR_INVALID_ARG if pin number is invalid or component_name is NULL
 * - ESP_ERR_NOT_FOUND if pin is not registered or not registered by this component
 * - ESP_ERR_TIMEOUT if mutex cannot be taken
 */
esp_err_t pin_validator_unregister_pin(gpio_num_t pin, const char* component_name);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_PIN_VALIDATOR_H */
