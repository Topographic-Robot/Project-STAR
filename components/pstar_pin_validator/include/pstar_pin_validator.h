/* components/pstar_pin_validator/include/pstar_pin_validator.h */

#ifndef PSTAR_PIN_VALIDATOR_H
#define PSTAR_PIN_VALIDATOR_H

#include "driver/gpio.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h" // Include for standard ESP error codes

/* Kconfig Defines */
#include "sdkconfig.h"

/* Constants ******************************************************************/

#ifndef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_MAX_PINS
#define PIN_VALIDATOR_MAX_PINS GPIO_NUM_MAX
#else
#define PIN_VALIDATOR_MAX_PINS CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_MAX_PINS
#endif

#ifndef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_MAX_REGISTRATIONS
#define PIN_VALIDATOR_MAX_REGISTRATIONS 5
#else
#define PIN_VALIDATOR_MAX_REGISTRATIONS CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_MAX_REGISTRATIONS
#endif

#ifndef CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_MAX_USAGE_DESC_LEN
#define PIN_VALIDATOR_MAX_USAGE_DESC_LEN 64
#else
#define PIN_VALIDATOR_MAX_USAGE_DESC_LEN CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_MAX_USAGE_DESC_LEN
#endif

/* Custom Error Codes */
// --- CORRECTED: Define a component-specific base ---
#define ESP_ERR_PSTAR_PIN_VALIDATOR_BASE (0x6000) /* Starting base for this component's errors */
// --- Use the new base for specific errors ---
#define ESP_ERR_PSTAR_PIN_VALIDATOR_CONFLICT                                                       \
  (ESP_ERR_PSTAR_PIN_VALIDATOR_BASE + 1) /* Pin conflict detected */
#define ESP_ERR_PSTAR_PIN_VALIDATOR_MAX_REGISTRATIONS_REACHED                                      \
  (ESP_ERR_PSTAR_PIN_VALIDATOR_BASE + 2) /* Too many registrations for a single pin */

/* Structures *****************************************************************/

/**
 * @brief Structure to hold information about a single registration for a pin.
 */
typedef struct {
  char component_name[PIN_VALIDATOR_MAX_USAGE_DESC_LEN]; /**< Name of the component using the pin */
  char usage_desc[PIN_VALIDATOR_MAX_USAGE_DESC_LEN];     /**< Description of the pin's usage */
  uint32_t count;     /**< How many times this specific component/usage combo was registered */
  bool     can_share; /**< Can this specific usage share the pin? */
} pin_registration_t;

/**
 * @brief Structure to hold overall usage information for a single GPIO pin.
 */
typedef struct {
  bool     in_use;      /**< Is this pin currently registered by any component? */
  bool     can_share;   /**< Overall shareability (true only if ALL registrations allow sharing) */
  uint32_t usage_count; /**< Total number of times this pin has been registered (across all
                           components/usages) */
  uint32_t num_registrations; /**< Number of unique component/usage registrations */
  pin_registration_t
       registrations[PIN_VALIDATOR_MAX_REGISTRATIONS]; /**< Array of individual registrations */
  char aggregated_component_name
    [PIN_VALIDATOR_MAX_USAGE_DESC_LEN]; /**< Aggregated string of component names */
  char aggregated_usage_desc
    [PIN_VALIDATOR_MAX_USAGE_DESC_LEN]; /**< Aggregated string of usage descriptions */
} pin_usage_info_t;

/* Public Function Prototypes *************************************************/

/**
 * @brief Initializes the pin validator module.
 *
 * Sets up the internal registry and mutex. Must be called before any other
 * pin validator functions.
 *
 * @return esp_err_t ESP_OK on success, ESP_ERR_NO_MEM if mutex creation fails.
 */
esp_err_t pin_validator_init(void);

/**
 * @brief Cleans up the pin validator resources.
 *
 * Deletes the mutex and resets the internal state. Should be called during
 * application shutdown or deinitialization.
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pin_validator_cleanup(void);

/**
 * @brief Registers a single GPIO pin usage.
 *
 * Records that a specific component is using a pin for a particular purpose.
 * Checks for conflicts based on the 'can_share' flag.
 *
 * @param pin GPIO number to register.
 * @param component_name Name of the component registering the pin.
 * @param usage_desc Description of how the pin is being used.
 * @param can_share True if this specific usage allows the pin to be shared
 *                  with other components/usages that also allow sharing.
 *                  False if this usage requires exclusive access.
 * @return esp_err_t
 *         - ESP_OK on success.
 *         - ESP_ERR_INVALID_ARG if pin number or descriptions are invalid.
 *         - ESP_ERR_INVALID_STATE if the validator is not initialized.
 *         - ESP_ERR_TIMEOUT if mutex cannot be acquired.
 *         - ESP_ERR_PSTAR_PIN_VALIDATOR_MAX_REGISTRATIONS_REACHED if the pin already has the
 * maximum allowed unique registrations.
 */
esp_err_t pin_validator_register_pin(gpio_num_t  pin,
                                     const char* component_name,
                                     const char* usage_desc,
                                     bool        can_share);

/**
 * @brief Registers multiple GPIO pins for the same component and usage.
 *
 * Convenience function to register an array of pins. Pins with value -1
 * in the array are skipped.
 *
 * @param pins Array of GPIO numbers to register.
 * @param num_pins Number of pins in the array.
 * @param component_name Name of the component registering the pins.
 * @param usage_desc Description of how the pins are being used.
 * @param can_share True if this usage allows sharing, False otherwise.
 * @return esp_err_t ESP_OK if all valid pins were registered successfully,
 *         or the first error code encountered during registration.
 */
esp_err_t pin_validator_register_pins(const gpio_num_t* pins,
                                      size_t            num_pins,
                                      const char*       component_name,
                                      const char*       usage_desc,
                                      bool              can_share);

/**
 * @brief Unregisters a component's usage of a specific pin.
 *
 * Decrements the usage count for the given component on the specified pin.
 * If the count reaches zero for that component/usage combination, the
 * registration entry is removed. If it's the last registration for the pin,
 * the pin is marked as completely unused.
 *
 * @param pin GPIO number to unregister.
 * @param component_name Name of the component unregistering the pin.
 * @return esp_err_t
 *         - ESP_OK on success.
 *         - ESP_ERR_INVALID_ARG if pin number or component name is invalid.
 *         - ESP_ERR_INVALID_STATE if the validator is not initialized.
 *         - ESP_ERR_TIMEOUT if mutex cannot be acquired.
 *         - ESP_ERR_NOT_FOUND if the pin was not registered or not registered
 *           by the specified component.
 */
esp_err_t pin_validator_unregister_pin(gpio_num_t pin, const char* component_name);

/**
 * @brief Validates all registered pin assignments for conflicts.
 *
 * Checks if any pin marked as non-shareable by any component is being used
 * by more than one component or for more than one purpose. Logs errors
 * for any conflicts found.
 *
 * @param halt_on_conflict If true, the function will enter an infinite loop
 *                         (effectively halting the system) upon detecting the
 *                         first conflict. If false, it logs all conflicts and
 *                         returns an error code if any conflicts were found.
 * @return esp_err_t
 *         - ESP_OK if no conflicts are found.
 *         - ESP_ERR_PSTAR_PIN_VALIDATOR_CONFLICT if conflicts are found.
 *         - ESP_ERR_TIMEOUT if mutex cannot be acquired.
 *         - ESP_ERR_INVALID_STATE if validator not initialized.
 */
esp_err_t pin_validator_validate_all(bool halt_on_conflict);

/**
 * @brief Prints a formatted table of all registered pin assignments to the console.
 *
 * @return esp_err_t ESP_OK on success, ESP_ERR_NO_MEM if buffer allocation fails,
 *         ESP_ERR_TIMEOUT if mutex cannot be acquired.
 */
esp_err_t pin_validator_print_assignments(void);

/**
 * @brief Clears all pin registrations from the validator.
 *
 * Resets the internal registry to an empty state.
 *
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if mutex cannot be acquired.
 */
esp_err_t pin_validator_clear_all(void);

#endif /* PSTAR_PIN_VALIDATOR_H */
