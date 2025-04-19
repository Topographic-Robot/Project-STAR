/* components/pstar_pin_validator/include/pstar_pin_validator.h */

#ifndef PSTAR_COMPONENT_PIN_VALIDATOR_H
#define PSTAR_COMPONENT_PIN_VALIDATOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* --- INCLUDES --- */

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "soc/gpio_num.h"

/* --- MACROS --- */

#define PIN_VALIDATOR_DESC_MAX_LEN (64)

/* --- STRUCTS --- */

/**
 * @brief Info for each GPIO pin.
 */
typedef struct {
  bool    can_be_shared; /**< Whether this pin can be shared. */
  uint8_t usage_count;   /**< Number of components using this pin. */
  char**  users;         /**< Dynamically allocated array of descriptions. */
} pstar_pin_info_t;

/**
 * @brief The main validator containing all pin usage information.
 */
typedef struct {
  pstar_pin_info_t  pins[GPIO_NUM_MAX]; /**< Every pin on the MCU */
  bool              initialized;        /**< Whether the validator has been initialized */
  SemaphoreHandle_t mutex;              /**< Mutex for thread safety */
} pstar_pin_validator_t;

/* --- FUNCTIONS --- */

/**
 * @brief Register a pin on the validator
 *
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t pstar_register_pin(gpio_num_t gpio_num, const char* desc, bool can_be_shared);

/**
 * @brief Validate pins
 *
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t pstar_validate_pins(void);

/**
 * @brief Free the pin validator
 *
 * @return ESP_OK if successful, otherwise an error code
 */
esp_err_t pstar_free_pin_validator(void);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_PIN_VALIDATOR_H */
