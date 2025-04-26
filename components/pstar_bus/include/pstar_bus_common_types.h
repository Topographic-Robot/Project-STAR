/* components/pstar_bus/include/pstar_bus_common_types.h */

#ifndef PSTAR_COMPONENT_BUS_COMMON_TYPES_H
#define PSTAR_COMPONENT_BUS_COMMON_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/gpio.h"
#include "driver/spi_master.h" /* Added for spi_host_device_t */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* --- Forward Declarations --- */

typedef struct pstar_bus_config  pstar_bus_config_t;
typedef struct pstar_bus_event   pstar_bus_event_t;
typedef struct pstar_bus_manager pstar_bus_manager_t;

/* --- Enums --- */

/**
 * @brief Types of supported bus interfaces.
 */
typedef enum {
  k_pstar_bus_type_none, /**< No bus type specified */
  k_pstar_bus_type_i2c,  /**< I2C bus */
  k_pstar_bus_type_spi,  /**< SPI bus */
  /* Add other types like UART, CAN if needed */
  k_pstar_bus_type_count, /**< Number of bus types */
} pstar_bus_type_t;

/* --- Callback Function Types --- */

/**
 * @brief Callback function type for pin registration.
 *
 * @param[in] pin_num   The GPIO pin number being registered.
 * @param[in] bus_name  The name of the bus configuration using this pin.
 * @param[in] pin_usage A descriptive string of how the pin is used (e.g., "I2C SDA", "SPI MOSI").
 * @param[in] user_ctx  User context pointer provided to pstar_bus_manager_register_all_pins.
 * @return esp_err_t Should return ESP_OK on successful registration, or an error code otherwise.
 */
typedef esp_err_t (*pin_registration_function_t)(gpio_num_t  pin_num,
                                                 const char* bus_name,
                                                 const char* pin_usage,
                                                 void*       user_ctx);

/* --- Public Functions --- */

/**
 * @brief Convert bus type to string.
 *
 * @param[in] type Bus type.
 * @return const char* String representation of the bus type. Returns "UNKNOWN" for invalid types.
 */
const char* pstar_bus_type_to_string(pstar_bus_type_t type);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_BUS_COMMON_TYPES_H */
