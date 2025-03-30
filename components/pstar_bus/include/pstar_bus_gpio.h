/* components/pstar_bus/include/pstar_bus_gpio.h */

#ifndef PSTAR_BUS_GPIO_H
#define PSTAR_BUS_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_common_types.h"

#include "driver/gpio.h"

#include <stdint.h>

#include "esp_err.h"

/* Public Functions ***********************************************************/

/**
 * @brief Initialize default GPIO operations.
 *
 * @param[out] ops Pointer to GPIO operations structure to initialize.
 */
void pstar_bus_gpio_init_default_ops(pstar_gpio_ops_t* ops);

/**
 * @brief Set GPIO pin level.
 *
 * @param[in] manager Pointer to the bus manager.
 * @param[in] name    Name of the GPIO bus.
 * @param[in] pin_num GPIO pin number.
 * @param[in] level   Level to set (0 or 1).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_gpio_set_level(const pstar_bus_manager_t* manager,
                                   const char*                name,
                                   gpio_num_t                 pin_num,
                                   uint32_t                   level);

/**
 * @brief Get GPIO pin level.
 *
 * @param[in]  manager Pointer to the bus manager.
 * @param[in]  name    Name of the GPIO bus.
 * @param[in]  pin_num GPIO pin number.
 * @param[out] level   Pointer to store the pin level.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_gpio_get_level(const pstar_bus_manager_t* manager,
                                   const char*                name,
                                   gpio_num_t                 pin_num,
                                   uint32_t*                  level);

/**
 * @brief Add an ISR handler for a GPIO pin.
 *
 * @param[in] manager     Pointer to the bus manager.
 * @param[in] name        Name of the GPIO bus.
 * @param[in] pin_num     GPIO pin number.
 * @param[in] isr_handler ISR handler function.
 * @param[in] args        Arguments to pass to the ISR handler.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t pstar_bus_gpio_isr_add(const pstar_bus_manager_t* manager,
                                 const char*                name,
                                 gpio_num_t                 pin_num,
                                 gpio_isr_t                 isr_handler,
                                 void*                      args);

/**
 * @brief Remove an ISR handler for a GPIO pin.
 *
 * @param[in] manager Pointer to the bus manager.
 * @param[in] name    Name of the GPIO bus.
 * @param[in] pin_num GPIO pin number.
 * @return esp_err_t ESP_OK on success, or an error code.
 */
esp_err_t
pstar_bus_gpio_isr_remove(const pstar_bus_manager_t* manager, const char* name, gpio_num_t pin_num);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_BUS_GPIO_H */
