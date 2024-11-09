/* components/common/include/common/gpio.h */

#ifndef TOPOROBO_GPIO_H
#define TOPOROBO_GPIO_H

#include <stdint.h>
#include <esp_err.h>
#include <driver/gpio.h>

/* Private Functions **********************************************************/

/**
 * This function configures the GPIO pin with the provided pin number,
 * mode, and pull-up configuration. It sets the GPIO mode to output and 
 * initializes the pin for communication with external devices like the DHT22.
 *
 * @param[in] data_io Pin number for the GPIO data line.
 *
 * @return
 *  - ESP_OK on successful initialization.
 *  - An error code from the esp_err_t enumeration on failure.
 *
 * @note The function enables internal pull-ups and disables pull-downs 
 * for the data pin. No interrupts are configured for this GPIO pin.
 */
esp_err_t priv_gpio_init(uint8_t data_io);

#endif /* TOPOROBO_GPIO_H */
