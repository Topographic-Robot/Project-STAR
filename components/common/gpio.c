/* components/common/gpio.c */

#include "common/gpio.h"

/* Private Functions **********************************************************/

esp_err_t priv_gpio_init(uint8_t data_io) 
{
  /* Define the GPIO configuration for the DHT22 data line */
  gpio_config_t io_conf;
  io_conf.pin_bit_mask = (1ULL << data_io);     /* Select the pin */
  io_conf.mode         = GPIO_MODE_OUTPUT;      /* Set as output mode */
  io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;    /* Enable pull-up */
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; /* Disable pull-down */
  io_conf.intr_type    = GPIO_INTR_DISABLE;     /* No interrupt */

  /* Apply the configuration */
  return gpio_config(&io_conf);
}
