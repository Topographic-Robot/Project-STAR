#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "pca9685.h"

#define SCL_IO_PIN GPIO_NUM_22
#define SDA_IO_PIN GPIO_NUM_21

void app_main(void) {
  /* Configure master */
  i2c_master_bus_config_t i2c_bus_config = {
      .clk_source        = I2C_CLK_SRC_DEFAULT,
      .i2c_port          = -1, /* -1 for auto selection */
      .scl_io_num        = SCL_IO_PIN,
      .sda_io_num        = SDA_IO_PIN,
      /* sets the glitch period of master bus, values less than this is
       * filtered, a typically value is 7 */
      .glitch_ignore_cnt = 7,
  };
  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

  /* Configure slave (pca9685) */
  i2c_pca9685_bus_config_t pca9685_config = {};
}
