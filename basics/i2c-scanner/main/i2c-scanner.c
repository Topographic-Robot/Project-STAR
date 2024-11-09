/* basics/i2c-scanner/main/i2c-scanner.c */

#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO         22        /* GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO         21        /* GPIO number for I2C master data */
#define I2C_MASTER_NUM            I2C_NUM_0 /* I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ        100000    /* I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0         /* I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0         /* I2C master doesn't need buffer */

static const char *TAG = "I2C Scanner";

static esp_err_t i2c_master_init(void) {
  i2c_config_t conf = {
      .mode             = I2C_MODE_MASTER,
      .sda_io_num       = I2C_MASTER_SDA_IO,
      .sda_pullup_en    = GPIO_PULLUP_ENABLE,
      .scl_io_num       = I2C_MASTER_SCL_IO,
      .scl_pullup_en    = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
  if (err != ESP_OK) {
    return err;
  }
  return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

void i2c_scanner() {
  printf("Scanning I2C bus...\n");
  /* XXX: Where is 127 from */
  for (int i = 1; i < 127; i++) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    /* XXX: Comment is needed, explain this */
    i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret =
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
      printf("Found I2C device at address 0x%02X\n", i);
    }
  }
  printf("I2C scan completed.\n");
}

void app_main(void) {
  ESP_ERROR_CHECK(i2c_master_init());
  i2c_scanner();
}
