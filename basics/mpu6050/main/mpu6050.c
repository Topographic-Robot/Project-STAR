#include "driver/i2c.h"
#include "esp_log.h"

/*
 * MPU-6050 Connections:
 * VCC -> 3.3V
 * GND -> GND
 * SCL -> GPIO 22
 * SDA -> GPIO 21
 * XDA -> Not Used
 * XCL -> Not Used
 * ADD -> GND (or VCC to change the I2C address to 0x69)
 * INT -> Not Used
 */

/* Uncomment the line below if ADD is connected to VCC */
/* #define MPU6050_SENSOR_ADDR 0x69 */

/* Uncomment the line below if ADD is connected to GND */
#define MPU6050_SENSOR_ADDR 0x68

#define I2C_MASTER_SCL_IO         22 /* GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO         21 /* GPIO number for I2C master data */
#define I2C_MASTER_NUM            I2C_NUM_1 /* I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ        100000    /* I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0         /* I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0         /* I2C master doesn't need buffer */
#define MPU6050_CMD_START         0x41      /* Command to start MPU6050 */
#define MPU6050_WHO_AM_I          0x75 /* Register to check MPU6050 identity */
#define MPU6050_PWR_MGMT_1_REG    0x6B /* Register to manage power mode */
#define MPU6050_ACCEL_XOUT_H      0x3B /* Register to read accel data */
#define MPU6050_GYRO_XOUT_H       0x43 /* Register to read gyro data */

static const char *TAG = "MPU6050";
#define NUM_READINGS 10

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

static esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);
  esp_err_t err =
      i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

static esp_err_t mpu6050_read(uint8_t reg_addr, uint8_t *data, size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  /* XXX: Explain this */
  i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_start(cmd);
  /* XXX: Explain this */
  i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t err =
      i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

static void mpu6050_init(void) {
  uint8_t who_am_i = 0;
  mpu6050_read(MPU6050_WHO_AM_I, &who_am_i, 1);
  if (who_am_i == 0x68) {
    ESP_LOGI(TAG, "MPU6050 found");
  } else {
    ESP_LOGE(TAG, "MPU6050 not found");
  }

  mpu6050_write_byte(MPU6050_PWR_MGMT_1_REG, 0x00); /* Wake up MPU6050 */
}

static void mpu6050_read_accel_gyro(void) {
  uint8_t data[14]; /* XXX: Explain the 14 */
  int16_t accel_x, accel_y, accel_z, temp, gyro_x, gyro_y, gyro_z;
  float   temperature;

  int32_t sum_accel_x = 0, sum_accel_y = 0, sum_accel_z = 0;
  int32_t sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;
  float   sum_temp = 0.0;

  for (int i = 0; i < NUM_READINGS; i++) {
    mpu6050_read(MPU6050_ACCEL_XOUT_H, data, 14);

  /* XXX: Explain this */
    accel_x = (int16_t)(data[0] << 8 | data[1]);
    accel_y = (int16_t)(data[2] << 8 | data[3]);
    accel_z = (int16_t)(data[4] << 8 | data[5]);
    temp    = (int16_t)(data[6] << 8 | data[7]);
    gyro_x  = (int16_t)(data[8] << 8 | data[9]);
    gyro_y  = (int16_t)(data[10] << 8 | data[11]);
    gyro_z  = (int16_t)(data[12] << 8 | data[13]);

    temperature = (temp / 340.0) +
                  36.53; /* Convert the raw temperature to degrees Celsius */

    sum_accel_x += accel_x;
    sum_accel_y += accel_y;
    sum_accel_z += accel_z;
    sum_temp    += temperature;
    sum_gyro_x  += gyro_x;
    sum_gyro_y  += gyro_y;
    sum_gyro_z  += gyro_z;

    vTaskDelay(10 / portTICK_PERIOD_MS); /* Small delay between readings */
  }

  accel_x     = sum_accel_x / NUM_READINGS;
  accel_y     = sum_accel_y / NUM_READINGS;
  accel_z     = sum_accel_z / NUM_READINGS;
  temperature = sum_temp / NUM_READINGS;
  gyro_x      = sum_gyro_x / NUM_READINGS;
  gyro_y      = sum_gyro_y / NUM_READINGS;
  gyro_z      = sum_gyro_z / NUM_READINGS;

  ESP_LOGI(TAG, "Accel X: %d, Y: %d, Z: %d", accel_x, accel_y, accel_z);
  ESP_LOGI(TAG, "Temp: %.2f C", temperature);
  ESP_LOGI(TAG, "Gyro X: %d, Y: %d, Z: %d", gyro_x, gyro_y, gyro_z);
}

void app_main(void) {
  ESP_ERROR_CHECK(i2c_master_init());
  mpu6050_init();

  while (1) {
    mpu6050_read_accel_gyro();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
