/* components/sensors/qmc5883l_hal/qmc5883l_hal.c */

#include "qmc5883l_hal.h"
#include "common/i2c.h"
#include <esp_log.h>
#include <math.h>

/* Constants ******************************************************************/

const uint8_t  qmc5883l_i2c_address        = 0x0D;
const uint8_t  qmc5883l_i2c_bus            = I2C_NUM_0;
const char    *qmc5883l_tag                = "QMC5883L";
const uint8_t  qmc5883l_scl_io             = GPIO_NUM_22;
const uint8_t  qmc5883l_sda_io             = GPIO_NUM_21;
const uint32_t qmc5883l_i2c_freq_hz        = 100000;
const uint32_t qmc5883l_polling_rate_ticks = pdMS_TO_TICKS(5 * 1000);
const uint8_t  qmc5883l_odr_setting        = k_qmc5883l_odr_100hz;

/* Static const array of QMC5883L configurations and scaling factors */
static const qmc5883l_scale_t qmc5883l_scale_configs[] = {
  {k_qmc5883l_range_2g, 200.0 / 32768.0}, /**< ±2 Gauss range, scaling factor */
  {k_qmc5883l_range_8g, 800.0 / 32768.0}, /**< ±8 Gauss range, scaling factor */
};

static const uint8_t qmc5883l_scale_config_idx = 0; /* Index of chosen values (0 for ±2G, 1 for ±8G) */

/* Public Functions ***********************************************************/

esp_err_t qmc5883l_init(void *sensor_data)
{
  qmc5883l_data_t *qmc5883l_data = (qmc5883l_data_t *)sensor_data;
  ESP_LOGI(qmc5883l_tag, "Starting Configuration");

  qmc5883l_data->i2c_address = qmc5883l_i2c_address;
  qmc5883l_data->i2c_bus     = qmc5883l_i2c_bus;
  qmc5883l_data->mag_x       = qmc5883l_data->mag_y = qmc5883l_data->mag_z = 0.0;
  qmc5883l_data->state       = k_qmc5883l_uninitialized; /* Start in uninitialized state */

  esp_err_t ret = priv_i2c_init(qmc5883l_scl_io, qmc5883l_sda_io, 
                                qmc5883l_i2c_freq_hz, qmc5883l_i2c_bus,
                                qmc5883l_tag);

  if (ret != ESP_OK) {
    ESP_LOGE(qmc5883l_tag, "I2C driver install failed: %s", esp_err_to_name(ret));
    qmc5883l_data->state = k_qmc5883l_power_on_error;
    return ret;
  } else {
    ESP_LOGI(qmc5883l_tag, "I2C driver install complete");
  }

  /* Combine all settings into one configuration byte */
  uint8_t ctrl1 = k_qmc5883l_mode_continuous | qmc5883l_odr_setting |
                  qmc5883l_scale_configs[qmc5883l_scale_config_idx].range |
                  k_qmc5883l_osr_512;

  /* Write the configuration to the CTRL1 register */
  ret = priv_i2c_write_reg_byte(k_qmc5883l_ctrl1_cmd, ctrl1,
                                qmc5883l_i2c_bus, qmc5883l_i2c_address, qmc5883l_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(qmc5883l_tag, "Configuration of CTRL1 register failed");
    qmc5883l_data->state = k_qmc5883l_error;
    return ret;
  } else {
    ESP_LOGI(qmc5883l_tag, "Configuration of CTRL1 register complete");
  }

  qmc5883l_data->state = k_qmc5883l_ready;
  ESP_LOGI(qmc5883l_tag, "Sensor Configuration Complete");
  return ESP_OK;
}

void qmc5883l_read(qmc5883l_data_t *sensor_data)
{
  if (sensor_data == NULL) {
    ESP_LOGE(qmc5883l_tag, "Sensor data pointer is NULL");
    return;
  }

  uint8_t mag_data[6];
  esp_err_t ret;

  /* Read 6 bytes of magnetometer data starting from Data Output X_LSB (0x00) */
  ret = priv_i2c_read_reg_bytes(0x00, mag_data, 6,
                                sensor_data->i2c_bus, sensor_data->i2c_address, qmc5883l_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(qmc5883l_tag, "Failed to read magnetometer data from QMC5883L");
    sensor_data->state = k_qmc5883l_error;
    return;
  }

  sensor_data->state = k_qmc5883l_data_updated;

  /* Combine high and low bytes to form the raw X, Y, Z values */
  int16_t mag_x_raw = (int16_t)((mag_data[1] << 8) | mag_data[0]);
  int16_t mag_y_raw = (int16_t)((mag_data[3] << 8) | mag_data[2]);
  int16_t mag_z_raw = (int16_t)((mag_data[5] << 8) | mag_data[4]);

  /* Convert the raw data to meaningful values using scaling factors */
  float scale_factor = qmc5883l_scale_configs[qmc5883l_scale_config_idx].scale;
  sensor_data->mag_x = mag_x_raw * scale_factor;
  sensor_data->mag_y = mag_y_raw * scale_factor;
  sensor_data->mag_z = mag_z_raw * scale_factor;

  /* Calculate the heading (yaw) from the X and Y magnetic field components */
  float heading = atan2(sensor_data->mag_y, sensor_data->mag_x) * 180.0 / M_PI;
  if (heading < 0) {
    heading += 360.0; /* Normalize heading to the range [0, 360] degrees */
  }

  /* Save the heading into the sensor data structure */
  sensor_data->heading = heading;

  /* Log the magnetic field values and the calculated heading */
  ESP_LOGI(qmc5883l_tag, "Mag X: %f, Mag Y: %f, Mag Z: %f, Heading: %f degrees", 
           sensor_data->mag_x, sensor_data->mag_y, sensor_data->mag_z, 
           sensor_data->heading);
}

void qmc5883l_tasks(void *sensor_data)
{
  qmc5883l_data_t *qmc5883l_data = (qmc5883l_data_t *)sensor_data;
  while (1) {
    qmc5883l_read(qmc5883l_data);
    vTaskDelay(qmc5883l_polling_rate_ticks);
  }
}
