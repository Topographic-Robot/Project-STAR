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
/* Note: (2^16)/2 = 32768 */
static const qmc5883l_scale_t qmc5883l_scale_configs[] = {
  {k_qmc5883l_range_2g, 200.0 / 32768.0}, /**< ±2 Gauss range, scaling factor = 200 µT / 32768 LSB = 0.0061 µT/LSB */
  {k_qmc5883l_range_8g, 800.0 / 32768.0}, /**< ±8 Gauss range, scaling factor = 800 µT / 32768 LSB = 0.0244 µT/LSB */
};

static const uint8_t qmc5883l_scale_config_idx = 0; /* Index of chosen values (0 for ±2G, 1 for ±8G) */

/* Public Functions ***********************************************************/

esp_err_t qmc5883l_init(qmc5883l_data_t *sensor_data, bool first_time)
{
  ESP_LOGI(qmc5883l_tag, "Starting Configuration");
  if (first_time) {
    sensor_data->sensor_mutex = NULL; /* Set NULL, and change it later when it's ready */
  }

  sensor_data->i2c_address = qmc5883l_i2c_address;
  sensor_data->i2c_bus     = qmc5883l_i2c_bus;
  sensor_data->mag_x       = sensor_data->mag_y = sensor_data->mag_z = 0.0;
  sensor_data->state       = 0; /* Start in uninitialized state */

  esp_err_t ret = priv_i2c_init(qmc5883l_scl_io, qmc5883l_sda_io, 
                                qmc5883l_i2c_freq_hz, qmc5883l_i2c_bus,
                                qmc5883l_tag);

  if (ret != ESP_OK) {
    ESP_LOGE(qmc5883l_tag, "I2C driver install failed: %s", esp_err_to_name(ret));
    return ret;
  } else {
    ESP_LOGI(qmc5883l_tag, "I2C driver install complete");
  }

  /* Configure the Output Data Rate (ODR) to 100 Hz. */
  /**
   * @brief Configure the Output Data Rate (ODR) for the QMC5883L sensor.
   *
   * Why is Output Data Rate (ODR) Important?
   * The Output Data Rate (ODR) controls how often the QMC5883L sensor samples and 
   * updates the magnetic field data. A higher ODR provides more frequent updates, 
   * which is beneficial for real-time systems, but it also increases
   * the power consumption of the sensor and the processing load on the microcontroller.
   * 
   * Reducing the ODR can be useful for:
   *
   * - Reducing power consumption: Lower ODR values reduce the power draw of 
   *   the sensor, which can be important in battery-powered applications where 
   *   power efficiency is critical.
   *
   * - Reducing processing load: If your application does not require high-frequency 
   *   data updates, a lower ODR helps reduce the amount of data the microcontroller 
   *   has to process, saving CPU cycles.
   *
   * - Matching system requirements: Many robotic and navigation systems only 
   *   require updates at 50 Hz or 100 Hz to function effectively. Setting a lower 
   *   ODR reduces unnecessary overhead.
   *
   * Set up the Output Data Rate (ODR) to 100 Hz.
   * This setting balances responsiveness with power consumption, making it ideal 
   * for many real-time applications
   * like navigation and orientation tracking.
   */
  ret = priv_i2c_write_byte(k_qmc5883l_ctrl1_cmd | qmc5883l_odr_setting, 
                            qmc5883l_i2c_bus, qmc5883l_i2c_address, qmc5883l_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(qmc5883l_tag, "ODR configuration failed");
    return ret;
  } else {
    ESP_LOGI(qmc5883l_tag, "ODR configuration complete");
  }

  /* Configure the full-scale magnetic range to ±2 Gauss. */
  /**
   * @brief Configure the full-scale range for the QMC5883L sensor.
   *
   * Why is Magnetic Range Important?
   * The magnetic range setting determines the maximum magnetic field strength 
   * that the sensor can measure. The QMC5883L offers two full-scale ranges: 
   * ±2 Gauss and ±8 Gauss.
   *
   * Choosing the appropriate magnetic range depends on the expected magnetic field 
   * strength in your application:
   *
   * - ±2 Gauss: This range provides the highest sensitivity, which is ideal 
   *   for detecting small magnetic field changes like the Earth’s magnetic field. 
   *   It offers finer resolution, making it suitable for applications such as 
   *   compass-based navigation or heading detection. However, this range can 
   *   saturate in the presence of strong magnetic fields.
   *
   * - ±8 Gauss: This range can measure stronger magnetic fields without saturating, 
   *   but it provides lower sensitivity compared to the ±2 Gauss setting. It is 
   *   useful for environments where stronger magnetic fields are present.
   *
   * Set up the full-scale magnetic range to ±2 Gauss for high sensitivity.
   * This range is sufficient for most applications that involve detecting the 
   * Earth’s magnetic field and is typically used for orientation and navigation purposes.
   */
  ret = priv_i2c_write_byte(k_qmc5883l_ctrl1_cmd | qmc5883l_scale_configs[qmc5883l_scale_config_idx].range,
                            qmc5883l_i2c_bus, qmc5883l_i2c_address, qmc5883l_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(qmc5883l_tag, "magnetic range configuration failed");
    return ret;
  } else {
    ESP_LOGI(qmc5883l_tag, "magnetic range configuration complete");
  }

  /* Configure the Over-Sampling Ratio (OSR) to 512. */
  /**
   * @brief Configure the Over-Sampling Ratio (OSR) for the QMC5883L sensor.
   *
   * Why is Over-Sampling Ratio (OSR) Important?
   * The Over-Sampling Ratio (OSR) controls how many internal measurements are 
   * averaged to produce the final magnetic field data. Higher OSR values improve 
   * the sensor's accuracy by reducing noise at the cost of slower updates. Lower 
   * OSR values result in faster updates but may introduce more noise into the 
   * measurements.
   *
   * Benefits of Higher OSR Values:
   *
   * - Improved accuracy: A higher OSR reduces noise and improves the precision 
   *   of the magnetic field measurements, which is important for applications that 
   *   require precise heading or orientation data, such as navigation systems.
   *
   * - Smoother data: Higher OSR values help filter out high-frequency noise, 
   *   providing smoother sensor readings.
   *
   * - Noise reduction: In applications with noisy environments, a higher OSR 
   *   helps to suppress random fluctuations in the magnetic field readings.
   *
   * The downside of higher OSR is the increased power consumption and slower data 
   * output rate, but for many applications, the trade-off is worth the improved 
   * accuracy.
   *
   * Set up the Over-Sampling Ratio (OSR) to 512 to improve noise performance and 
   * accuracy. This value provides a good balance between accuracy and speed, 
   * especially in navigation and orientation applications where high precision 
   * is required.
   */
  ret = priv_i2c_write_byte(k_qmc5883l_ctrl1_cmd | k_qmc5883l_osr_512, qmc5883l_i2c_bus, 
                            qmc5883l_i2c_address, qmc5883l_tag);
  if (ret != ESP_OK) {
    ESP_LOGE(qmc5883l_tag, "OSR configuration failed");
    return ret;
  } else {
    ESP_LOGI(qmc5883l_tag, "OSR configuration complete");
  }

  if (sensor_data->sensor_mutex == NULL) {
    sensor_data->sensor_mutex = xSemaphoreCreateMutex();
    if (sensor_data->sensor_mutex == NULL) {
      ESP_LOGE(qmc5883l_tag, "ESP32 ran out of memory");
      return ESP_ERR_NO_MEM;
    } else {
      ESP_LOGI(qmc5883l_tag, "Allocated memory for semaphore");
    }
  }

  sensor_data->state = 1; /* Sensor is initialized */
  ESP_LOGI(qmc5883l_tag, "Sensor Configuration Complete");
  return ESP_OK;
}

void qmc5883l_read(qmc5883l_data_t *sensor_data)
{
  if (sensor_data == NULL) {
    ESP_LOGE(qmc5883l_tag, "Sensor data pointer is NULL");
    return;
  }

  if (sensor_data->sensor_mutex == NULL) {
    ESP_LOGE(qmc5883l_tag, "Sensor data pointer's mutex is NULL");
    return;
  }

  if (xSemaphoreTake(sensor_data->sensor_mutex, 2 * qmc5883l_polling_rate_ticks) != pdTRUE) {
    ESP_LOGW(qmc5883l_tag, "Failed to take sensor mutex");
    return;
  }

//  uint8_t mag_data[6];
//  esp_err_t ret = priv_i2c_read_bytes(mag_data, 6, qmc5883l_i2c_bus, 
//                                      qmc5883l_i2c_address, qmc5883l_tag);
//  if (ret != ESP_OK) {
//    ESP_LOGE(qmc5883l_tag, "Failed to read magnetometer data from QMC5883L");
//    xSemaphoreGive(sensor_data->sensor_mutex);
//    return;
//  }
//
//  /* Combine high and low bytes to form the raw X, Y, Z values */
//  int16_t mag_x_raw = (int16_t)((mag_data[1] << 8) | mag_data[0]);
//  int16_t mag_y_raw = (int16_t)((mag_data[3] << 8) | mag_data[2]);
//  int16_t mag_z_raw = (int16_t)((mag_data[5] << 8) | mag_data[4]);
//
//  /* Convert the raw data to meaningful values using scaling factors */
//  float scale_factor = qmc5883l_scale_configs[qmc5883l_scale_config_idx].scale;
//  sensor_data->mag_x = mag_x_raw * scale_factor;
//  sensor_data->mag_y = mag_y_raw * scale_factor;
//  sensor_data->mag_z = mag_z_raw * scale_factor;
//
//  /* Calculate the heading (yaw) from the X and Y magnetic field components */
//  float heading = atan2(sensor_data->mag_y, sensor_data->mag_x) * 180.0 / M_PI;
//  if (heading < 0) {
//    heading += 360.0; /* Normalize heading to the range [0, 360] degrees */
//  }
//
//  /* Save the heading into the sensor data structure */
//  sensor_data->heading = heading;
//
//  /* Log the magnetic field values and the calculated heading */
//  ESP_LOGI(qmc5883l_tag, "Mag X: %f, Mag Y: %f, Mag Z: %f, Heading: %f degrees", 
//      sensor_data->mag_x, sensor_data->mag_y, sensor_data->mag_z, sensor_data->heading);
//
//  /* Release the mutex after accessing the shared data */
//  xSemaphoreGive(sensor_data->sensor_mutex);
}

void qmc5883l_tasks(void *sensor_data)
{
  while (1) {
    qmc5883l_data_t *qmc5883l_data = (qmc5883l_data_t *)sensor_data;
    qmc5883l_read(qmc5883l_data);
    vTaskDelay(qmc5883l_polling_rate_ticks);
  }
}

