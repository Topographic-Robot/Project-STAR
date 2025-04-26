/* components/pstar_mpu6050_hal/pstar_mpu6050_hal.c */

#include "pstar_mpu6050_hal.h"

#include "pstar_bus_config.h"
#include "pstar_bus_i2c.h"
#include "pstar_bus_manager.h"
#include "pstar_pin_validator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" /* For mutex */
#include "freertos/task.h"   /* For vTaskDelay */

#include <math.h>   /* For powf */
#include <stdlib.h> /* For malloc, free */
#include <string.h> /* For strdup, memset */

#include "esp_check.h" /* For ESP_RETURN_ON_FALSE, ESP_GOTO_ON_ERROR */
#include "esp_log.h"
#include "sdkconfig.h" /* Include sdkconfig for Kconfig defines */

static const char* TAG = "MPU6050 HAL";

/* --- MPU6050 Register Definitions --- */
#define MPU6050_REG_SMPLRT_DIV 0x19   /**< Sample Rate Divider */
#define MPU6050_REG_CONFIG 0x1A       /**< Configuration (DLPF) */
#define MPU6050_REG_GYRO_CONFIG 0x1B  /**< Gyroscope Configuration */
#define MPU6050_REG_ACCEL_CONFIG 0x1C /**< Accelerometer Configuration */
#define MPU6050_REG_INT_PIN_CFG 0x37  /**< INT Pin / Bypass Enable Configuration */
#define MPU6050_REG_INT_ENABLE 0x38   /**< Interrupt Enable */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B /**< Accelerometer Measurements */
#define MPU6050_REG_ACCEL_XOUT_L 0x3C
#define MPU6050_REG_ACCEL_YOUT_H 0x3D
#define MPU6050_REG_ACCEL_YOUT_L 0x3E
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F
#define MPU6050_REG_ACCEL_ZOUT_L 0x40
#define MPU6050_REG_TEMP_OUT_H 0x41 /**< Temperature Measurement */
#define MPU6050_REG_TEMP_OUT_L 0x42
#define MPU6050_REG_GYRO_XOUT_H 0x43 /**< Gyroscope Measurements */
#define MPU6050_REG_GYRO_XOUT_L 0x44
#define MPU6050_REG_GYRO_YOUT_H 0x45
#define MPU6050_REG_GYRO_YOUT_L 0x46
#define MPU6050_REG_GYRO_ZOUT_H 0x47
#define MPU6050_REG_GYRO_ZOUT_L 0x48
#define MPU6050_REG_PWR_MGMT_1 0x6B /**< Power Management 1 */
#define MPU6050_REG_PWR_MGMT_2 0x6C /**< Power Management 2 */
#define MPU6050_REG_WHO_AM_I 0x75   /**< WHO_AM_I register, should return 0x68 */

/* --- MPU6050 WHO_AM_I Value --- */
#define MPU6050_WHO_AM_I_VAL 0x68

/* --- Sensitivity Scale Factors (LSB/unit) --- */
/* Accelerometer */
#define MPU6050_ACCEL_SENS_2G 16384.0f
#define MPU6050_ACCEL_SENS_4G 8192.0f
#define MPU6050_ACCEL_SENS_8G 4096.0f
#define MPU6050_ACCEL_SENS_16G 2048.0f
/* Gyroscope */
#define MPU6050_GYRO_SENS_250DPS 131.0f
#define MPU6050_GYRO_SENS_500DPS 65.5f
#define MPU6050_GYRO_SENS_1000DPS 32.8f
#define MPU6050_GYRO_SENS_2000DPS 16.4f
/* Temperature */
#define MPU6050_TEMP_SENS 340.0f
#define MPU6050_TEMP_OFFSET 36.53f

/* --- Constants --- */
#define MPU6050_MUTEX_TIMEOUT_MS 1000 /* Timeout for acquiring HAL mutex */

/* --- Internal Handle Structure --- */
typedef struct pstar_mpu6050_hal_dev_t {
  const pstar_bus_manager_t* bus_manager;      /**< Pointer to the bus manager */
  char*                      bus_name;         /**< Name of the bus config (copied) */
  mpu6050_accel_fs_t         current_accel_fs; /**< Current accelerometer range */
  mpu6050_gyro_fs_t          current_gyro_fs;  /**< Current gyroscope range */
  float                      accel_sens;       /**< Accelerometer sensitivity factor */
  float                      gyro_sens;        /**< Gyroscope sensitivity factor */
  SemaphoreHandle_t          mutex;            /**< Mutex for thread-safe access */
} pstar_mpu6050_hal_dev_t;

/* --- Helper Functions --- */

/**
 * @brief Write a single byte to an MPU6050 register.
 *        Assumes mutex is already taken.
 */
static esp_err_t
priv_mpu6050_write_byte_nolock(pstar_mpu6050_hal_handle_t handle, uint8_t reg_addr, uint8_t value)
{
  return pstar_bus_i2c_write(handle->bus_manager, handle->bus_name, &value, 1, reg_addr, NULL);
}

/**
 * @brief Read a single byte from an MPU6050 register.
 *        Assumes mutex is already taken.
 */
static esp_err_t
priv_mpu6050_read_byte_nolock(pstar_mpu6050_hal_handle_t handle, uint8_t reg_addr, uint8_t* value)
{
  return pstar_bus_i2c_read(handle->bus_manager, handle->bus_name, value, 1, reg_addr, NULL);
}

/**
 * @brief Read multiple bytes from MPU6050 starting from a register address.
 *        Assumes mutex is already taken.
 */
static esp_err_t priv_mpu6050_read_bytes_nolock(pstar_mpu6050_hal_handle_t handle,
                                                uint8_t                    reg_addr,
                                                uint8_t*                   data,
                                                size_t                     len)
{
  return pstar_bus_i2c_read(handle->bus_manager, handle->bus_name, data, len, reg_addr, NULL);
}

/**
 * @brief Get the sensitivity factor for a given accelerometer range.
 */
static float priv_get_accel_sensitivity(mpu6050_accel_fs_t range)
{
  switch (range) {
    case MPU6050_ACCEL_FS_2G:
      return MPU6050_ACCEL_SENS_2G;
    case MPU6050_ACCEL_FS_4G:
      return MPU6050_ACCEL_SENS_4G;
    case MPU6050_ACCEL_FS_8G:
      return MPU6050_ACCEL_SENS_8G;
    case MPU6050_ACCEL_FS_16G:
      return MPU6050_ACCEL_SENS_16G;
    default:
      return MPU6050_ACCEL_SENS_2G; /* Default */
  }
}

/**
 * @brief Get the sensitivity factor for a given gyroscope range.
 */
static float priv_get_gyro_sensitivity(mpu6050_gyro_fs_t range)
{
  switch (range) {
    case MPU6050_GYRO_FS_250DPS:
      return MPU6050_GYRO_SENS_250DPS;
    case MPU6050_GYRO_FS_500DPS:
      return MPU6050_GYRO_SENS_500DPS;
    case MPU6050_GYRO_FS_1000DPS:
      return MPU6050_GYRO_SENS_1000DPS;
    case MPU6050_GYRO_FS_2000DPS:
      return MPU6050_GYRO_SENS_2000DPS;
    default:
      return MPU6050_GYRO_SENS_250DPS; /* Default */
  }
}

/* --- Public Function Implementations --- */

esp_err_t pstar_mpu6050_hal_init(const pstar_bus_manager_t*        manager,
                                 const pstar_mpu6050_hal_config_t* config,
                                 pstar_mpu6050_hal_handle_t*       out_handle)
{
  ESP_RETURN_ON_FALSE(manager && config && config->bus_name && out_handle,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid arguments");
  *out_handle   = NULL;
  esp_err_t ret = ESP_OK;

  /* Find the bus configuration provided by the user */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, config->bus_name);
  ESP_RETURN_ON_FALSE(bus_config,
                      ESP_ERR_NOT_FOUND,
                      TAG,
                      "Bus '%s' not found in manager",
                      config->bus_name);
  ESP_RETURN_ON_FALSE(bus_config->initialized,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "Bus '%s' is not initialized",
                      config->bus_name);
  ESP_RETURN_ON_FALSE(bus_config->type == k_pstar_bus_type_i2c,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Bus '%s' is not I2C",
                      config->bus_name);

  /* Allocate memory for the device handle */
  pstar_mpu6050_hal_handle_t dev =
    (pstar_mpu6050_hal_handle_t)malloc(sizeof(pstar_mpu6050_hal_dev_t));
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Failed to allocate device handle");
  memset(dev, 0, sizeof(pstar_mpu6050_hal_dev_t));

  dev->bus_manager = manager;
  dev->bus_name    = strdup(config->bus_name); /* Store a copy of the name */
  if (!dev->bus_name) {
    ESP_LOGE(TAG, "Failed to duplicate bus name");
    free(dev);
    return ESP_ERR_NO_MEM;
  }

  /* Create mutex */
  dev->mutex = xSemaphoreCreateMutex();
  if (!dev->mutex) {
    ESP_LOGE(TAG, "Failed to create mutex for handle '%s'", dev->bus_name);
    free(dev->bus_name);
    free(dev);
    return ESP_ERR_NO_MEM;
  }

  /* Take mutex for the rest of initialization */
  if (xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(MPU6050_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex during init for '%s'", dev->bus_name);
    vSemaphoreDelete(dev->mutex);
    free(dev->bus_name);
    free(dev);
    return ESP_ERR_TIMEOUT;
  }

  /* --- MPU6050 Initialization Sequence --- */
  ESP_LOGI(TAG, "Initializing MPU6050 on bus '%s'...", dev->bus_name);

  /* 1. Check WHO_AM_I */
  uint8_t who_am_i = 0;
  ret              = priv_mpu6050_read_byte_nolock(dev, MPU6050_REG_WHO_AM_I, &who_am_i);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
    goto init_fail_mutex;
  }
  if (who_am_i != MPU6050_WHO_AM_I_VAL) {
    ESP_LOGE(TAG,
             "WHO_AM_I check failed. Expected 0x%02X, got 0x%02X",
             MPU6050_WHO_AM_I_VAL,
             who_am_i);
    ret = ESP_ERR_INVALID_RESPONSE;
    goto init_fail_mutex;
  }
  ESP_LOGD(TAG, "WHO_AM_I check passed (0x%02X)", who_am_i);

  /* 2. Wake up sensor (clear SLEEP bit in PWR_MGMT_1) and set clock source */
  /* Recommended clock source: PLL with X Gyro reference (0x01) */
  ret = priv_mpu6050_write_byte_nolock(dev, MPU6050_REG_PWR_MGMT_1, 0x01);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write PWR_MGMT_1 (wakeup/clock): %s", esp_err_to_name(ret));
    goto init_fail_mutex;
  }
  vTaskDelay(pdMS_TO_TICKS(100)); /* Wait for sensor to stabilize */

  /* 3. Set Accelerometer Full Scale Range */
  ret = priv_mpu6050_write_byte_nolock(dev,
                                       MPU6050_REG_ACCEL_CONFIG,
                                       (uint8_t)(config->accel_fs_sel << 3));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set accel range: %s", esp_err_to_name(ret));
    goto init_fail_mutex;
  }
  dev->current_accel_fs = config->accel_fs_sel;
  dev->accel_sens       = priv_get_accel_sensitivity(dev->current_accel_fs);

  /* 4. Set Gyroscope Full Scale Range */
  ret = priv_mpu6050_write_byte_nolock(dev,
                                       MPU6050_REG_GYRO_CONFIG,
                                       (uint8_t)(config->gyro_fs_sel << 3));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set gyro range: %s", esp_err_to_name(ret));
    goto init_fail_mutex;
  }
  dev->current_gyro_fs = config->gyro_fs_sel;
  dev->gyro_sens       = priv_get_gyro_sensitivity(dev->current_gyro_fs);

  /* 5. Configure Digital Low Pass Filter (DLPF) */
  ret = priv_mpu6050_write_byte_nolock(dev, MPU6050_REG_CONFIG, (uint8_t)config->dlpf_bw);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set DLPF bandwidth: %s", esp_err_to_name(ret));
    goto init_fail_mutex;
  }

  /* Optional: Disable interrupts if not used */
  ret = priv_mpu6050_write_byte_nolock(dev, MPU6050_REG_INT_ENABLE, 0x00);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to disable interrupts (INT_ENABLE): %s", esp_err_to_name(ret));
    /* Continue anyway */
  }

  /* Initialization successful */
  xSemaphoreGive(dev->mutex); /* Release mutex */

  ESP_LOGI(TAG, "MPU6050 initialized successfully on bus '%s'", dev->bus_name);
  ESP_LOGI(TAG,
           "  Accel Range: +/- %dg, Gyro Range: +/- %d dps, DLPF BW: %d",
           (int)powf(2, dev->current_accel_fs + 1),
           (int)(250 * powf(2, dev->current_gyro_fs)),
           config->dlpf_bw);
  *out_handle = dev;
  return ESP_OK;

init_fail_mutex:
  xSemaphoreGive(dev->mutex); /* Release mutex on failure */
  ESP_LOGE(TAG, "MPU6050 initialization failed on bus '%s'", dev->bus_name);
  if (dev) {
    if (dev->mutex) {
      vSemaphoreDelete(dev->mutex);
    }
    if (dev->bus_name) {
      free(dev->bus_name);
    }
    free(dev);
  }
  *out_handle = NULL;
  return ret;
}

esp_err_t pstar_mpu6050_hal_deinit(pstar_mpu6050_hal_handle_t handle, bool sleep)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGI(TAG,
           "Deinitializing MPU6050 HAL for bus '%s'",
           handle->bus_name ? handle->bus_name : "UNKNOWN");

  /* Take mutex */
  if (handle->mutex &&
      xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(MPU6050_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex during deinit for '%s'", handle->bus_name);
    /* Continue cleanup cautiously */
  }

  if (sleep) {
    esp_err_t sleep_ret = priv_mpu6050_write_byte_nolock(handle, MPU6050_REG_PWR_MGMT_1, 0x40);
    if (sleep_ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to send Sleep command during deinit: %s", esp_err_to_name(sleep_ret));
    } else {
      ESP_LOGD(TAG, "Sent Sleep command.");
    }
  }

  /* Release and delete mutex */
  if (handle->mutex) {
    xSemaphoreGive(handle->mutex);
    vSemaphoreDelete(handle->mutex);
    handle->mutex = NULL;
  }

  if (handle->bus_name) {
    free(handle->bus_name);
  }
  free(handle);
  return ESP_OK;
}

esp_err_t pstar_mpu6050_hal_read_raw_accel(pstar_mpu6050_hal_handle_t handle,
                                           mpu6050_raw_accel_t*       raw_data)
{
  ESP_RETURN_ON_FALSE(handle && raw_data, ESP_ERR_INVALID_ARG, TAG, "Handle or raw_data is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;
  uint8_t   buffer[6];

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(MPU6050_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for read_raw_accel on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  ret = priv_mpu6050_read_bytes_nolock(handle, MPU6050_REG_ACCEL_XOUT_H, buffer, 6);
  if (ret == ESP_OK) {
    raw_data->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw_data->y = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw_data->z = (int16_t)((buffer[4] << 8) | buffer[5]);
  } else {
    ESP_LOGE(TAG, "Failed to read raw accel data: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_mpu6050_hal_read_raw_gyro(pstar_mpu6050_hal_handle_t handle,
                                          mpu6050_raw_gyro_t*        raw_data)
{
  ESP_RETURN_ON_FALSE(handle && raw_data, ESP_ERR_INVALID_ARG, TAG, "Handle or raw_data is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;
  uint8_t   buffer[6];

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(MPU6050_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for read_raw_gyro on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  ret = priv_mpu6050_read_bytes_nolock(handle, MPU6050_REG_GYRO_XOUT_H, buffer, 6);
  if (ret == ESP_OK) {
    raw_data->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw_data->y = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw_data->z = (int16_t)((buffer[4] << 8) | buffer[5]);
  } else {
    ESP_LOGE(TAG, "Failed to read raw gyro data: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_mpu6050_hal_read_raw_temp(pstar_mpu6050_hal_handle_t handle, int16_t* raw_temp)
{
  ESP_RETURN_ON_FALSE(handle && raw_temp, ESP_ERR_INVALID_ARG, TAG, "Handle or raw_temp is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;
  uint8_t   buffer[2];

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(MPU6050_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for read_raw_temp on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  ret = priv_mpu6050_read_bytes_nolock(handle, MPU6050_REG_TEMP_OUT_H, buffer, 2);
  if (ret == ESP_OK) {
    *raw_temp = (int16_t)((buffer[0] << 8) | buffer[1]);
  } else {
    ESP_LOGE(TAG, "Failed to read raw temp data: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_mpu6050_hal_read_accel(pstar_mpu6050_hal_handle_t handle,
                                       mpu6050_accel_t*           accel_data)
{
  ESP_RETURN_ON_FALSE(handle && accel_data,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Handle or accel_data is NULL");

  mpu6050_raw_accel_t raw_data;
  esp_err_t           ret = pstar_mpu6050_hal_read_raw_accel(handle, &raw_data);
  if (ret != ESP_OK) {
    return ret;
  }

  /* Sensitivity factor is updated when range is set and protected by mutex */
  float sens = handle->accel_sens;
  if (sens <= 0) { /* Defensive check */
    ESP_LOGE(TAG, "Invalid accelerometer sensitivity factor (%.2f)", sens);
    return ESP_ERR_INVALID_STATE;
  }

  accel_data->x = (float)raw_data.x / sens;
  accel_data->y = (float)raw_data.y / sens;
  accel_data->z = (float)raw_data.z / sens;

  return ESP_OK;
}

esp_err_t pstar_mpu6050_hal_read_gyro(pstar_mpu6050_hal_handle_t handle, mpu6050_gyro_t* gyro_data)
{
  ESP_RETURN_ON_FALSE(handle && gyro_data, ESP_ERR_INVALID_ARG, TAG, "Handle or gyro_data is NULL");

  mpu6050_raw_gyro_t raw_data;
  esp_err_t          ret = pstar_mpu6050_hal_read_raw_gyro(handle, &raw_data);
  if (ret != ESP_OK) {
    return ret;
  }

  /* Sensitivity factor is updated when range is set and protected by mutex */
  float sens = handle->gyro_sens;
  if (sens <= 0) { /* Defensive check */
    ESP_LOGE(TAG, "Invalid gyroscope sensitivity factor (%.2f)", sens);
    return ESP_ERR_INVALID_STATE;
  }

  gyro_data->x = (float)raw_data.x / sens;
  gyro_data->y = (float)raw_data.y / sens;
  gyro_data->z = (float)raw_data.z / sens;

  return ESP_OK;
}

esp_err_t pstar_mpu6050_hal_read_temp(pstar_mpu6050_hal_handle_t handle, float* temp_c)
{
  ESP_RETURN_ON_FALSE(handle && temp_c, ESP_ERR_INVALID_ARG, TAG, "Handle or temp_c is NULL");

  int16_t   raw_temp;
  esp_err_t ret = pstar_mpu6050_hal_read_raw_temp(handle, &raw_temp);
  if (ret != ESP_OK) {
    return ret;
  }

  *temp_c = ((float)raw_temp / MPU6050_TEMP_SENS) + MPU6050_TEMP_OFFSET;

  return ESP_OK;
}

esp_err_t pstar_mpu6050_hal_set_accel_range(pstar_mpu6050_hal_handle_t handle,
                                            mpu6050_accel_fs_t         range)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(MPU6050_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for set_accel_range on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  ret = priv_mpu6050_write_byte_nolock(handle, MPU6050_REG_ACCEL_CONFIG, (uint8_t)(range << 3));
  if (ret == ESP_OK) {
    handle->current_accel_fs = range;
    handle->accel_sens       = priv_get_accel_sensitivity(range);
    ESP_LOGD(TAG,
             "Set accel range to %d (+/- %dg), sensitivity %.1f LSB/g",
             range,
             (int)powf(2, range + 1),
             handle->accel_sens);
  } else {
    ESP_LOGE(TAG, "Failed to set accel range: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_mpu6050_hal_set_gyro_range(pstar_mpu6050_hal_handle_t handle,
                                           mpu6050_gyro_fs_t          range)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(MPU6050_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for set_gyro_range on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  ret = priv_mpu6050_write_byte_nolock(handle, MPU6050_REG_GYRO_CONFIG, (uint8_t)(range << 3));
  if (ret == ESP_OK) {
    handle->current_gyro_fs = range;
    handle->gyro_sens       = priv_get_gyro_sensitivity(range);
    ESP_LOGD(TAG,
             "Set gyro range to %d (+/- %d dps), sensitivity %.1f LSB/(deg/s)",
             range,
             (int)(250 * powf(2, range)),
             handle->gyro_sens);
  } else {
    ESP_LOGE(TAG, "Failed to set gyro range: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_mpu6050_hal_set_dlpf_bandwidth(pstar_mpu6050_hal_handle_t handle,
                                               mpu6050_dlpf_bw_t          bandwidth)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(MPU6050_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for set_dlpf_bandwidth on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  ret = priv_mpu6050_write_byte_nolock(handle, MPU6050_REG_CONFIG, (uint8_t)bandwidth);
  if (ret == ESP_OK) {
    ESP_LOGD(TAG, "Set DLPF bandwidth to %d", bandwidth);
  } else {
    ESP_LOGE(TAG, "Failed to set DLPF bandwidth: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

/* --- Convenience Creation Functions --- */

esp_err_t pstar_mpu6050_hal_create_kconfig_default(pstar_bus_manager_t*        manager,
                                                   pstar_mpu6050_hal_handle_t* out_handle)
{
#if !CONFIG_PSTAR_KCONFIG_MPU6050_ENABLED
  ESP_LOGE(TAG, "MPU6050 component is disabled in Kconfig.");
  if (out_handle)
    *out_handle = NULL;
  return ESP_ERR_NOT_SUPPORTED;
#else
  ESP_RETURN_ON_FALSE(manager && out_handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

  *out_handle                            = NULL;
  esp_err_t           ret                = ESP_OK;
  bool                config_created     = false;
  bool                config_added       = false;
  pstar_bus_config_t* mpu6050_i2c_config = NULL;
  const char*         bus_name           = CONFIG_PSTAR_KCONFIG_MPU6050_BUS_NAME;

  ESP_LOGI(TAG, "Creating MPU6050 with KConfig default configuration");

#ifdef CONFIG_PSTAR_KCONFIG_MPU6050_I2C_PORT_0
  const i2c_port_t port = I2C_NUM_0;
#else
  const i2c_port_t port = I2C_NUM_1;
#endif

#ifdef CONFIG_PSTAR_KCONFIG_MPU6050_ADDR_LOW
  const uint8_t addr = MPU6050_I2C_ADDR_LOW;
#else
  const uint8_t addr = MPU6050_I2C_ADDR_HIGH;
#endif

  /* 1. Create Bus Configuration using KConfig values */
  mpu6050_i2c_config = pstar_bus_config_create_i2c(bus_name,
                                                   port,
                                                   addr,
                                                   CONFIG_PSTAR_KCONFIG_MPU6050_SDA_PIN,
                                                   CONFIG_PSTAR_KCONFIG_MPU6050_SCL_PIN,
                                                   CONFIG_PSTAR_KCONFIG_MPU6050_I2C_FREQ_HZ);

  ESP_GOTO_ON_FALSE(mpu6050_i2c_config != NULL,
                    ESP_ERR_NO_MEM,
                    cleanup,
                    TAG,
                    "Failed to create MPU6050 I2C config");
  config_created = true;

  /* 2. Add Configuration to the Manager */
  ret = pstar_bus_manager_add_bus(manager, mpu6050_i2c_config);
  if (ret == ESP_ERR_INVALID_STATE) { /* Already exists */
    ESP_LOGW(TAG, "Bus config '%s' already exists, finding...", bus_name);
    mpu6050_i2c_config = pstar_bus_manager_find_bus(manager, bus_name);
    ESP_GOTO_ON_FALSE(mpu6050_i2c_config,
                      ESP_FAIL,
                      cleanup,
                      TAG,
                      "Failed to find existing bus '%s'",
                      bus_name);
    config_added   = true;
    config_created = false;
    ret            = ESP_OK;
  } else {
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed to add MPU6050 config to manager: %s",
                      esp_err_to_name(ret));
    config_added   = true;
    config_created = false;
  }

  /* 3. Initialize the bus hardware */
  if (!mpu6050_i2c_config->initialized) {
    // *** CORRECTED LINE ***
    ret = pstar_bus_config_init(mpu6050_i2c_config, manager);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed to initialize bus hardware for '%s': %s",
                      bus_name,
                      esp_err_to_name(ret));
  }

  /* 4. Initialize the MPU6050 HAL Component with KConfig settings */
  pstar_mpu6050_hal_config_t hal_config = {
    .bus_name     = bus_name,
    .accel_fs_sel = CONFIG_PSTAR_KCONFIG_MPU6050_ACCEL_FS,
    .gyro_fs_sel  = CONFIG_PSTAR_KCONFIG_MPU6050_GYRO_FS,
    .dlpf_bw      = CONFIG_PSTAR_KCONFIG_MPU6050_DLPF_BW,
  };

  ret = pstar_mpu6050_hal_init(manager, &hal_config, out_handle);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to initialize MPU6050 HAL: %s",
                    esp_err_to_name(ret));

  ESP_LOGI(TAG, "MPU6050 created and initialized successfully with KConfig defaults");
  return ESP_OK;

cleanup:
  if (config_created && !config_added) {
    ESP_LOGW(TAG, "Destroying config that was created but not added to manager");
    pstar_bus_config_destroy(mpu6050_i2c_config);
  }
  *out_handle = NULL;
  return ret;
#endif /* CONFIG_PSTAR_KCONFIG_MPU6050_ENABLED */
}

esp_err_t pstar_mpu6050_hal_create_custom(pstar_bus_manager_t*        manager,
                                          const char*                 bus_name,
                                          i2c_port_t                  port,
                                          uint8_t                     addr,
                                          int                         sda_pin,
                                          int                         scl_pin,
                                          uint32_t                    i2c_freq_hz,
                                          mpu6050_accel_fs_t          accel_fs,
                                          mpu6050_gyro_fs_t           gyro_fs,
                                          mpu6050_dlpf_bw_t           dlpf_bw,
                                          pstar_mpu6050_hal_handle_t* out_handle)
{
  ESP_RETURN_ON_FALSE(manager && bus_name && out_handle,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid arguments");
  ESP_RETURN_ON_FALSE(addr == MPU6050_I2C_ADDR_LOW || addr == MPU6050_I2C_ADDR_HIGH,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid MPU6050 I2C address: 0x%02X",
                      addr);

  *out_handle                            = NULL;
  esp_err_t           ret                = ESP_OK;
  bool                config_created     = false;
  bool                config_added       = false;
  pstar_bus_config_t* mpu6050_i2c_config = NULL;

  ESP_LOGI(TAG, "Creating MPU6050 with custom configuration for bus '%s'", bus_name);

  /* 1. Create Bus Configuration */
  mpu6050_i2c_config =
    pstar_bus_config_create_i2c(bus_name, port, addr, sda_pin, scl_pin, i2c_freq_hz);
  ESP_GOTO_ON_FALSE(mpu6050_i2c_config != NULL,
                    ESP_ERR_NO_MEM,
                    cleanup,
                    TAG,
                    "Failed create custom MPU6050 I2C config");
  config_created = true;

  /* 2. Add Config to Manager */
  ret = pstar_bus_manager_add_bus(manager, mpu6050_i2c_config);
  if (ret == ESP_ERR_INVALID_STATE) { /* Already exists */
    ESP_LOGW(TAG, "Custom bus config '%s' already exists, finding...", bus_name);
    mpu6050_i2c_config = pstar_bus_manager_find_bus(manager, bus_name);
    ESP_GOTO_ON_FALSE(mpu6050_i2c_config,
                      ESP_FAIL,
                      cleanup,
                      TAG,
                      "Failed find existing custom bus '%s'",
                      bus_name);
    config_added   = true;
    config_created = false;
    ret            = ESP_OK;
  } else {
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed add custom MPU6050 config: %s",
                      esp_err_to_name(ret));
    config_added   = true;
    config_created = false;
  }

  /* 3. Initialize Bus Hardware */
  if (!mpu6050_i2c_config->initialized) {
    // *** CORRECTED LINE ***
    ret = pstar_bus_config_init(mpu6050_i2c_config, manager);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed init custom MPU6050 bus '%s': %s",
                      bus_name,
                      esp_err_to_name(ret));
  }

  /* 4. Initialize HAL */
  pstar_mpu6050_hal_config_t hal_config = {
    .bus_name     = bus_name,
    .accel_fs_sel = accel_fs,
    .gyro_fs_sel  = gyro_fs,
    .dlpf_bw      = dlpf_bw,
  };
  ret = pstar_mpu6050_hal_init(manager, &hal_config, out_handle);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed init custom MPU6050 HAL: %s", esp_err_to_name(ret));

  ESP_LOGI(TAG, "MPU6050 (Custom) created and initialized successfully");
  return ESP_OK;

cleanup:
  if (config_created && !config_added) {
    pstar_bus_config_destroy(mpu6050_i2c_config);
  }
  *out_handle = NULL;
  return ret;
}

/* --- Pin Registration Functions --- */

esp_err_t pstar_mpu6050_register_kconfig_pins(void)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_MPU6050_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering MPU6050 pins with validator (from KConfig)...");

  ret = pstar_mpu6050_register_custom_pins(CONFIG_PSTAR_KCONFIG_MPU6050_SDA_PIN,
                                           CONFIG_PSTAR_KCONFIG_MPU6050_SCL_PIN);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register MPU6050 KConfig pins");

  ESP_LOGI(TAG, "MPU6050 KConfig pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator or MPU6050 disabled, skipping MPU6050 KConfig pin registration.");
  return ESP_OK;
#endif
}

esp_err_t pstar_mpu6050_register_custom_pins(int sda_pin, int scl_pin)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG,
           "Registering MPU6050 custom pins (SDA:%d, SCL:%d) with validator...",
           sda_pin,
           scl_pin);

  ret = pstar_register_pin(sda_pin, "MPU6050 Custom SDA", true); /* I2C pins shareable */
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom SDA pin (%d)", sda_pin);
  ret = pstar_register_pin(scl_pin, "MPU6050 Custom SCL", true); /* I2C pins shareable */
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom SCL pin (%d)", scl_pin);

  ESP_LOGI(TAG, "MPU6050 custom pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator disabled, skipping MPU6050 custom pin registration.");
  return ESP_OK;
#endif
}