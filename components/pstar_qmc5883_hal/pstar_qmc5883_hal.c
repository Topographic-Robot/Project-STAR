/* components/pstar_qmc5883_hal/pstar_qmc5883_hal.c */

#include "pstar_qmc5883_hal.h"

#include "pstar_bus_config.h"
#include "pstar_bus_i2c.h"
#include "pstar_bus_manager.h"
#include "pstar_pin_validator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" /* For mutex */
#include "freertos/task.h"   /* For vTaskDelay */

#include <math.h>   /* For calculations if needed */
#include <stdlib.h> /* For malloc, free */
#include <string.h> /* For strdup, memset */

#include "esp_check.h" /* For ESP_RETURN_ON_FALSE, ESP_GOTO_ON_ERROR */
#include "esp_log.h"
#include "sdkconfig.h" /* Include sdkconfig for Kconfig defines */

static const char* TAG = "QMC5883 HAL";

/* --- QMC5883 Register Definitions --- */
#define QMC5883_REG_DATA_X_LSB 0x00
#define QMC5883_REG_DATA_X_MSB 0x01
#define QMC5883_REG_DATA_Y_LSB 0x02
#define QMC5883_REG_DATA_Y_MSB 0x03
#define QMC5883_REG_DATA_Z_LSB 0x04
#define QMC5883_REG_DATA_Z_MSB 0x05
#define QMC5883_REG_STATUS 0x06
#define QMC5883_REG_TEMP_LSB 0x07
#define QMC5883_REG_TEMP_MSB 0x08
#define QMC5883_REG_CONTROL_1 0x09
#define QMC5883_REG_CONTROL_2 0x0A
#define QMC5883_REG_SET_RESET_PERIOD 0x0B /* FBR */
#define QMC5883_REG_CHIP_ID 0x0D          /* Should return 0xFF */

/* --- QMC5883 Status Register Bits --- */
#define QMC5883_STATUS_DRDY (1 << 0) /* Data Ready */
#define QMC5883_STATUS_OVL (1 << 1)  /* Overflow */
#define QMC5883_STATUS_DOR (1 << 2)  /* Data Skipped */

/* --- QMC5883 Control Register 1 Bits --- */
/* MODE (bits 0-1), ODR (bits 2-3), RNG (bits 4-5), OSR (bits 6-7) */
#define QMC5883_CTRL1_MODE_SHIFT 0
#define QMC5883_CTRL1_ODR_SHIFT 2
#define QMC5883_CTRL1_FSR_SHIFT 4 /* RNG in datasheet */
#define QMC5883_CTRL1_OSR_SHIFT 6

/* --- QMC5883 Control Register 2 Bits --- */
#define QMC5883_CTRL2_INT_ENB (1 << 0)  /* Interrupt Pin Enable */
#define QMC5883_CTRL2_ROL_PNT (1 << 6)  /* Pointer Roll-over */
#define QMC5883_CTRL2_SOFT_RST (1 << 7) /* Soft Reset */

/* --- Sensitivity Scale Factors (LSB/Gauss) --- */
#define QMC5883_SENS_2G 12000.0f
#define QMC5883_SENS_8G 3000.0f

/* --- Constants --- */
#define QMC5883_MUTEX_TIMEOUT_MS 1000 /* Timeout for acquiring HAL mutex */
#define QMC5883_CHIP_ID_VAL 0xFF      /* Expected value in Chip ID register */

/* --- Internal Handle Structure --- */
typedef struct pstar_qmc5883_hal_dev_t {
  const pstar_bus_manager_t* bus_manager;  /**< Pointer to the bus manager */
  char*                      bus_name;     /**< Name of the bus config (copied) */
  qmc5883_fsr_t              current_fsr;  /**< Current full scale range */
  float                      sensitivity;  /**< Sensitivity factor based on current FSR */
  uint8_t                    control_reg1; /**< Cached value of Control Register 1 */
  SemaphoreHandle_t          mutex;        /**< Mutex for thread-safe access */
} pstar_qmc5883_hal_dev_t;

/* --- Helper Functions --- */

/**
 * @brief Write a single byte to a QMC5883 register.
 *        Assumes mutex is already taken.
 */
static esp_err_t
priv_qmc5883_write_byte_nolock(pstar_qmc5883_hal_handle_t handle, uint8_t reg_addr, uint8_t value)
{
  return pstar_bus_i2c_write(handle->bus_manager, handle->bus_name, &value, 1, reg_addr, NULL);
}

/**
 * @brief Read a single byte from a QMC5883 register.
 *        Assumes mutex is already taken.
 */
static esp_err_t
priv_qmc5883_read_byte_nolock(pstar_qmc5883_hal_handle_t handle, uint8_t reg_addr, uint8_t* value)
{
  return pstar_bus_i2c_read(handle->bus_manager, handle->bus_name, value, 1, reg_addr, NULL);
}

/**
 * @brief Read multiple bytes from QMC5883 starting from a register address.
 *        Assumes mutex is already taken.
 */
static esp_err_t priv_qmc5883_read_bytes_nolock(pstar_qmc5883_hal_handle_t handle,
                                                uint8_t                    reg_addr,
                                                uint8_t*                   data,
                                                size_t                     len)
{
  return pstar_bus_i2c_read(handle->bus_manager, handle->bus_name, data, len, reg_addr, NULL);
}

/**
 * @brief Get the sensitivity factor for a given FSR.
 */
static float priv_get_sensitivity(qmc5883_fsr_t fsr)
{
  switch (fsr) {
    case QMC5883_FSR_2G:
      return QMC5883_SENS_2G;
    case QMC5883_FSR_8G:
      return QMC5883_SENS_8G;
    default:
      ESP_LOGW(TAG, "Unknown FSR value %d, defaulting to 2G sensitivity", fsr);
      return QMC5883_SENS_2G;
  }
}

/**
 * @brief Write the cached Control Register 1 value to the sensor.
 *        Assumes mutex is already taken.
 */
static esp_err_t priv_qmc5883_write_ctrl1_nolock(pstar_qmc5883_hal_handle_t handle)
{
  return priv_qmc5883_write_byte_nolock(handle, QMC5883_REG_CONTROL_1, handle->control_reg1);
}

/* --- Public Function Implementations --- */

esp_err_t pstar_qmc5883_hal_init(const pstar_bus_manager_t*        manager,
                                 const pstar_qmc5883_hal_config_t* config,
                                 pstar_qmc5883_hal_handle_t*       out_handle)
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
  /* Check if the address matches the expected QMC5883 address */
  /* --- CORRECTED ACCESS --- */
  if (bus_config->proto.i2c.address != QMC5883_I2C_ADDR) {
    ESP_LOGW(TAG,
             "Bus '%s' address (0x%02X) does not match expected QMC5883 address (0x%02X)",
             config->bus_name,
             bus_config->proto.i2c.address, /* --- CORRECTED ACCESS --- */
             QMC5883_I2C_ADDR);
    /* Continue anyway, but log warning */
  }

  /* Allocate memory for the device handle */
  pstar_qmc5883_hal_handle_t dev =
    (pstar_qmc5883_hal_handle_t)malloc(sizeof(pstar_qmc5883_hal_dev_t));
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Failed to allocate device handle");
  memset(dev, 0, sizeof(pstar_qmc5883_hal_dev_t));

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
  if (xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(QMC5883_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex during init for '%s'", dev->bus_name);
    vSemaphoreDelete(dev->mutex);
    free(dev->bus_name);
    free(dev);
    return ESP_ERR_TIMEOUT;
  }

  /* --- QMC5883 Initialization Sequence --- */
  ESP_LOGI(TAG, "Initializing QMC5883 on bus '%s'...", dev->bus_name);

  /* 1. Check Chip ID (Optional but recommended) */
  uint8_t chip_id = 0;
  ret             = priv_qmc5883_read_byte_nolock(dev, QMC5883_REG_CHIP_ID, &chip_id);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to read Chip ID: %s. Assuming QMC5883.", esp_err_to_name(ret));
    /* Continue initialization even if Chip ID read fails */
    ret = ESP_OK; /* Reset ret to allow continuation */
  } else if (chip_id != QMC5883_CHIP_ID_VAL) {
    ESP_LOGW(TAG,
             "Chip ID mismatch. Expected 0x%02X, got 0x%02X. May not be a QMC5883.",
             QMC5883_CHIP_ID_VAL,
             chip_id);
    /* Continue initialization anyway */
  } else {
    ESP_LOGD(TAG, "Chip ID check passed (0x%02X)", chip_id);
  }

  /* 2. Perform Soft Reset */
  ret = priv_qmc5883_write_byte_nolock(dev, QMC5883_REG_CONTROL_2, QMC5883_CTRL2_SOFT_RST);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to perform soft reset: %s", esp_err_to_name(ret));
    goto init_fail_mutex;
  }
  vTaskDelay(pdMS_TO_TICKS(10)); /* Wait a bit after reset */

  /* 3. Configure Control Register 1 (Mode, ODR, FSR, OSR) */
  dev->control_reg1 = ((uint8_t)config->osr << QMC5883_CTRL1_OSR_SHIFT) |
                      ((uint8_t)config->fsr << QMC5883_CTRL1_FSR_SHIFT) |
                      ((uint8_t)config->odr << QMC5883_CTRL1_ODR_SHIFT) |
                      ((uint8_t)config->mode << QMC5883_CTRL1_MODE_SHIFT);

  ret = priv_qmc5883_write_ctrl1_nolock(dev);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write Control Register 1: %s", esp_err_to_name(ret));
    goto init_fail_mutex;
  }

  /* 4. Configure Control Register 2 (Disable Interrupt, Enable Pointer Roll-over) */
  /* Pointer roll-over is useful for reading all data registers sequentially */
  ret = priv_qmc5883_write_byte_nolock(dev, QMC5883_REG_CONTROL_2, QMC5883_CTRL2_ROL_PNT);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write Control Register 2: %s", esp_err_to_name(ret));
    goto init_fail_mutex;
  }

  /* 5. Configure Set/Reset Period (Optional, use default) */
  /* Default value 0x01 is recommended by datasheet */
  ret = priv_qmc5883_write_byte_nolock(dev, QMC5883_REG_SET_RESET_PERIOD, 0x01);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to write Set/Reset Period Register: %s", esp_err_to_name(ret));
    /* Continue anyway */
    ret = ESP_OK;
  }

  /* Store initial configuration */
  dev->current_fsr = config->fsr;
  dev->sensitivity = priv_get_sensitivity(dev->current_fsr);

  /* Initialization successful */
  xSemaphoreGive(dev->mutex); /* Release mutex */

  ESP_LOGI(TAG, "QMC5883 initialized successfully on bus '%s'", dev->bus_name);
  ESP_LOGI(TAG,
           "  Mode: %d, ODR: %d, FSR: %d, OSR: %d",
           config->mode,
           config->odr,
           config->fsr,
           config->osr);
  *out_handle = dev;
  return ESP_OK;

init_fail_mutex:
  xSemaphoreGive(dev->mutex); /* Release mutex on failure */
  ESP_LOGE(TAG, "QMC5883 initialization failed on bus '%s'", dev->bus_name);
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

esp_err_t pstar_qmc5883_hal_deinit(pstar_qmc5883_hal_handle_t handle, bool standby)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGI(TAG,
           "Deinitializing QMC5883 HAL for bus '%s'",
           handle->bus_name ? handle->bus_name : "UNKNOWN");

  /* Take mutex */
  if (handle->mutex &&
      xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(QMC5883_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex during deinit for '%s'", handle->bus_name);
    /* Continue cleanup cautiously */
  }

  if (standby) {
    uint8_t ctrl1 = handle->control_reg1;
    ctrl1         = (ctrl1 & ~(0x03 << QMC5883_CTRL1_MODE_SHIFT)) |
            ((uint8_t)QMC5883_MODE_STANDBY << QMC5883_CTRL1_MODE_SHIFT);
    esp_err_t stdby_ret = priv_qmc5883_write_byte_nolock(handle, QMC5883_REG_CONTROL_1, ctrl1);
    if (stdby_ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to send Standby command during deinit: %s", esp_err_to_name(stdby_ret));
    } else {
      ESP_LOGD(TAG, "Sent Standby command.");
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

esp_err_t pstar_qmc5883_hal_set_mode(pstar_qmc5883_hal_handle_t handle, qmc5883_mode_t mode)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(QMC5883_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for set_mode on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  handle->control_reg1 = (handle->control_reg1 & ~(0x03 << QMC5883_CTRL1_MODE_SHIFT)) |
                         ((uint8_t)mode << QMC5883_CTRL1_MODE_SHIFT);
  ret = priv_qmc5883_write_ctrl1_nolock(handle);
  if (ret == ESP_OK) {
    ESP_LOGD(TAG, "Set mode to %d for '%s'", mode, handle->bus_name);
  } else {
    ESP_LOGE(TAG, "Failed to set mode: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_qmc5883_hal_set_odr(pstar_qmc5883_hal_handle_t handle, qmc5883_odr_t odr)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(QMC5883_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for set_odr on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  handle->control_reg1 = (handle->control_reg1 & ~(0x03 << QMC5883_CTRL1_ODR_SHIFT)) |
                         ((uint8_t)odr << QMC5883_CTRL1_ODR_SHIFT);
  ret = priv_qmc5883_write_ctrl1_nolock(handle);
  if (ret == ESP_OK) {
    ESP_LOGD(TAG, "Set ODR to %d for '%s'", odr, handle->bus_name);
  } else {
    ESP_LOGE(TAG, "Failed to set ODR: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_qmc5883_hal_set_fsr(pstar_qmc5883_hal_handle_t handle, qmc5883_fsr_t fsr)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(QMC5883_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for set_fsr on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  handle->control_reg1 = (handle->control_reg1 & ~(0x03 << QMC5883_CTRL1_FSR_SHIFT)) |
                         ((uint8_t)fsr << QMC5883_CTRL1_FSR_SHIFT);
  ret = priv_qmc5883_write_ctrl1_nolock(handle);
  if (ret == ESP_OK) {
    handle->current_fsr = fsr;
    handle->sensitivity = priv_get_sensitivity(fsr);
    ESP_LOGD(TAG,
             "Set FSR to %d (+/- %dG), sensitivity %.1f LSB/G",
             fsr,
             fsr == 0 ? 2 : 8,
             handle->sensitivity);
  } else {
    ESP_LOGE(TAG, "Failed to set FSR: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_qmc5883_hal_set_osr(pstar_qmc5883_hal_handle_t handle, qmc5883_osr_t osr)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(QMC5883_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for set_osr on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  handle->control_reg1 = (handle->control_reg1 & ~(0x03 << QMC5883_CTRL1_OSR_SHIFT)) |
                         ((uint8_t)osr << QMC5883_CTRL1_OSR_SHIFT);
  ret = priv_qmc5883_write_ctrl1_nolock(handle);
  if (ret == ESP_OK) {
    ESP_LOGD(TAG, "Set OSR to %d for '%s'", osr, handle->bus_name);
  } else {
    ESP_LOGE(TAG, "Failed to set OSR: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_qmc5883_hal_is_data_ready(pstar_qmc5883_hal_handle_t handle, bool* data_ready)
{
  ESP_RETURN_ON_FALSE(handle && data_ready,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Handle or data_ready pointer is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret    = ESP_OK;
  uint8_t   status = 0;
  *data_ready      = false; /* Default to false */

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(QMC5883_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for is_data_ready on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  ret = priv_qmc5883_read_byte_nolock(handle, QMC5883_REG_STATUS, &status);
  if (ret == ESP_OK) {
    *data_ready = (status & QMC5883_STATUS_DRDY) ? true : false;
    /* Optionally check OVL and DOR bits */
    if (status & QMC5883_STATUS_OVL) {
      ESP_LOGW(TAG, "Status check on '%s': Overflow detected (OVL=1)", handle->bus_name);
    }
    if (status & QMC5883_STATUS_DOR) {
      ESP_LOGW(TAG, "Status check on '%s': Data skipped (DOR=1)", handle->bus_name);
    }
  } else {
    ESP_LOGE(TAG, "Failed to read status register: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_qmc5883_hal_read_raw_data(pstar_qmc5883_hal_handle_t handle,
                                          qmc5883_raw_data_t*        raw_data)
{
  ESP_RETURN_ON_FALSE(handle && raw_data, ESP_ERR_INVALID_ARG, TAG, "Handle or raw_data is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;
  uint8_t   buffer[6];

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(QMC5883_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for read_raw_data on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  ret = priv_qmc5883_read_bytes_nolock(handle, QMC5883_REG_DATA_X_LSB, buffer, 6);
  if (ret == ESP_OK) {
    raw_data->x = (int16_t)((buffer[1] << 8) | buffer[0]);
    raw_data->y = (int16_t)((buffer[3] << 8) | buffer[2]);
    raw_data->z = (int16_t)((buffer[5] << 8) | buffer[4]);
  } else {
    ESP_LOGE(TAG, "Failed to read raw mag data: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_qmc5883_hal_read_data(pstar_qmc5883_hal_handle_t handle, qmc5883_data_t* data)
{
  ESP_RETURN_ON_FALSE(handle && data, ESP_ERR_INVALID_ARG, TAG, "Handle or data pointer is NULL");

  qmc5883_raw_data_t raw_data;
  esp_err_t          ret = pstar_qmc5883_hal_read_raw_data(handle, &raw_data);
  if (ret != ESP_OK) {
    return ret;
  }

  /* Sensitivity factor is updated when range is set and protected by mutex */
  float sens = handle->sensitivity;
  if (sens <= 0) { /* Defensive check */
    ESP_LOGE(TAG, "Invalid sensitivity factor (%.2f)", sens);
    return ESP_ERR_INVALID_STATE;
  }

  data->x = (float)raw_data.x / sens;
  data->y = (float)raw_data.y / sens;
  data->z = (float)raw_data.z / sens;

  return ESP_OK;
}

esp_err_t pstar_qmc5883_hal_read_raw_temp(pstar_qmc5883_hal_handle_t handle, int16_t* raw_temp)
{
  ESP_RETURN_ON_FALSE(handle && raw_temp, ESP_ERR_INVALID_ARG, TAG, "Handle or raw_temp is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;
  uint8_t   buffer[2];

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(QMC5883_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for read_raw_temp on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  ret = priv_qmc5883_read_bytes_nolock(handle, QMC5883_REG_TEMP_LSB, buffer, 2);
  if (ret == ESP_OK) {
    *raw_temp = (int16_t)((buffer[1] << 8) | buffer[0]);
    /* Note: Datasheet doesn't specify temp conversion factor. */
    /* It's often around 100 LSB/¬∞C, but calibration needed. */
  } else {
    ESP_LOGE(TAG, "Failed to read raw temp data: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_qmc5883_hal_reset(pstar_qmc5883_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(QMC5883_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for reset on '%s'", handle->bus_name);
    return ESP_ERR_TIMEOUT;
  }

  ESP_LOGI(TAG, "Performing software reset for '%s'", handle->bus_name);
  ret = priv_qmc5883_write_byte_nolock(handle, QMC5883_REG_CONTROL_2, QMC5883_CTRL2_SOFT_RST);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write soft reset command: %s", esp_err_to_name(ret));
  } else {
    /* Reset cached control register value to default (standby) */
    handle->control_reg1 = 0x00;
    vTaskDelay(pdMS_TO_TICKS(10)); /* Wait a bit after reset */
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

/* --- Convenience Creation Functions --- */

esp_err_t pstar_qmc5883_hal_create_kconfig_default(pstar_bus_manager_t*        manager,
                                                   pstar_qmc5883_hal_handle_t* out_handle)
{
#if !CONFIG_PSTAR_KCONFIG_QMC5883_ENABLED
  ESP_LOGE(TAG, "QMC5883 component is disabled in Kconfig.");
  if (out_handle)
    *out_handle = NULL;
  return ESP_ERR_NOT_SUPPORTED;
#else
  ESP_RETURN_ON_FALSE(manager && out_handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

  *out_handle                            = NULL;
  esp_err_t           ret                = ESP_OK;
  bool                config_created     = false;
  bool                config_added       = false;
  pstar_bus_config_t* qmc5883_i2c_config = NULL;
  const char*         bus_name           = CONFIG_PSTAR_KCONFIG_QMC5883_BUS_NAME;

  ESP_LOGI(TAG, "Creating QMC5883 with KConfig default configuration");

#ifdef CONFIG_PSTAR_KCONFIG_QMC5883_I2C_PORT_0
  const i2c_port_t port = I2C_NUM_0;
#else
  const i2c_port_t port = I2C_NUM_1;
#endif

  /* 1. Create Bus Configuration using KConfig values */
  qmc5883_i2c_config = pstar_bus_config_create_i2c(bus_name,
                                                   port,
                                                   QMC5883_I2C_ADDR, /* Use fixed address */
                                                   CONFIG_PSTAR_KCONFIG_QMC5883_SDA_PIN,
                                                   CONFIG_PSTAR_KCONFIG_QMC5883_SCL_PIN,
                                                   CONFIG_PSTAR_KCONFIG_QMC5883_I2C_FREQ_HZ);

  ESP_GOTO_ON_FALSE(qmc5883_i2c_config != NULL,
                    ESP_ERR_NO_MEM,
                    cleanup,
                    TAG,
                    "Failed to create QMC5883 I2C config");
  config_created = true;

  /* 2. Add Configuration to the Manager */
  ret = pstar_bus_manager_add_bus(manager, qmc5883_i2c_config);
  if (ret == ESP_ERR_INVALID_STATE) { /* Already exists */
    ESP_LOGW(TAG, "Bus config '%s' already exists, finding...", bus_name);
    qmc5883_i2c_config = pstar_bus_manager_find_bus(manager, bus_name);
    ESP_GOTO_ON_FALSE(qmc5883_i2c_config,
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
                      "Failed to add QMC5883 config to manager: %s",
                      esp_err_to_name(ret));
    config_added   = true;
    config_created = false;
  }

  /* 3. Initialize the bus hardware */
  if (!qmc5883_i2c_config->initialized) {
    // *** CORRECTED LINE ***
    ret = pstar_bus_config_init(qmc5883_i2c_config, manager);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed to initialize bus hardware for '%s': %s",
                      bus_name,
                      esp_err_to_name(ret));
  }

  /* 4. Initialize the QMC5883 HAL Component with KConfig settings */
  pstar_qmc5883_hal_config_t hal_config = {
    .bus_name = bus_name,
    .odr      = CONFIG_PSTAR_KCONFIG_QMC5883_ODR,
    .fsr      = CONFIG_PSTAR_KCONFIG_QMC5883_FSR,
    .osr      = CONFIG_PSTAR_KCONFIG_QMC5883_OSR,
    .mode     = CONFIG_PSTAR_KCONFIG_QMC5883_MODE,
  };

  ret = pstar_qmc5883_hal_init(manager, &hal_config, out_handle);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to initialize QMC5883 HAL: %s",
                    esp_err_to_name(ret));

  ESP_LOGI(TAG, "QMC5883 created and initialized successfully with KConfig defaults");
  return ESP_OK;

cleanup:
  if (config_created && !config_added) {
    ESP_LOGW(TAG, "Destroying config that was created but not added to manager");
    pstar_bus_config_destroy(qmc5883_i2c_config);
  }
  *out_handle = NULL;
  return ret;
#endif /* CONFIG_PSTAR_KCONFIG_QMC5883_ENABLED */
}

esp_err_t pstar_qmc5883_hal_create_custom(pstar_bus_manager_t*        manager,
                                          const char*                 bus_name,
                                          i2c_port_t                  port,
                                          int                         sda_pin,
                                          int                         scl_pin,
                                          uint32_t                    i2c_freq_hz,
                                          qmc5883_odr_t               odr,
                                          qmc5883_fsr_t               fsr,
                                          qmc5883_osr_t               osr,
                                          qmc5883_mode_t              mode,
                                          pstar_qmc5883_hal_handle_t* out_handle)
{
  ESP_RETURN_ON_FALSE(manager && bus_name && out_handle,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid arguments");

  *out_handle                            = NULL;
  esp_err_t           ret                = ESP_OK;
  bool                config_created     = false;
  bool                config_added       = false;
  pstar_bus_config_t* qmc5883_i2c_config = NULL;

  ESP_LOGI(TAG, "Creating QMC5883 with custom configuration for bus '%s'", bus_name);

  /* 1. Create Bus Configuration */
  qmc5883_i2c_config =
    pstar_bus_config_create_i2c(bus_name, port, QMC5883_I2C_ADDR, sda_pin, scl_pin, i2c_freq_hz);
  ESP_GOTO_ON_FALSE(qmc5883_i2c_config != NULL,
                    ESP_ERR_NO_MEM,
                    cleanup,
                    TAG,
                    "Failed create custom QMC5883 I2C config");
  config_created = true;

  /* 2. Add Config to Manager */
  ret = pstar_bus_manager_add_bus(manager, qmc5883_i2c_config);
  if (ret == ESP_ERR_INVALID_STATE) { /* Already exists */
    ESP_LOGW(TAG, "Custom bus config '%s' already exists, finding...", bus_name);
    qmc5883_i2c_config = pstar_bus_manager_find_bus(manager, bus_name);
    ESP_GOTO_ON_FALSE(qmc5883_i2c_config,
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
                      "Failed add custom QMC5883 config: %s",
                      esp_err_to_name(ret));
    config_added   = true;
    config_created = false;
  }

  /* 3. Initialize Bus Hardware */
  if (!qmc5883_i2c_config->initialized) {
    // *** CORRECTED LINE ***
    ret = pstar_bus_config_init(qmc5883_i2c_config, manager);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed init custom QMC5883 bus '%s': %s",
                      bus_name,
                      esp_err_to_name(ret));
  }

  /* 4. Initialize HAL */
  pstar_qmc5883_hal_config_t hal_config = {.bus_name = bus_name,
                                           .odr      = odr,
                                           .fsr      = fsr,
                                           .osr      = osr,
                                           .mode     = mode};
  ret                                   = pstar_qmc5883_hal_init(manager, &hal_config, out_handle);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed init custom QMC5883 HAL: %s", esp_err_to_name(ret));

  ESP_LOGI(TAG, "QMC5883 (Custom) created and initialized successfully");
  return ESP_OK;

cleanup:
  if (config_created && !config_added) {
    pstar_bus_config_destroy(qmc5883_i2c_config);
  }
  *out_handle = NULL;
  return ret;
}

/* --- Pin Registration Functions --- */

esp_err_t pstar_qmc5883_register_kconfig_pins(void)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_QMC5883_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering QMC5883 pins with validator (from KConfig)...");

  ret = pstar_qmc5883_register_custom_pins(CONFIG_PSTAR_KCONFIG_QMC5883_SDA_PIN,
                                           CONFIG_PSTAR_KCONFIG_QMC5883_SCL_PIN);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register QMC5883 KConfig pins");

  ESP_LOGI(TAG, "QMC5883 KConfig pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator or QMC5883 disabled, skipping QMC5883 KConfig pin registration.");
  return ESP_OK;
#endif
}

esp_err_t pstar_qmc5883_register_custom_pins(int sda_pin, int scl_pin)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG,
           "Registering QMC5883 custom pins (SDA:%d, SCL:%d) with validator...",
           sda_pin,
           scl_pin);

  ret = pstar_register_pin(sda_pin, "QMC5883 Custom SDA", true); /* I2C pins shareable */
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom SDA pin (%d)", sda_pin);
  ret = pstar_register_pin(scl_pin, "QMC5883 Custom SCL", true); /* I2C pins shareable */
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom SCL pin (%d)", scl_pin);

  ESP_LOGI(TAG, "QMC5883 custom pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator disabled, skipping QMC5883 custom pin registration.");
  return ESP_OK;
#endif
}