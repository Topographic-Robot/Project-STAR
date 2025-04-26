/* components/pstar_bh1750_hal/pstar_bh1750_hal.c */

#include "pstar_bh1750_hal.h"

/* Include necessary headers regardless of Kconfig */
#include "pstar_bus_config.h"
#include "pstar_bus_i2c.h"
#include "pstar_bus_manager.h"
#include "pstar_pin_validator.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "sdkconfig.h" /* Needed for Kconfig checks inside functions */

static const char* TAG = "BH1750 HAL";

/* BH1750 Command Opcodes */
#define BH1750_CMD_POWER_DOWN (0x00)
#define BH1750_CMD_POWER_ON (0x01)
#define BH1750_CMD_RESET (0x07) /* Resets data register (not measurement mode) */

/* Conversion factor: Lux = Measurement / 1.2 */
#define BH1750_LUX_CONVERSION_FACTOR (1.2f)

/* Measurement time estimates (add margin) */
#define BH1750_DELAY_LOW_RES_MS (24)
#define BH1750_DELAY_HIGH_RES_MS (180) /* Max time for H-ret modes */

/* --- Internal Handle Structure --- */
typedef struct bh1750_hal_dev_t {
  const pstar_bus_manager_t* bus_manager;          /* Pointer to the manager */
  char*                      bus_name;             /* Name of the bus config (copied) */
  bh1750_hal_mode_t          current_mode;         /* Currently configured mode */
  uint16_t                   measurement_delay_ms; /* Estimated delay for current mode */
} bh1750_hal_dev_t;

/* --- Helper Functions --- */

/* Write a command byte using the bus manager */
static esp_err_t priv_bh1750_write_cmd(bh1750_hal_handle_t handle, uint8_t cmd)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  /* Use the specific write_command function from the bus manager */
  return pstar_bus_i2c_write_command(handle->bus_manager, handle->bus_name, cmd);
}

/* Read data bytes using the bus manager */
static esp_err_t priv_bh1750_read_data(bh1750_hal_handle_t handle, uint8_t* data, size_t len)
{
  ESP_RETURN_ON_FALSE(handle && data,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Device handle or data ptr is NULL");
  /* Use the specific read_raw function from the bus manager */
  return pstar_bus_i2c_read_raw(handle->bus_manager, handle->bus_name, data, len, NULL);
}

/* Get estimated delay for a given mode */
static uint16_t priv_bh1750_get_delay_ms(bh1750_hal_mode_t mode)
{
  switch (mode) {
    case BH1750_MODE_CONTINUOUS_LOW_RES:
    case BH1750_MODE_ONE_TIME_LOW_RES:
      return BH1750_DELAY_LOW_RES_MS;
    case BH1750_MODE_CONTINUOUS_HIGH_RES:
    case BH1750_MODE_CONTINUOUS_HIGH_RES2:
    case BH1750_MODE_ONE_TIME_HIGH_RES:
    case BH1750_MODE_ONE_TIME_HIGH_RES2:
    default:
      return BH1750_DELAY_HIGH_RES_MS;
  }
}

/* --- Public Function Implementations --- */

esp_err_t pstar_bh1750_hal_init(
  const pstar_bus_manager_t* manager, // Keep const here if init doesn't modify manager
  const bh1750_hal_config_t* config,
  bh1750_hal_mode_t          initial_mode,
  bh1750_hal_handle_t*       out_handle)
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
  bh1750_hal_handle_t dev = (bh1750_hal_handle_t)malloc(sizeof(bh1750_hal_dev_t));
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Failed to allocate device handle");
  memset(dev, 0, sizeof(bh1750_hal_dev_t));

  dev->bus_manager = manager;
  dev->bus_name    = strdup(config->bus_name); /* Store a copy of the name */
  if (!dev->bus_name) {
    ESP_LOGE(TAG, "Failed to duplicate bus name");
    free(dev);
    return ESP_ERR_NO_MEM;
  }

  /* --- BH1750 Initialization Sequence --- */
  ESP_LOGI(TAG, "Initializing BH1750 on bus '%s'...", dev->bus_name);

  /* 1. Power On */
  ret = priv_bh1750_write_cmd(dev, BH1750_CMD_POWER_ON);
  ESP_GOTO_ON_ERROR(ret,
                    init_fail,
                    TAG,
                    "Failed to send Power On command: %s",
                    esp_err_to_name(ret));
  vTaskDelay(pdMS_TO_TICKS(5)); /* Small delay after power on */

  /* 2. Set Initial Mode */
  ret = pstar_bh1750_hal_set_mode(dev, initial_mode); /* This also updates internal state */
  ESP_GOTO_ON_ERROR(ret, init_fail, TAG, "Failed to set initial mode: %s", esp_err_to_name(ret));

  ESP_LOGI(TAG,
           "BH1750 initialized successfully on bus '%s' in mode 0x%02X",
           dev->bus_name,
           initial_mode);
  *out_handle = dev;
  return ESP_OK;

init_fail:
  if (dev) {
    if (dev->bus_name) {
      free(dev->bus_name); /* Free duplicated name */
    }
    free(dev);
  }
  *out_handle = NULL;
  return ret;
}

esp_err_t pstar_bh1750_hal_deinit(bh1750_hal_handle_t handle, bool power_down)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGI(TAG,
           "Deinitializing BH1750 HAL for bus '%s'",
           handle->bus_name ? handle->bus_name : "UNKNOWN");

  if (power_down) {
    esp_err_t pd_ret = priv_bh1750_write_cmd(handle, BH1750_CMD_POWER_DOWN);
    if (pd_ret != ESP_OK) {
      /* Log error but continue deinit */
      ESP_LOGE(TAG, "Failed to send Power Down command during deinit: %s", esp_err_to_name(pd_ret));
    } else {
      ESP_LOGD(TAG, "Sent Power Down command.");
    }
  }

  if (handle->bus_name) {
    free(handle->bus_name);
  }
  free(handle);
  return ESP_OK;
}

esp_err_t pstar_bh1750_hal_set_mode(bh1750_hal_handle_t handle, bh1750_hal_mode_t mode)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  esp_err_t ret;

  ESP_LOGD(TAG, "Setting mode to 0x%02X for '%s'", mode, handle->bus_name);
  ret = priv_bh1750_write_cmd(handle, (uint8_t)mode);
  if (ret == ESP_OK) {
    handle->current_mode         = mode;
    handle->measurement_delay_ms = priv_bh1750_get_delay_ms(mode);
    /* Wait for measurement time after setting mode, especially for one-time modes */
    /* Or require user to wait before reading */
    vTaskDelay(pdMS_TO_TICKS(handle->measurement_delay_ms));
    ESP_LOGD(TAG, "Mode set, waited %ums", handle->measurement_delay_ms);
  } else {
    ESP_LOGE(TAG,
             "Failed to set mode 0x%02X for '%s': %s",
             mode,
             handle->bus_name,
             esp_err_to_name(ret));
  }
  return ret;
}

esp_err_t pstar_bh1750_hal_read_lux(bh1750_hal_handle_t handle, float* lux)
{
  ESP_RETURN_ON_FALSE(handle && lux, ESP_ERR_INVALID_ARG, TAG, "Handle or lux pointer is NULL");
  *lux = 0.0f; /* Default value on error */
  esp_err_t ret;
  uint8_t   read_buffer[2];

  /* Optional: Add delay here if not done in set_mode, or if mode is continuous */
  /* vTaskDelay(pdMS_TO_TICKS(handle->measurement_delay_ms)); */
  /* Reading immediately might get stale data in continuous mode if read too fast */

  ESP_LOGD(TAG, "Reading data for '%s'", handle->bus_name);
  ret = priv_bh1750_read_data(handle, read_buffer, 2);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read data for '%s': %s", handle->bus_name, esp_err_to_name(ret));
    return ret;
  }

  /* Combine bytes */
  uint16_t raw_value = ((uint16_t)read_buffer[0] << 8) | (uint16_t)read_buffer[1];

  /* Convert to Lux */
  *lux = (float)raw_value / BH1750_LUX_CONVERSION_FACTOR;

  /* Adjust for MTreg or High Res Mode 2 if needed (datasheet) */
  /* This basic conversion works for standard modes */
  if (handle->current_mode == BH1750_MODE_CONTINUOUS_HIGH_RES2 ||
      handle->current_mode == BH1750_MODE_ONE_TIME_HIGH_RES2) {
    /* Mode 2 has 0.5 lux resolution, effectively dividing by 2 */
    *lux /= 2.0f;
  }

  ESP_LOGD(TAG, "Read Success from '%s': Raw=%u, Lux=%.2f", handle->bus_name, raw_value, *lux);
  return ESP_OK;
}

esp_err_t pstar_bh1750_hal_power_down(bh1750_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGD(TAG, "Sending Power Down command to '%s'", handle->bus_name);
  return priv_bh1750_write_cmd(handle, BH1750_CMD_POWER_DOWN);
}

esp_err_t pstar_bh1750_hal_power_on(bh1750_hal_handle_t handle)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGD(TAG, "Sending Power On command to '%s'", handle->bus_name);
  esp_err_t ret = priv_bh1750_write_cmd(handle, BH1750_CMD_POWER_ON);
  if (ret == ESP_OK) {
    vTaskDelay(pdMS_TO_TICKS(5)); /* Small delay after power on */
  }
  return ret;
}

esp_err_t pstar_bh1750_hal_create_kconfig_default(pstar_bus_manager_t* manager,
                                                  bh1750_hal_handle_t* out_handle)
{
/* --- Add Kconfig check at the beginning --- */
#if !CONFIG_PSTAR_KCONFIG_BH1750_ENABLED
  ESP_LOGE(TAG, "BH1750 component is disabled in Kconfig.");
  if (out_handle)
    *out_handle = NULL;
  return ESP_ERR_NOT_SUPPORTED;
#else
  ESP_RETURN_ON_FALSE(manager && out_handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

  *out_handle                           = NULL;
  esp_err_t           ret               = ESP_OK;
  bool                config_created    = false;
  bool                config_added      = false;
  pstar_bus_config_t* bh1750_i2c_config = NULL;

  ESP_LOGI(TAG, "Creating BH1750 with KConfig default configuration");

  /* Define Kconfig dependent values locally */
#ifdef CONFIG_PSTAR_KCONFIG_BH1750_I2C_PORT_0
  const i2c_port_t BH1750_I2C_MASTER_PORT = I2C_NUM_0;
#elif CONFIG_PSTAR_KCONFIG_BH1750_I2C_PORT_1
  const i2c_port_t BH1750_I2C_MASTER_PORT = I2C_NUM_1;
#else
#error "No default I2C port selected for BH1750 in Kconfig"
  const i2c_port_t BH1750_I2C_MASTER_PORT = I2C_NUM_0; /* Fallback */
#endif

#ifdef CONFIG_PSTAR_KCONFIG_BH1750_ADDR_LOW
  const uint8_t BH1750_I2C_ADDR = 0x23;
#elif CONFIG_PSTAR_KCONFIG_BH1750_ADDR_HIGH
  const uint8_t BH1750_I2C_ADDR = 0x5C;
#else
#error "No default I2C address selected for BH1750 in Kconfig"
  const uint8_t BH1750_I2C_ADDR = 0x23; /* Fallback */
#endif

  const char* BH1750_BUS_NAME = CONFIG_PSTAR_KCONFIG_BH1750_BUS_NAME;

  /* 1. Create Bus Configuration for BH1750 using KConfig values */
  bh1750_i2c_config = pstar_bus_config_create_i2c(BH1750_BUS_NAME,
                                                  BH1750_I2C_MASTER_PORT,
                                                  BH1750_I2C_ADDR,
                                                  CONFIG_PSTAR_KCONFIG_BH1750_SDA_PIN,
                                                  CONFIG_PSTAR_KCONFIG_BH1750_SCL_PIN,
                                                  CONFIG_PSTAR_KCONFIG_BH1750_I2C_FREQ_HZ);

  ESP_GOTO_ON_FALSE(bh1750_i2c_config != NULL,
                    ESP_ERR_NO_MEM,
                    cleanup,
                    TAG,
                    "Failed to create BH1750 I2C config");
  config_created = true;

  /* 2. Add Configuration to the Manager */
  ret = pstar_bus_manager_add_bus(manager, bh1750_i2c_config);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to add BH1750 config to manager: %s",
                    esp_err_to_name(ret));
  config_added   = true;
  config_created = false; /* Manager now owns the config pointer */

  /* 3. Initialize the bus hardware */
  // *** CORRECTED LINE ***
  ret = pstar_bus_config_init(bh1750_i2c_config, manager);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to initialize bus hardware for '%s': %s",
                    BH1750_BUS_NAME,
                    esp_err_to_name(ret));

  /* 4. Initialize the BH1750 HAL Component with default mode from KConfig */
  bh1750_hal_config_t hal_config = {.bus_name = BH1750_BUS_NAME};

  /* Get the default measurement mode from KConfig */
  bh1750_hal_mode_t default_mode;

#if CONFIG_PSTAR_KCONFIG_BH1750_MODE_CONTINUOUS_HIGH_RES
  default_mode = BH1750_MODE_CONTINUOUS_HIGH_RES;
#elif CONFIG_PSTAR_KCONFIG_BH1750_MODE_CONTINUOUS_HIGH_RES2
  default_mode = BH1750_MODE_CONTINUOUS_HIGH_RES2;
#elif CONFIG_PSTAR_KCONFIG_BH1750_MODE_CONTINUOUS_LOW_RES
  default_mode = BH1750_MODE_CONTINUOUS_LOW_RES;
#elif CONFIG_PSTAR_KCONFIG_BH1750_MODE_ONE_TIME_HIGH_RES
  default_mode = BH1750_MODE_ONE_TIME_HIGH_RES;
#elif CONFIG_PSTAR_KCONFIG_BH1750_MODE_ONE_TIME_HIGH_RES2
  default_mode = BH1750_MODE_ONE_TIME_HIGH_RES2;
#elif CONFIG_PSTAR_KCONFIG_BH1750_MODE_ONE_TIME_LOW_RES
  default_mode = BH1750_MODE_ONE_TIME_LOW_RES;
#else
#error "No default BH1750 mode selected in KConfig"
  default_mode = BH1750_MODE_CONTINUOUS_HIGH_RES; /* Fallback */
#endif

  ret = pstar_bh1750_hal_init(manager, &hal_config, default_mode, out_handle);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to initialize BH1750 HAL: %s", esp_err_to_name(ret));

  ESP_LOGI(TAG, "BH1750 created and initialized successfully with KConfig defaults");
  return ESP_OK;

cleanup:
  /* Clean up resources on failure */
  if (config_created && !config_added) {
    ESP_LOGW(TAG, "Destroying config that was created but not added to manager");
    pstar_bus_config_destroy(bh1750_i2c_config);
  }
  /* Note: If config was added but bus init or HAL init failed,
   * the bus config remains in the manager. The main app's cleanup
   * (pstar_bus_manager_deinit) should handle removing it.
   */

  *out_handle = NULL;
  return ret;
#endif /* CONFIG_PSTAR_KCONFIG_BH1750_ENABLED */
}

esp_err_t pstar_bh1750_hal_create_custom(pstar_bus_manager_t* manager,
                                         const char*          bus_name,
                                         i2c_port_t           port,
                                         uint8_t              addr,
                                         int                  sda_pin,
                                         int                  scl_pin,
                                         uint32_t             freq_hz,
                                         bh1750_hal_mode_t    mode,
                                         bh1750_hal_handle_t* out_handle)
{
  ESP_RETURN_ON_FALSE(manager && bus_name && out_handle,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid arguments");

  *out_handle                           = NULL;
  esp_err_t           ret               = ESP_OK;
  bool                config_created    = false;
  bool                config_added      = false;
  pstar_bus_config_t* bh1750_i2c_config = NULL;

  ESP_LOGI(TAG, "Creating BH1750 with custom configuration");

  /* 1. Create Bus Configuration for BH1750 using provided values */
  bh1750_i2c_config = pstar_bus_config_create_i2c(bus_name, port, addr, sda_pin, scl_pin, freq_hz);

  ESP_GOTO_ON_FALSE(bh1750_i2c_config != NULL,
                    ESP_ERR_NO_MEM,
                    cleanup,
                    TAG,
                    "Failed to create BH1750 I2C config");
  config_created = true;

  /* 2. Add Configuration to the Manager */
  ret = pstar_bus_manager_add_bus(manager, bh1750_i2c_config);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to add BH1750 config to manager: %s",
                    esp_err_to_name(ret));
  config_added   = true;
  config_created = false; /* Manager now owns the config pointer */

  /* 3. Initialize the bus hardware */
  // *** CORRECTED LINE ***
  ret = pstar_bus_config_init(bh1750_i2c_config, manager);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to initialize bus hardware for '%s': %s",
                    bus_name,
                    esp_err_to_name(ret));

  /* 4. Initialize the BH1750 HAL Component with provided mode */
  bh1750_hal_config_t hal_config = {.bus_name = bus_name};

  ret = pstar_bh1750_hal_init(manager, &hal_config, mode, out_handle);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to initialize BH1750 HAL: %s", esp_err_to_name(ret));

  ESP_LOGI(TAG, "BH1750 created and initialized successfully with custom config");
  return ESP_OK;

cleanup:
  /* Clean up resources on failure */
  if (config_created && !config_added) {
    ESP_LOGW(TAG, "Destroying config that was created but not added to manager");
    pstar_bus_config_destroy(bh1750_i2c_config);
  }
  /* Note: If config was added but bus init or HAL init failed,
   * the bus config remains in the manager. The main app's cleanup
   * (pstar_bus_manager_deinit) should handle removing it.
   */

  *out_handle = NULL;
  return ret;
}

esp_err_t pstar_bh1750_register_kconfig_pins(void)
{
/* --- Add Kconfig check at the beginning --- */
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_BH1750_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering BH1750 pins with validator (from KConfig)...");

  /* Register SDA pin */
  const char* sda_desc = "BH1750 SDA";
  ret                  = pstar_register_pin(CONFIG_PSTAR_KCONFIG_BH1750_SDA_PIN, sda_desc, true);
  ESP_RETURN_ON_ERROR(ret,
                      TAG,
                      "Failed to register BH1750 SDA pin (%d)",
                      CONFIG_PSTAR_KCONFIG_BH1750_SDA_PIN);

  /* Register SCL pin */
  const char* scl_desc = "BH1750 SCL";
  ret                  = pstar_register_pin(CONFIG_PSTAR_KCONFIG_BH1750_SCL_PIN, scl_desc, true);
  ESP_RETURN_ON_ERROR(ret,
                      TAG,
                      "Failed to register BH1750 SCL pin (%d)",
                      CONFIG_PSTAR_KCONFIG_BH1750_SCL_PIN);

  ESP_LOGI(TAG, "BH1750 KConfig pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator or BH1750 disabled, skipping BH1750 KConfig pin registration.");
  return ESP_OK; /* Not an error if validator or component is disabled */
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_BH1750_ENABLED */
}

esp_err_t pstar_bh1750_register_custom_pins(int sda_pin, int scl_pin)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering BH1750 custom pins with validator...");

  /* Register SDA pin */
  const char* sda_desc = "BH1750 Custom SDA";
  ret                  = pstar_register_pin(sda_pin, sda_desc, true);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register BH1750 custom SDA pin (%d)", sda_pin);

  /* Register SCL pin */
  const char* scl_desc = "BH1750 Custom SCL";
  ret                  = pstar_register_pin(scl_pin, scl_desc, true);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to register BH1750 custom SCL pin (%d)", scl_pin);

  ESP_LOGI(TAG, "BH1750 custom pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator disabled, skipping BH1750 custom pin registration.");
  return ESP_OK; /* Not an error if validator is disabled */
#endif /* CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED */
}