/* components/pstar_bus/pstar_bus_config.c */

#include "pstar_bus_config.h"

#include "pstar_bus_i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

/* --- Constants --- */

static const char* TAG = "BusConfig";

/* --- Private Function Prototypes --- */

static void priv_pstar_bus_config_cleanup(pstar_bus_config_t* config);

static pstar_bus_config_t* priv_pstar_bus_config_create_common(const char*      name,
                                                               pstar_bus_type_t type);

/* --- Public Functions --- */

pstar_bus_config_t* pstar_bus_config_create_i2c(const char* name,
                                                i2c_port_t  port,
                                                uint8_t     address,
                                                gpio_num_t  sda_pin,
                                                gpio_num_t  scl_pin,
                                                uint32_t    clk_speed)
{
  pstar_bus_config_t* config = priv_pstar_bus_config_create_common(name, k_pstar_bus_type_i2c);
  ESP_RETURN_ON_FALSE(config,
                      NULL,
                      TAG,
                      "Failed to create common config for %s",
                      name ? name : "NULL");

  /* Configure I2C-specific parameters */
  config->i2c.port    = port;
  config->i2c.address = address;

  /* Initialize with default I2C configuration */
  memset(&config->i2c.config, 0, sizeof(i2c_config_t));
  config->i2c.config.mode             = I2C_MODE_MASTER;
  config->i2c.config.sda_io_num       = sda_pin;
  config->i2c.config.scl_io_num       = scl_pin;
  config->i2c.config.sda_pullup_en    = GPIO_PULLUP_ENABLE; /* Common practice */
  config->i2c.config.scl_pullup_en    = GPIO_PULLUP_ENABLE; /* Common practice */
  config->i2c.config.master.clk_speed = clk_speed;

  /* Initialize default operations */
  pstar_bus_i2c_init_default_ops(&config->i2c.ops);

  ESP_LOGI(TAG,
           "Created I2C config '%s' (Port: %d, Addr: 0x%02X, SDA: %d, SCL: %d, Speed: %lu Hz)",
           name,
           port,
           address,
           sda_pin,
           scl_pin,
           (unsigned long)clk_speed);
  return config;
}

/* Creation functions for other bus types removed */

esp_err_t pstar_bus_config_destroy(pstar_bus_config_t* config)
{
  ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "Config pointer is NULL");

  /* If the bus is initialized, deinitialize it first */
  if (config->initialized) {
    esp_err_t ret = pstar_bus_config_deinit(config);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG,
               "Failed to deinitialize bus '%s' before destroying: %s. Continuing cleanup.",
               config->name ? config->name : "UNKNOWN",
               esp_err_to_name(ret));
      /* Continue cleanup even if deinit failed to avoid memory leaks */
    }
  }

  ESP_LOGI(TAG, "Destroying bus config '%s'", config->name ? config->name : "UNKNOWN");

  /* Clean up and free the configuration */
  priv_pstar_bus_config_cleanup(config);

  /* IMPORTANT: After this call, the config pointer is invalid. */
  return ESP_OK;
}

esp_err_t pstar_bus_config_init(pstar_bus_config_t* config)
{
  ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "Config pointer is NULL");
  ESP_RETURN_ON_FALSE(config->type == k_pstar_bus_type_i2c,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Config type is not I2C");
  ESP_RETURN_ON_FALSE(!config->initialized,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "Bus '%s' is already initialized",
                      config->name ? config->name : "UNKNOWN");

  esp_err_t   ret      = ESP_OK;
  const char* bus_name = config->name ? config->name : "UNKNOWN"; /* Use for logging */

  /* Initialize I2C bus */
  ESP_LOGI(TAG, "Initializing I2C bus '%s' (Port %d)", bus_name, config->i2c.port);

  ret = i2c_param_config(config->i2c.port, &config->i2c.config);
  ESP_GOTO_ON_ERROR(ret,
                    fail,
                    TAG,
                    "i2c_param_config failed for '%s': %s",
                    bus_name,
                    esp_err_to_name(ret));

  ret = i2c_driver_install(config->i2c.port,
                           config->i2c.config.mode,
                           0,  /* RX buffer size (master mode) */
                           0,  /* TX buffer size (master mode) */
                           0); /* Flags */
  ESP_GOTO_ON_ERROR(ret,
                    fail_i2c_driver,
                    TAG,
                    "i2c_driver_install failed for '%s': %s",
                    bus_name,
                    esp_err_to_name(ret));

  /* Mark as initialized */
  config->initialized = true;
  ESP_LOGI(TAG,
           "Successfully initialized bus '%s' (%s)",
           bus_name,
           pstar_bus_type_to_string(config->type));
  return ESP_OK;

/* Failure points with specific cleanup needs */
fail_i2c_driver:
  /* If driver install failed after param_config, no specific cleanup needed for param_config */
  goto fail;

/* Generic failure point */
fail:
  ESP_LOGE(TAG, "Failed to initialize bus '%s'", bus_name);
  return ret;
}

esp_err_t pstar_bus_config_deinit(pstar_bus_config_t* config)
{
  ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "Config pointer is NULL");
  ESP_RETURN_ON_FALSE(config->type == k_pstar_bus_type_i2c,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Config type is not I2C");
  ESP_RETURN_ON_FALSE(config->initialized,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "Bus '%s' is not initialized",
                      config->name ? config->name : "UNKNOWN");

  esp_err_t   ret      = ESP_OK;
  const char* bus_name = config->name ? config->name : "UNKNOWN";

  /* Deinitialize I2C bus */
  ESP_LOGI(TAG, "Deinitializing I2C bus '%s' (Port %d)", bus_name, config->i2c.port);
  ret = i2c_driver_delete(config->i2c.port);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to deinitialize bus '%s': %s", bus_name, esp_err_to_name(ret));
    /* Do not return early, mark as uninitialized anyway */
  }

  /* Mark as not initialized */
  config->initialized = false;
  config->handle      = NULL; /* Clear handle (though unused for I2C) */

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Successfully deinitialized bus '%s'", bus_name);
  }

  return ret; /* Return the ret of the deinit operation */
}

/* --- Private Functions --- */

/**
 * @brief Clean up and free memory for a bus configuration structure.
 *        Does NOT deinitialize the hardware/driver.
 * @param[in] config Pointer to the bus configuration to clean up.
 */
static void priv_pstar_bus_config_cleanup(pstar_bus_config_t* config)
{
  if (config == NULL) {
    return;
  }

  /* Free the name if it was allocated (strdup'd in create_common) */
  if (config->name) {
    free((void*)config->name); /* Cast needed as it's stored as const char* */
    config->name = NULL;
  }

  /* Clear other pointers */
  config->handle   = NULL;
  config->user_ctx = NULL;
  config->next     = NULL;

  /* Free the bus configuration structure itself */
  free(config);
}

/**
 * @brief Create a generic bus configuration structure and allocate memory.
 *
 * @param[in] name Name of the bus (will be duplicated). Must not be NULL.
 * @param[in] type Type of the bus.
 * @return pstar_bus_config_t* Pointer to the created configuration, or NULL on failure.
 */
static pstar_bus_config_t* priv_pstar_bus_config_create_common(const char*      name,
                                                               pstar_bus_type_t type)
{
  /* Name validation */
  if (name == NULL) {
    ESP_LOGE(TAG, "Bus name cannot be NULL");
    return NULL;
  }

  /* Allocate memory for the bus configuration */
  pstar_bus_config_t* config = (pstar_bus_config_t*)malloc(sizeof(pstar_bus_config_t));
  if (config == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for bus configuration");
    return NULL;
  }

  /* Initialize the configuration to zeros */
  memset(config, 0, sizeof(pstar_bus_config_t));

  /* Make a copy of the bus name */
  char* name_copy = strdup(name);
  if (name_copy == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for bus name");
    free(config);
    return NULL;
  }
  config->name = name_copy; /* Store the copy */

  /* Set common parameters */
  config->type        = type;
  config->initialized = false;
  config->handle      = NULL;
  config->user_ctx    = NULL;
  config->next        = NULL;

  /* Initialize callback structs to NULL (only I2C relevant now) */
  memset(&config->i2c.callbacks, 0, sizeof(config->i2c.callbacks));

  return config;
}
