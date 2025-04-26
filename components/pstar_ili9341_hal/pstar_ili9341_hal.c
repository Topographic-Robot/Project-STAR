/* components/pstar_ili9341_hal/pstar_ili9341_hal.c */

#include "pstar_ili9341_hal.h"

#include "pstar_bus_config.h"
#include "pstar_bus_spi.h"
#include "pstar_pin_validator.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" /* For mutex */
#include "freertos/task.h"   /* For vTaskDelay */

#include <stdlib.h> /* For malloc, free */
#include <string.h> /* For strdup, memset */

#include "esp_check.h"     /* For ESP_RETURN_ON_FALSE, ESP_GOTO_ON_ERROR */
#include "esp_heap_caps.h" /* For DMA-capable memory */
#include "esp_log.h"
#include "sdkconfig.h" /* Include sdkconfig for Kconfig defines */

static const char* TAG = "ILI9341 HAL";

/* --- ILI9341 Command Definitions --- */
#define ILI9341_CMD_NOP 0x00
#define ILI9341_CMD_SWRESET 0x01
#define ILI9341_CMD_RDDID 0x04
#define ILI9341_CMD_RDDST 0x09

#define ILI9341_CMD_SLPIN 0x10
#define ILI9341_CMD_SLPOUT 0x11
#define ILI9341_CMD_PTLON 0x12
#define ILI9341_CMD_NORON 0x13

#define ILI9341_CMD_RDMODE 0x0A
#define ILI9341_CMD_RDMADCTL 0x0B
#define ILI9341_CMD_RDPIXFMT 0x0C
#define ILI9341_CMD_RDIMGFMT 0x0D
#define ILI9341_CMD_RDSELFDIAG 0x0F

#define ILI9341_CMD_INVOFF 0x20
#define ILI9341_CMD_INVON 0x21
#define ILI9341_CMD_GAMMASET 0x26
#define ILI9341_CMD_DISPOFF 0x28
#define ILI9341_CMD_DISPON 0x29

#define ILI9341_CMD_CASET 0x2A
#define ILI9341_CMD_PASET 0x2B
#define ILI9341_CMD_RAMWR 0x2C
#define ILI9341_CMD_RAMRD 0x2E

#define ILI9341_CMD_PTLAR 0x30
#define ILI9341_CMD_MADCTL 0x36
#define ILI9341_CMD_PIXFMT 0x3A

#define ILI9341_CMD_FRMCTR1 0xB1
#define ILI9341_CMD_FRMCTR2 0xB2
#define ILI9341_CMD_FRMCTR3 0xB3
#define ILI9341_CMD_INVCTR 0xB4
#define ILI9341_CMD_DFUNCTR 0xB6

#define ILI9341_CMD_PWCTR1 0xC0
#define ILI9341_CMD_PWCTR2 0xC1
#define ILI9341_CMD_PWCTR3 0xC2
#define ILI9341_CMD_PWCTR4 0xC3
#define ILI9341_CMD_PWCTR5 0xC4
#define ILI9341_CMD_VMCTR1 0xC5
#define ILI9341_CMD_VMCTR2 0xC7

#define ILI9341_CMD_RDID1 0xDA
#define ILI9341_CMD_RDID2 0xDB
#define ILI9341_CMD_RDID3 0xDC
#define ILI9341_CMD_RDID4 0xDD

#define ILI9341_CMD_GMCTRP1 0xE0
#define ILI9341_CMD_GMCTRN1 0xE1

/* --- MADCTL Flags --- */
#define ILI9341_MADCTL_MY 0x80  /* Row address order */
#define ILI9341_MADCTL_MX 0x40  /* Column address order */
#define ILI9341_MADCTL_MV 0x20  /* Row/Column exchange */
#define ILI9341_MADCTL_ML 0x10  /* Vertical refresh order */
#define ILI9341_MADCTL_BGR 0x08 /* BGR color filter panel */
#define ILI9341_MADCTL_MH 0x04  /* Horizontal refresh order */

/* --- Constants --- */
#define ILI9341_MUTEX_TIMEOUT_MS 1000 /* Timeout for acquiring HAL mutex */
/* Max transaction size for DMA buffer (adjust based on memory) */
#define ILI9341_MAX_XFER_BYTES (ILI9341_WIDTH * ILI9341_HEIGHT * 2 / 4) /* Quarter screen */

/* --- Internal Handle Structure --- */
typedef struct pstar_ili9341_hal_dev_t {
  const pstar_bus_manager_t* bus_manager;      /**< Pointer to the bus manager */
  char*                      spi_bus_name;     /**< Name of the bus config (copied) */
  gpio_num_t                 dc_pin;           /**< Data/Command pin */
  gpio_num_t                 rst_pin;          /**< Reset pin (<0 if not used) */
  gpio_num_t                 blk_pin;          /**< Backlight pin (<0 if not used) */
  bool                       blk_active_low;   /**< Backlight active level */
  uint16_t                   width;            /**< Current width based on rotation */
  uint16_t                   height;           /**< Current height based on rotation */
  pstar_ili9341_rotation_t   current_rotation; /**< Current rotation setting */
  SemaphoreHandle_t          mutex;            /**< Mutex for thread-safe access */
  spi_device_handle_t        spi_handle;       /**< Cached SPI device handle */
} pstar_ili9341_hal_dev_t;

/* --- Helper Functions --- */

/**
 * @brief Send a command byte (DC low). Assumes mutex is taken.
 */
static esp_err_t priv_ili9341_send_command_nolock(pstar_ili9341_hal_handle_t handle, uint8_t cmd)
{
  /* Set DC pin low for command */
  esp_err_t ret = gpio_set_level(handle->dc_pin, 0);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set DC pin low");

  /* Transmit command via bus manager */
  ret = pstar_bus_spi_transmit(handle->bus_manager, handle->spi_bus_name, &cmd, 1, 0);
  return ret;
}

/**
 * @brief Send a data byte (DC high). Assumes mutex is taken.
 */
static esp_err_t priv_ili9341_send_data_nolock(pstar_ili9341_hal_handle_t handle, uint8_t data)
{
  /* Set DC pin high for data */
  esp_err_t ret = gpio_set_level(handle->dc_pin, 1);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set DC pin high");

  /* Transmit data via bus manager */
  ret = pstar_bus_spi_transmit(handle->bus_manager, handle->spi_bus_name, &data, 1, 0);
  return ret;
}

/**
 * @brief Send a block of data bytes (DC high). Assumes mutex is taken.
 * Uses DMA-capable buffer if possible.
 */
static esp_err_t priv_ili9341_send_data_block_nolock(pstar_ili9341_hal_handle_t handle,
                                                     const uint8_t*             data,
                                                     size_t                     len)
{
  if (len == 0) {
    return ESP_OK;
  }

  /* Set DC pin high for data */
  esp_err_t ret = gpio_set_level(handle->dc_pin, 1);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set DC pin high");

  /* Transmit data block via bus manager */
  ret = pstar_bus_spi_transmit(handle->bus_manager, handle->spi_bus_name, data, len, 0);
  return ret;
}

/**
 * @brief Perform hardware reset using RST pin. Assumes mutex is taken.
 */
static void priv_ili9341_hw_reset_nolock(pstar_ili9341_hal_handle_t handle)
{
  if (handle->rst_pin >= 0) {
    ESP_LOGD(TAG, "Performing hardware reset via pin %d", handle->rst_pin);
    gpio_set_level(handle->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(handle->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
  } else {
    ESP_LOGD(TAG, "Hardware reset skipped (RST pin not configured)");
    /* If no HW reset, send SW reset command */
    priv_ili9341_send_command_nolock(handle, ILI9341_CMD_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

/**
 * @brief Set the drawing window (address window). Assumes mutex is taken.
 */
static esp_err_t priv_ili9341_set_addr_window_nolock(pstar_ili9341_hal_handle_t handle,
                                                     uint16_t                   x0,
                                                     uint16_t                   y0,
                                                     uint16_t                   x1,
                                                     uint16_t                   y1)
{
  esp_err_t ret;
  uint8_t   data[4];

  /* Column Address Set */
  ret = priv_ili9341_send_command_nolock(handle, ILI9341_CMD_CASET);
  if (ret != ESP_OK)
    return ret;
  data[0] = (x0 >> 8) & 0xFF;
  data[1] = x0 & 0xFF;
  data[2] = (x1 >> 8) & 0xFF;
  data[3] = x1 & 0xFF;
  ret     = priv_ili9341_send_data_block_nolock(handle, data, 4);
  if (ret != ESP_OK)
    return ret;

  /* Page Address Set */
  ret = priv_ili9341_send_command_nolock(handle, ILI9341_CMD_PASET);
  if (ret != ESP_OK)
    return ret;
  data[0] = (y0 >> 8) & 0xFF;
  data[1] = y0 & 0xFF;
  data[2] = (y1 >> 8) & 0xFF;
  data[3] = y1 & 0xFF;
  ret     = priv_ili9341_send_data_block_nolock(handle, data, 4);
  if (ret != ESP_OK)
    return ret;

  /* Write to RAM */
  ret = priv_ili9341_send_command_nolock(handle, ILI9341_CMD_RAMWR);
  return ret;
}

/* --- Public Function Implementations --- */

esp_err_t pstar_ili9341_hal_init(const pstar_bus_manager_t*        manager,
                                 const pstar_ili9341_hal_config_t* config,
                                 pstar_ili9341_hal_handle_t*       out_handle)
{
  ESP_RETURN_ON_FALSE(manager && config && config->spi_bus_name && out_handle,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid arguments");
  ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(config->dc_pin),
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid DC pin");
  if (config->rst_pin >= 0) {
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(config->rst_pin),
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "Invalid RST pin");
  }
  if (config->blk_pin >= 0) {
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(config->blk_pin),
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "Invalid BLK pin");
  }
  *out_handle = NULL;

  /* Find the SPI device configuration in the manager */
  pstar_bus_config_t* bus_config = pstar_bus_manager_find_bus(manager, config->spi_bus_name);
  ESP_RETURN_ON_FALSE(bus_config,
                      ESP_ERR_NOT_FOUND,
                      TAG,
                      "SPI device '%s' not found in manager",
                      config->spi_bus_name);
  ESP_RETURN_ON_FALSE(bus_config->initialized,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "SPI device '%s' is not initialized",
                      config->spi_bus_name);
  ESP_RETURN_ON_FALSE(bus_config->type == k_pstar_bus_type_spi,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Bus '%s' is not SPI",
                      config->spi_bus_name);
  ESP_RETURN_ON_FALSE(bus_config->handle != NULL,
                      ESP_ERR_INVALID_STATE,
                      TAG,
                      "SPI device handle for '%s' is NULL",
                      config->spi_bus_name);

  /* Allocate memory for the device handle */
  pstar_ili9341_hal_handle_t dev =
    (pstar_ili9341_hal_handle_t)malloc(sizeof(pstar_ili9341_hal_dev_t));
  ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Failed to allocate device handle");
  memset(dev, 0, sizeof(pstar_ili9341_hal_dev_t));

  dev->bus_manager  = manager;
  dev->spi_bus_name = strdup(config->spi_bus_name); /* Store a copy of the name */
  if (!dev->spi_bus_name) {
    ESP_LOGE(TAG, "Failed to duplicate SPI bus name");
    free(dev);
    return ESP_ERR_NO_MEM;
  }
  dev->spi_handle     = (spi_device_handle_t)bus_config->handle; /* Cache the handle */
  dev->dc_pin         = config->dc_pin;
  dev->rst_pin        = config->rst_pin;
  dev->blk_pin        = config->blk_pin;
  dev->blk_active_low = config->blk_active_low; /* Copy from config struct */
  /* Initial dimensions will be set by set_rotation */
  dev->width  = 0;
  dev->height = 0;

  /* Create mutex */
  dev->mutex = xSemaphoreCreateMutex();
  if (!dev->mutex) {
    ESP_LOGE(TAG, "Failed to create mutex for handle '%s'", dev->spi_bus_name);
    free(dev->spi_bus_name);
    free(dev);
    return ESP_ERR_NO_MEM;
  }

  /* Take mutex for the rest of initialization */
  if (xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(ILI9341_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex during init for '%s'", dev->spi_bus_name);
    vSemaphoreDelete(dev->mutex);
    free(dev->spi_bus_name);
    free(dev);
    return ESP_ERR_TIMEOUT;
  }

  esp_err_t ret = ESP_OK;

  /* --- Configure GPIOs --- */
  gpio_config_t io_conf = {
    .mode         = GPIO_MODE_OUTPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE,
  };

  /* Configure DC Pin */
  io_conf.pin_bit_mask = (1ULL << dev->dc_pin);
  ret                  = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure DC pin %d: %s", dev->dc_pin, esp_err_to_name(ret));
    goto init_fail_mutex;
  }

  /* Configure RST Pin (if used) */
  if (dev->rst_pin >= 0) {
    io_conf.pin_bit_mask = (1ULL << dev->rst_pin);
    ret                  = gpio_config(&io_conf);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to configure RST pin %d: %s", dev->rst_pin, esp_err_to_name(ret));
      goto init_fail_mutex;
    }
  }

  /* Configure BLK Pin (if used) */
  if (dev->blk_pin >= 0) {
    io_conf.pin_bit_mask = (1ULL << dev->blk_pin);
    ret                  = gpio_config(&io_conf);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to configure BLK pin %d: %s", dev->blk_pin, esp_err_to_name(ret));
      goto init_fail_mutex;
    }
    /* Set initial backlight state (OFF) */
    ret = gpio_set_level(dev->blk_pin, dev->blk_active_low ? 1 : 0);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to set initial BLK pin level: %s", esp_err_to_name(ret));
      goto init_fail_mutex;
    }
  }

  /* --- ILI9341 Initialization Sequence --- */
  ESP_LOGI(TAG, "Initializing ILI9341 display on SPI bus '%s'...", dev->spi_bus_name);

  priv_ili9341_hw_reset_nolock(dev); /* Hardware or Software Reset */

  /* Exit Sleep Mode */
  ret = priv_ili9341_send_command_nolock(dev, ILI9341_CMD_SLPOUT);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  vTaskDelay(pdMS_TO_TICKS(120)); /* Wait >120ms */

  /* Power Control Settings (Typical values, may need tuning) */
  ret = priv_ili9341_send_command_nolock(dev, ILI9341_CMD_PWCTR1);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  ret = priv_ili9341_send_data_nolock(dev, 0x23); /* VRH[5:0] */
  if (ret != ESP_OK)
    goto init_fail_mutex;

  ret = priv_ili9341_send_command_nolock(dev, ILI9341_CMD_PWCTR2);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  ret = priv_ili9341_send_data_nolock(dev, 0x10); /* SAP[2:0];BT[3:0] */
  if (ret != ESP_OK)
    goto init_fail_mutex;

  ret = priv_ili9341_send_command_nolock(dev, ILI9341_CMD_VMCTR1);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  ret = priv_ili9341_send_data_nolock(dev, 0x3e);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  ret = priv_ili9341_send_data_nolock(dev, 0x28);
  if (ret != ESP_OK)
    goto init_fail_mutex;

  ret = priv_ili9341_send_command_nolock(dev, ILI9341_CMD_VMCTR2);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  ret = priv_ili9341_send_data_nolock(dev, 0x86); /* --/-- */
  if (ret != ESP_OK)
    goto init_fail_mutex;

  /* Memory Access Control (Set default rotation later) */
  ret = priv_ili9341_send_command_nolock(dev, ILI9341_CMD_MADCTL);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  ret = priv_ili9341_send_data_nolock(dev, ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR);
  if (ret != ESP_OK)
    goto init_fail_mutex;

  /* Pixel Format Set */
  ret = priv_ili9341_send_command_nolock(dev, ILI9341_CMD_PIXFMT);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  ret = priv_ili9341_send_data_nolock(dev, 0x55); /* 16 bits/pixel */
  if (ret != ESP_OK)
    goto init_fail_mutex;

  /* Frame Rate Control (Typical values) */
  ret = priv_ili9341_send_command_nolock(dev, ILI9341_CMD_FRMCTR1);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  ret = priv_ili9341_send_data_nolock(dev, 0x00);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  ret = priv_ili9341_send_data_nolock(dev, 0x18);
  if (ret != ESP_OK)
    goto init_fail_mutex;

  /* Display Function Control (Typical values) */
  ret = priv_ili9341_send_command_nolock(dev, ILI9341_CMD_DFUNCTR);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  ret = priv_ili9341_send_data_nolock(dev, 0x08);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  ret = priv_ili9341_send_data_nolock(dev, 0x82);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  ret = priv_ili9341_send_data_nolock(dev, 0x27);
  if (ret != ESP_OK)
    goto init_fail_mutex;

  /* Gamma Set (Disable) */
  ret = priv_ili9341_send_command_nolock(dev, ILI9341_CMD_GAMMASET);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  ret = priv_ili9341_send_data_nolock(dev, 0x01); /* Gamma curve 1 */
  if (ret != ESP_OK)
    goto init_fail_mutex;

  /* Positive Gamma Correction (Typical values) */
  ret = priv_ili9341_send_command_nolock(dev, ILI9341_CMD_GMCTRP1);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  const uint8_t gamma_p[] =
    {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00};
  ret = priv_ili9341_send_data_block_nolock(dev, gamma_p, sizeof(gamma_p));
  if (ret != ESP_OK)
    goto init_fail_mutex;

  /* Negative Gamma Correction (Typical values) */
  ret = priv_ili9341_send_command_nolock(dev, ILI9341_CMD_GMCTRN1);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  const uint8_t gamma_n[] =
    {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F};
  ret = priv_ili9341_send_data_block_nolock(dev, gamma_n, sizeof(gamma_n));
  if (ret != ESP_OK)
    goto init_fail_mutex;

  /* Display ON */
  ret = priv_ili9341_send_command_nolock(dev, ILI9341_CMD_DISPON);
  if (ret != ESP_OK)
    goto init_fail_mutex;
  vTaskDelay(pdMS_TO_TICKS(100));

  /* Set initial rotation and clear screen */
  xSemaphoreGive(dev->mutex); /* Release mutex before calling public functions */
  ret = pstar_ili9341_hal_set_rotation(dev, config->rotation);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set initial rotation: %s", esp_err_to_name(ret));
    /* Don't fail init completely, but log error */
  }
  ret = pstar_ili9341_hal_fill_screen(dev, 0x0000); /* Clear to black */
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to clear screen during init: %s", esp_err_to_name(ret));
    /* Don't fail init completely */
  }

  /* Turn on backlight if configured */
  ret = pstar_ili9341_hal_set_backlight(dev, true);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { /* Ignore error if BLK not configured */
    ESP_LOGE(TAG, "Failed to turn on backlight during init: %s", esp_err_to_name(ret));
  }

  ESP_LOGI(TAG, "ILI9341 Initialized Successfully on SPI bus '%s'", dev->spi_bus_name);
  *out_handle = dev;
  return ESP_OK;

init_fail_mutex:
  xSemaphoreGive(dev->mutex); /* Release mutex on failure */
  ESP_LOGE(TAG, "ILI9341 initialization failed on SPI bus '%s'", dev->spi_bus_name);
  if (dev) {
    if (dev->mutex) {
      vSemaphoreDelete(dev->mutex);
    }
    if (dev->spi_bus_name) {
      free(dev->spi_bus_name);
    }
    free(dev);
  }
  *out_handle = NULL;
  return ret;
}

esp_err_t pstar_ili9341_hal_deinit(pstar_ili9341_hal_handle_t handle, bool turn_off_backlight)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_LOGI(TAG,
           "Deinitializing ILI9341 HAL for SPI bus '%s'",
           handle->spi_bus_name ? handle->spi_bus_name : "UNKNOWN");

  /* Turn off backlight if requested and possible */
  if (turn_off_backlight) {
    pstar_ili9341_hal_set_backlight(handle, false); /* Ignore return value */
  }

  /* Take mutex for final cleanup */
  if (handle->mutex &&
      xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(ILI9341_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for final deinit cleanup");
    /* Try to continue cleanup anyway */
  }

  /* Reset GPIO pins */
  gpio_reset_pin(handle->dc_pin);
  if (handle->rst_pin >= 0) {
    gpio_reset_pin(handle->rst_pin);
  }
  if (handle->blk_pin >= 0) {
    gpio_reset_pin(handle->blk_pin);
  }

  /* Release and delete mutex */
  if (handle->mutex) {
    xSemaphoreGive(handle->mutex);
    vSemaphoreDelete(handle->mutex);
    handle->mutex = NULL;
  }

  if (handle->spi_bus_name) {
    free(handle->spi_bus_name);
  }
  free(handle);
  return ESP_OK;
}

esp_err_t pstar_ili9341_hal_send_command(pstar_ili9341_hal_handle_t handle, uint8_t command)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(ILI9341_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for send_command");
    return ESP_ERR_TIMEOUT;
  }

  ret = priv_ili9341_send_command_nolock(handle, command);

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_ili9341_hal_send_data(pstar_ili9341_hal_handle_t handle, uint8_t data)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(ILI9341_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for send_data");
    return ESP_ERR_TIMEOUT;
  }

  ret = priv_ili9341_send_data_nolock(handle, data);

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_ili9341_hal_send_data_block(pstar_ili9341_hal_handle_t handle,
                                            const uint8_t*             data,
                                            size_t                     len)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(data, ESP_ERR_INVALID_ARG, TAG, "Data buffer is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  if (len == 0) {
    return ESP_OK;
  }

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(ILI9341_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for send_data_block");
    return ESP_ERR_TIMEOUT;
  }

  ret = priv_ili9341_send_data_block_nolock(handle, data, len);

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_ili9341_hal_fill_rect(pstar_ili9341_hal_handle_t handle,
                                      int16_t                    x,
                                      int16_t                    y,
                                      int16_t                    w,
                                      int16_t                    h,
                                      uint16_t                   color)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  if (w <= 0 || h <= 0 || x >= handle->width || y >= handle->height) {
    return ESP_OK; /* Nothing to draw */
  }

  /* Clamp coordinates */
  if ((x + w - 1) >= handle->width)
    w = handle->width - x;
  if ((y + h - 1) >= handle->height)
    h = handle->height - y;

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(ILI9341_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for fill_rect");
    return ESP_ERR_TIMEOUT;
  }

  /* Set address window */
  ret = priv_ili9341_set_addr_window_nolock(handle, x, y, x + w - 1, y + h - 1);
  if (ret != ESP_OK) {
    goto fill_rect_fail;
  }

  /* Prepare color data (swap bytes for SPI) */
  uint8_t color_data[2] = {(color >> 8) & 0xFF, color & 0xFF};
  size_t  total_pixels  = (size_t)w * (size_t)h;

  /* Set DC pin high for data */
  ret = gpio_set_level(handle->dc_pin, 1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed set DC pin high for fill_rect: %s", esp_err_to_name(ret));
    goto fill_rect_fail;
  }

  /* Send color data repeatedly */
  /* Optimization: Use larger buffer for fewer SPI transactions */
  size_t   buffer_size  = 1024; /* Use a buffer (e.g., 1KB) */
  uint8_t* color_buffer = (uint8_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  if (!color_buffer) {
    ESP_LOGE(TAG, "Failed to allocate DMA buffer for fill_rect");
    ret = ESP_ERR_NO_MEM;
    goto fill_rect_fail;
  }

  /* Fill the buffer with the color */
  for (size_t i = 0; i < buffer_size / 2; ++i) {
    color_buffer[i * 2]     = color_data[0];
    color_buffer[i * 2 + 1] = color_data[1];
  }

  size_t pixels_sent = 0;
  while (pixels_sent < total_pixels) {
    size_t pixels_to_send = total_pixels - pixels_sent;
    size_t bytes_to_send  = pixels_to_send * 2;
    if (bytes_to_send > buffer_size) {
      bytes_to_send = buffer_size;
    }
    ret = pstar_bus_spi_transmit(handle->bus_manager,
                                 handle->spi_bus_name,
                                 color_buffer,
                                 bytes_to_send,
                                 0);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "SPI transmit failed during fill_rect: %s", esp_err_to_name(ret));
      break;
    }
    pixels_sent += (bytes_to_send / 2);
  }

  heap_caps_free(color_buffer);

fill_rect_fail:
  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_ili9341_hal_fill_screen(pstar_ili9341_hal_handle_t handle, uint16_t color)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  /* Mutex is handled by fill_rect */
  return pstar_ili9341_hal_fill_rect(handle, 0, 0, handle->width, handle->height, color);
}

esp_err_t pstar_ili9341_hal_set_rotation(pstar_ili9341_hal_handle_t handle,
                                         pstar_ili9341_rotation_t   rotation)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(ILI9341_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for set_rotation");
    return ESP_ERR_TIMEOUT;
  }

  uint8_t madctl_val = 0;
  rotation %= 4; /* Normalize rotation value */
  handle->current_rotation = rotation;

  switch (rotation) {
    case ILI9341_ROTATION_0: /* Portrait */
      madctl_val     = ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR;
      handle->width  = ILI9341_WIDTH;
      handle->height = ILI9341_HEIGHT;
      break;
    case ILI9341_ROTATION_90: /* Landscape */
      madctl_val     = ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
      handle->width  = ILI9341_HEIGHT;
      handle->height = ILI9341_WIDTH;
      break;
    case ILI9341_ROTATION_180: /* Portrait Upside Down */
      madctl_val     = ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR;
      handle->width  = ILI9341_WIDTH;
      handle->height = ILI9341_HEIGHT;
      break;
    case ILI9341_ROTATION_270: /* Landscape Upside Down */
      madctl_val = ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
      handle->width  = ILI9341_HEIGHT;
      handle->height = ILI9341_WIDTH;
      break;
  }

  ret = priv_ili9341_send_command_nolock(handle, ILI9341_CMD_MADCTL);
  if (ret == ESP_OK) {
    ret = priv_ili9341_send_data_nolock(handle, madctl_val);
  }

  if (ret == ESP_OK) {
    ESP_LOGD(TAG, "Set rotation to %d", rotation);
  } else {
    ESP_LOGE(TAG, "Failed to set rotation: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

esp_err_t pstar_ili9341_hal_set_backlight(pstar_ili9341_hal_handle_t handle, bool on)
{
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Device handle is NULL");
  ESP_RETURN_ON_FALSE(handle->mutex, ESP_ERR_INVALID_STATE, TAG, "Handle mutex not initialized");

  esp_err_t ret = ESP_OK;

  if (handle->blk_pin < 0) {
    ESP_LOGD(TAG, "Backlight control requested, but BLK pin not configured.");
    return ESP_ERR_INVALID_STATE;
  }

  if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(ILI9341_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout acquiring mutex for set_backlight");
    return ESP_ERR_TIMEOUT;
  }

  uint32_t level = on ? (handle->blk_active_low ? 0 : 1) : (handle->blk_active_low ? 1 : 0);
  ret            = gpio_set_level(handle->blk_pin, level);

  if (ret == ESP_OK) {
    ESP_LOGD(TAG, "Set backlight %s", on ? "ON" : "OFF");
  } else {
    ESP_LOGE(TAG, "Failed to set backlight: %s", esp_err_to_name(ret));
  }

  xSemaphoreGive(handle->mutex);
  return ret;
}

/* --- Convenience Creation Functions --- */

esp_err_t pstar_ili9341_hal_create_kconfig_default(pstar_bus_manager_t*        manager,
                                                   pstar_ili9341_hal_handle_t* out_handle)
{
#if !CONFIG_PSTAR_KCONFIG_ILI9341_ENABLED
  ESP_LOGE(TAG, "ILI9341 component is disabled in Kconfig.");
  if (out_handle)
    *out_handle = NULL;
  return ESP_ERR_NOT_SUPPORTED;
#else
  ESP_RETURN_ON_FALSE(manager && out_handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

  *out_handle                            = NULL;
  esp_err_t           ret                = ESP_OK;
  bool                config_created     = false;
  bool                config_added       = false;
  pstar_bus_config_t* ili9341_spi_config = NULL;
  const char*         bus_name           = CONFIG_PSTAR_KCONFIG_ILI9341_SPI_BUS_NAME;

  ESP_LOGI(TAG, "Creating ILI9341 with KConfig default configuration");

  /* 1. Create SPI Device Configuration using KConfig values */
  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = CONFIG_PSTAR_KCONFIG_ILI9341_SPI_FREQ_HZ,
    .mode           = CONFIG_PSTAR_KCONFIG_ILI9341_SPI_MODE,
    .spics_io_num   = CONFIG_PSTAR_KCONFIG_ILI9341_CS_PIN,
    .queue_size     = 7, /* Queue 7 transactions */
    /* We'll handle DC pin manually before transactions */
    .flags = 0,
    /* .pre_cb = NULL, // Can use this for DC pin control if desired */
    /* .user = NULL, */
  };

  /* Use the renamed parameters posi_pin, piso_pin when calling create */
  ili9341_spi_config = pstar_bus_config_create_spi_device(bus_name,
                                                          CONFIG_PSTAR_KCONFIG_ILI9341_SPI_HOST,
                                                          CONFIG_PSTAR_KCONFIG_ILI9341_POSI_PIN,
                                                          CONFIG_PSTAR_KCONFIG_ILI9341_PISO_PIN,
                                                          CONFIG_PSTAR_KCONFIG_ILI9341_SCLK_PIN,
                                                          SPI_DMA_CH_AUTO,
                                                          &devcfg);

  ESP_GOTO_ON_FALSE(ili9341_spi_config != NULL,
                    ESP_ERR_NO_MEM,
                    cleanup,
                    TAG,
                    "Failed to create ILI9341 SPI config");
  config_created = true;

  /* 2. Add Configuration to the Manager */
  ret = pstar_bus_manager_add_bus(manager, ili9341_spi_config);
  if (ret == ESP_ERR_INVALID_STATE) { /* Already exists */
    ESP_LOGW(TAG, "Bus config '%s' already exists, finding...", bus_name);
    ili9341_spi_config = pstar_bus_manager_find_bus(manager, bus_name);
    ESP_GOTO_ON_FALSE(ili9341_spi_config,
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
                      "Failed to add ILI9341 config to manager: %s",
                      esp_err_to_name(ret));
    config_added   = true;
    config_created = false;
  }

  /* 3. Initialize the bus hardware */
  if (!ili9341_spi_config->initialized) {
    // *** CORRECTED LINE ***
    ret = pstar_bus_config_init(ili9341_spi_config, manager);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed to initialize bus hardware for '%s': %s",
                      bus_name,
                      esp_err_to_name(ret));
  }

  /* 4. Initialize the ILI9341 HAL Component with KConfig settings */
  pstar_ili9341_hal_config_t hal_config = {
    .spi_bus_name = bus_name,
    .dc_pin       = CONFIG_PSTAR_KCONFIG_ILI9341_DC_PIN,
    .rst_pin      = CONFIG_PSTAR_KCONFIG_ILI9341_RST_PIN,
    .blk_pin      = CONFIG_PSTAR_KCONFIG_ILI9341_BLK_PIN,
    /* --- CORRECTED KCONFIG USAGE --- */
    .blk_active_low =
#if CONFIG_PSTAR_KCONFIG_ILI9341_BLK_PIN >= 0 /* Check if pin is enabled first */
      CONFIG_PSTAR_KCONFIG_ILI9341_BLK_IS_ACTIVE_LOW, /* Use the corrected hidden config */
#else
      false, /* Default if BLK pin is disabled */
#endif
    /* --- End Correction --- */
    .width    = 0, /* Will be set by rotation */
    .height   = 0,
    .rotation = CONFIG_PSTAR_KCONFIG_ILI9341_ROTATION,
  };

  ret = pstar_ili9341_hal_init(manager, &hal_config, out_handle);
  ESP_GOTO_ON_ERROR(ret,
                    cleanup,
                    TAG,
                    "Failed to initialize ILI9341 HAL: %s",
                    esp_err_to_name(ret));

  ESP_LOGI(TAG, "ILI9341 created and initialized successfully with KConfig defaults");
  return ESP_OK;

cleanup:
  if (config_created && !config_added) {
    ESP_LOGW(TAG, "Destroying config that was created but not added to manager");
    pstar_bus_config_destroy(ili9341_spi_config);
  }
  *out_handle = NULL;
  return ret;
#endif /* CONFIG_PSTAR_KCONFIG_ILI9341_ENABLED */
}

esp_err_t pstar_ili9341_hal_create_custom(pstar_bus_manager_t*        manager,
                                          const char*                 spi_bus_name,
                                          spi_host_device_t           host,
                                          int                         clk_speed_hz,
                                          uint8_t                     spi_mode,
                                          gpio_num_t                  posi_pin,
                                          gpio_num_t                  sclk_pin,
                                          gpio_num_t                  cs_pin,
                                          gpio_num_t                  dc_pin,
                                          gpio_num_t                  rst_pin,
                                          gpio_num_t                  blk_pin,
                                          bool                        blk_active_low,
                                          pstar_ili9341_rotation_t    rotation,
                                          pstar_ili9341_hal_handle_t* out_handle)
{
  ESP_RETURN_ON_FALSE(manager && spi_bus_name && out_handle,
                      ESP_ERR_INVALID_ARG,
                      TAG,
                      "Invalid arguments");

  *out_handle                            = NULL;
  esp_err_t           ret                = ESP_OK;
  bool                config_created     = false;
  bool                config_added       = false;
  pstar_bus_config_t* ili9341_spi_config = NULL;

  ESP_LOGI(TAG, "Creating ILI9341 with custom configuration for bus '%s'", spi_bus_name);

  /* 1. Create SPI Device Configuration */
  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = clk_speed_hz,
    .mode           = spi_mode,
    .spics_io_num   = cs_pin,
    .queue_size     = 7,
    .flags          = 0,
  };

  /* Use the renamed parameters posi_pin, piso_pin when calling create */
  ili9341_spi_config = pstar_bus_config_create_spi_device(spi_bus_name,
                                                          host,
                                                          posi_pin,
                                                          -1, /* PISO not used by ILI9341 */
                                                          sclk_pin,
                                                          SPI_DMA_CH_AUTO,
                                                          &devcfg);

  ESP_GOTO_ON_FALSE(ili9341_spi_config != NULL,
                    ESP_ERR_NO_MEM,
                    cleanup,
                    TAG,
                    "Failed create custom ILI9341 SPI config");
  config_created = true;

  /* 2. Add Config to Manager */
  ret = pstar_bus_manager_add_bus(manager, ili9341_spi_config);
  if (ret == ESP_ERR_INVALID_STATE) { /* Already exists */
    ESP_LOGW(TAG, "Custom bus config '%s' already exists, finding...", spi_bus_name);
    ili9341_spi_config = pstar_bus_manager_find_bus(manager, spi_bus_name);
    ESP_GOTO_ON_FALSE(ili9341_spi_config,
                      ESP_FAIL,
                      cleanup,
                      TAG,
                      "Failed find existing custom bus '%s'",
                      spi_bus_name);
    config_added   = true;
    config_created = false;
    ret            = ESP_OK;
  } else {
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed add custom ILI9341 config: %s",
                      esp_err_to_name(ret));
    config_added   = true;
    config_created = false;
  }

  /* 3. Initialize Bus Hardware */
  if (!ili9341_spi_config->initialized) {
    // *** CORRECTED LINE ***
    ret = pstar_bus_config_init(ili9341_spi_config, manager);
    ESP_GOTO_ON_ERROR(ret,
                      cleanup,
                      TAG,
                      "Failed init custom ILI9341 bus '%s': %s",
                      spi_bus_name,
                      esp_err_to_name(ret));
  }

  /* 4. Initialize HAL */
  pstar_ili9341_hal_config_t hal_config = {.spi_bus_name   = spi_bus_name,
                                           .dc_pin         = dc_pin,
                                           .rst_pin        = rst_pin,
                                           .blk_pin        = blk_pin,
                                           .blk_active_low = blk_active_low,
                                           .width          = 0,
                                           .height         = 0,
                                           .rotation       = rotation};
  ret                                   = pstar_ili9341_hal_init(manager, &hal_config, out_handle);
  ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed init custom ILI9341 HAL: %s", esp_err_to_name(ret));

  ESP_LOGI(TAG, "ILI9341 (Custom) created and initialized successfully for bus '%s'", spi_bus_name);
  return ESP_OK;

cleanup:
  if (config_created && !config_added) {
    pstar_bus_config_destroy(ili9341_spi_config);
  }
  *out_handle = NULL;
  return ret;
}

/* --- Pin Registration Functions --- */

esp_err_t pstar_ili9341_register_kconfig_pins(void)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED && CONFIG_PSTAR_KCONFIG_ILI9341_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering ILI9341 pins with validator (from KConfig)...");

  ret = pstar_ili9341_register_custom_pins(CONFIG_PSTAR_KCONFIG_ILI9341_POSI_PIN,
                                           CONFIG_PSTAR_KCONFIG_ILI9341_PISO_PIN,
                                           CONFIG_PSTAR_KCONFIG_ILI9341_SCLK_PIN,
                                           CONFIG_PSTAR_KCONFIG_ILI9341_CS_PIN,
                                           CONFIG_PSTAR_KCONFIG_ILI9341_DC_PIN,
                                           CONFIG_PSTAR_KCONFIG_ILI9341_RST_PIN,
                                           CONFIG_PSTAR_KCONFIG_ILI9341_BLK_PIN);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register ILI9341 KConfig pins");

  ESP_LOGI(TAG, "ILI9341 KConfig pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator or ILI9341 disabled, skipping ILI9341 KConfig pin registration.");
  return ESP_OK;
#endif
}

esp_err_t pstar_ili9341_register_custom_pins(int posi_pin,
                                             int piso_pin,
                                             int sclk_pin,
                                             int cs_pin,
                                             int dc_pin,
                                             int rst_pin,
                                             int blk_pin)
{
#if CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "Registering custom ILI9341 pins with validator...");

  /* SPI bus pins (POSI, PISO, SCLK) are potentially shared */
  ret = pstar_register_pin(posi_pin, "ILI9341 Custom POSI", true);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom POSI pin (%d)", posi_pin);
  if (piso_pin >= 0) { /* Only register PISO if used */
    ret = pstar_register_pin(piso_pin, "ILI9341 Custom PISO", true);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom PISO pin (%d)", piso_pin);
  }
  ret = pstar_register_pin(sclk_pin, "ILI9341 Custom SCLK", true);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom SCLK pin (%d)", sclk_pin);

  /* Device specific pins (CS, DC, RST, BLK) are typically not shared */
  ret = pstar_register_pin(cs_pin, "ILI9341 Custom CS", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom CS pin (%d)", cs_pin);
  ret = pstar_register_pin(dc_pin, "ILI9341 Custom DC", false);
  ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom DC pin (%d)", dc_pin);

  if (rst_pin >= 0) {
    ret = pstar_register_pin(rst_pin, "ILI9341 Custom RST", false);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom RST pin (%d)", rst_pin);
  }
  if (blk_pin >= 0) {
    ret = pstar_register_pin(blk_pin, "ILI9341 Custom BLK", false);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed register custom BLK pin (%d)", blk_pin);
  }

  ESP_LOGI(TAG, "ILI9341 custom pins registered successfully.");
  return ESP_OK;
#else
  ESP_LOGD(TAG, "Pin validator disabled, skipping ILI9341 custom pin registration.");
  return ESP_OK;
#endif
}