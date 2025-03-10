/* components/storage/sd_card_hal/sd_card_hal.c */

/* TODO: SDIO */
/* TODO: Error handler */

#include "sd_card_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "error_handler.h"
#include "log_handler.h"
#include "driver/gpio.h"
#include "common/bus_manager.h"
#include "common/common_setup.h"
#include "common/common_cleanup.h"

/* Constants ******************************************************************/

const char* const       sd_card_tag                  = "SD Card";
const char* const       sd_card_mount_path           = "/sdcard";
const uint8_t           sd_card_cs                   = GPIO_NUM_5;
const uint8_t           sd_card_data_to_card         = GPIO_NUM_23;
const uint8_t           sd_card_clk                  = GPIO_NUM_14;
const uint8_t           sd_card_data_from_card       = GPIO_NUM_19;
const uint8_t           sd_card_cd                   = GPIO_NUM_13; /* Card Detect pin */
const uint32_t          sd_card_spi_freq_hz          = 1000000;     /* 1 MHz SPI frequency */
const spi_host_device_t sd_card_spi_host             = SPI2_HOST;
const uint8_t           sd_card_max_files            = 5;
const uint32_t          sd_card_allocation_unit_size = 16 * 1024;
const uint32_t          sd_card_max_transfer_sz      = 4092;        /* Default size in Bytes */
const uint8_t           sd_card_max_retries          = 5;
const uint32_t          sd_card_retry_delay_ms       = 500;         /* 500ms delay between retries */
const uint32_t          sd_card_debounce_ms          = 100;         /* Debounce time for CD pin in ms */

/* Private Variables **********************************************************/

static sdmmc_card_t*                   s_card                  = NULL;  /**< Pointer to hold SD card descriptor */
static SemaphoreHandle_t               s_sd_mutex              = NULL;  /**< Mutex for thread-safe access */
static bool                            s_sd_card_available     = false; /**< Flag to track SD card availability */
static bool                            s_sd_card_initialized   = false; /**< Flag to track initialization status */
static sd_card_availability_callback_t s_availability_callback = NULL;  /**< Callback for SD card availability changes */
static TaskHandle_t                    s_mount_task_handle     = NULL;  /**< Handle for the mount/unmount task */
static error_handler_t                 s_sd_error_handler;              /**< Error handler for SD card operations */

/* Private (Static) Functions *************************************************/

/**
 * @brief Internal function to unmount and cleanup resources.
 */
static void priv_sd_card_cleanup(void)
{
  if (s_card != NULL) {
    esp_err_t ret = esp_vfs_fat_sdcard_unmount(sd_card_mount_path, s_card);
    if (ret != ESP_OK) {
      ERROR_HARDWARE(&s_sd_error_handler, ret, k_error_severity_medium, 
                    "Failed to unmount SD card");
      log_error(sd_card_tag, "Unmount Error", "Failed to unmount SD card: %s", esp_err_to_name(ret));
    } else {
      log_info(sd_card_tag, "Unmount Success", "SD card unmounted successfully");
    }
    s_card = NULL;
  }
  
  /* Deinitialize SPI bus */
  esp_err_t ret = bus_manager_spi_cleanup(sd_card_spi_host);
  if (ret != ESP_OK) {
    ERROR_HARDWARE(&s_sd_error_handler, ret, k_error_severity_low, 
                  "Failed to deinitialize SPI bus");
    log_warn(sd_card_tag, "SPI Warning", "Failed to deinitialize SPI bus: %s", esp_err_to_name(ret));
  }
}

/**
 * @brief Task that handles mounting/unmounting the SD card when it's inserted/removed.
 */
static void priv_sd_card_mount_task(void* arg)
{
  bool last_card_state = false;
  
  while (1) {
    /* Wait for notification from ISR */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    /* Debounce */
    vTaskDelay(pdMS_TO_TICKS(sd_card_debounce_ms));
    
    /* Check card state */
    bool card_present = !gpio_get_level(sd_card_cd);
    
    /* Only process if state changed */
    if (card_present != last_card_state) {
      last_card_state = card_present;
      
      if (xSemaphoreTake(s_sd_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (card_present && !s_sd_card_available) {
          /* Card inserted, mount it */
          log_info(sd_card_tag, "Card Inserted", "SD card detected, mounting...");
          
          esp_err_t ret = sd_card_init();
          if (ret == ESP_OK) {
            s_sd_card_available = true;
            log_info(sd_card_tag, "Mount Success", "SD card mounted successfully");
          } else {
            ERROR_HARDWARE(&s_sd_error_handler, ret, k_error_severity_medium, 
                          "Failed to mount SD card after insertion");
            log_error(sd_card_tag, "Mount Error", "Failed to mount SD card: %s", esp_err_to_name(ret));
          }
        } else if (!card_present && s_sd_card_available) {
          /* Card removed, unmount it */
          log_info(sd_card_tag, "Card Removed", "SD card removed, unmounting...");
          
          priv_sd_card_cleanup();
          s_sd_card_available = false;
          log_info(sd_card_tag, "Unmount Complete", "SD card unmounted after removal");
        }
        
        /* Notify callback if registered */
        if (s_availability_callback != NULL) {
          s_availability_callback(s_sd_card_available);
        }
        
        xSemaphoreGive(s_sd_mutex);
      }
    }
  }
}

/**
 * @brief ISR handler for card detect pin.
 */
static void IRAM_ATTR priv_sd_card_isr_handler(void* arg)
{
  if (s_mount_task_handle != NULL) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(s_mount_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/**
 * @brief Reset function for error handler.
 */
static esp_err_t priv_sd_card_reset(void* context)
{
  log_info(sd_card_tag, "Reset", "Attempting to reset SD card subsystem");
  
  /* Cleanup existing resources */
  priv_sd_card_cleanup();
  
  /* Reinitialize */
  return sd_card_init();
}

/**
 * @brief Initialize the card detect pin.
 */
static esp_err_t priv_sd_card_init_cd_pin(void)
{
  /* Configure CD pin using common_setup */
  uint64_t pin_bit_mask = (1ULL << sd_card_cd);
  esp_err_t ret = common_setup_gpio(pin_bit_mask, 
                                   GPIO_MODE_INPUT,
                                   GPIO_PULLUP_ENABLE,
                                   GPIO_PULLDOWN_DISABLE,
                                   GPIO_INTR_ANYEDGE,
                                   sd_card_tag);
  
  if (ret != ESP_OK) {
    ERROR_HARDWARE(&s_sd_error_handler, ret, k_error_severity_high, 
                  "Failed to configure CD pin");
    return ESP_FAIL;
  }
  
  /* Install GPIO ISR service */
  ret = gpio_install_isr_service(0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    /* ESP_ERR_INVALID_STATE means the service is already installed, which is fine */
    ERROR_HARDWARE(&s_sd_error_handler, ret, k_error_severity_high, 
                  "Failed to install GPIO ISR service");
    return ESP_FAIL;
  }
  
  /* Add ISR handler for CD pin */
  ret = gpio_isr_handler_add(sd_card_cd, priv_sd_card_isr_handler, NULL);
  if (ret != ESP_OK) {
    ERROR_HARDWARE(&s_sd_error_handler, ret, k_error_severity_high, 
                  "Failed to add ISR handler for CD pin");
    return ESP_FAIL;
  }
  
  /* Create mount task */
  BaseType_t task_created = xTaskCreate(priv_sd_card_mount_task,
                                        "sd_mount_task",
                                        4096,
                                        NULL,
                                        5,
                                        &s_mount_task_handle);
  
  if (task_created != pdPASS) {
    ERROR_HARDWARE(&s_sd_error_handler, ESP_FAIL, k_error_severity_high, 
                  "Failed to create SD card mount task");
    gpio_isr_handler_remove(sd_card_cd);
    return ESP_FAIL;
  }
  
  /* Check initial card state */
  bool card_present = !gpio_get_level(sd_card_cd);
  if (card_present) {
    /* Notify the task to check the card */
    xTaskNotifyGive(s_mount_task_handle);
  }
  
  return ESP_OK;
}

/* Public Functions ***********************************************************/

esp_err_t sd_card_init(void) 
{
  esp_err_t ret;
  uint8_t   retry_count = 0;

  log_info(sd_card_tag, "Init Start", "Beginning SD card initialization");

  while (retry_count < sd_card_max_retries) {
    /* Configure SPI host */
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot         = sd_card_spi_host;           /* Use the defined SPI host */
    host.max_freq_khz = sd_card_spi_freq_hz / 1000; /* Convert Hz to kHz for the frequency */

    /* Initialize SPI bus through bus manager */
    ret = bus_manager_spi_init(sd_card_data_to_card, 
                              sd_card_data_from_card, 
                              sd_card_clk, 
                              sd_card_spi_host, 
                              SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
      ERROR_HARDWARE(&s_sd_error_handler, ret, k_error_severity_medium, 
                    "Failed to initialize SPI bus");
      log_error(sd_card_tag, "SPI Error", "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
      vTaskDelay(pdMS_TO_TICKS(sd_card_retry_delay_ms));
      retry_count++;
      continue;
    }

    /* Configure SD card slot */
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs               = sd_card_cs;
    slot_config.host_id               = sd_card_spi_host;

    /* Filesystem mount configuration */
    esp_vfs_fat_mount_config_t mount_config = {
      .format_if_mount_failed = false,
      .max_files              = sd_card_max_files,
      .allocation_unit_size   = sd_card_allocation_unit_size,
    };

    /* Mount the filesystem */
    ret = esp_vfs_fat_sdspi_mount(sd_card_mount_path, &host, &slot_config, &mount_config, &s_card);
    if (ret == ESP_OK) {
      /* Log SD card information */
      sdmmc_card_print_info(stdout, s_card);
      log_info(sd_card_tag, "Init Complete", "SD card mounted at %s", sd_card_mount_path);
      
      /* Record successful status in error handler */
      error_handler_record_status(&s_sd_error_handler, ESP_OK);
      
      return ESP_OK;
    } else {
      ERROR_HARDWARE(&s_sd_error_handler, ret, k_error_severity_medium, 
                    "Failed to mount filesystem");
      log_error(sd_card_tag, "Mount Error", "Failed to mount filesystem: %s", esp_err_to_name(ret));
      priv_sd_card_cleanup();
      vTaskDelay(pdMS_TO_TICKS(sd_card_retry_delay_ms));
      retry_count++;
    }
  }

  ERROR_HARDWARE(&s_sd_error_handler, ESP_FAIL, k_error_severity_high, 
                "SD card initialization failed after maximum retries");
  log_error(sd_card_tag, "Init Error", "SD card initialization failed after %u retries", sd_card_max_retries);
  return ESP_FAIL;
}

esp_err_t sd_card_detection_init(void)
{
  if (s_sd_card_initialized) {
    return ESP_OK;
  }
  
  /* Initialize error handler */
  error_handler_init(&s_sd_error_handler, 
                    sd_card_tag,
                    sd_card_max_retries, 
                    sd_card_retry_delay_ms,
                    sd_card_retry_delay_ms * 10, 
                    priv_sd_card_reset,
                    NULL, 
                    sd_card_retry_delay_ms,
                    sd_card_retry_delay_ms * 20);
  
  /* Register component with system error handler */
  component_info_t sd_component = {
    .component_id = "sd_card",
    .handler = &s_sd_error_handler,
    .parent_id = NULL,
    .priority = 10
  };
  
  esp_err_t ret = error_handler_register_component(&sd_component);
  if (ret != ESP_OK) {
    log_error(sd_card_tag, "Error Handler", "Failed to register component with error handler");
    return ESP_FAIL;
  }
  
  /* Create mutex using common_setup */
  ret = common_setup_mutex(&s_sd_mutex, sd_card_tag);
  if (ret != ESP_OK) {
    ERROR_HARDWARE(&s_sd_error_handler, ret, k_error_severity_high, 
                  "Failed to create SD card mutex");
    error_handler_unregister_component("sd_card");
    return ESP_FAIL;
  }
  
  /* Initialize CD pin and interrupt */
  ret = priv_sd_card_init_cd_pin();
  if (ret != ESP_OK) {
    ERROR_HARDWARE(&s_sd_error_handler, ret, k_error_severity_high, 
                  "Failed to initialize card detect pin");
    common_cleanup_mutex(&s_sd_mutex, sd_card_tag);
    error_handler_unregister_component("sd_card");
    return ESP_FAIL;
  }
  
  s_sd_card_initialized = true;
  log_info(sd_card_tag, "Detection Init", "SD card detection system initialized successfully");
  
  return ESP_OK;
}

bool sd_card_is_available(void)
{
  bool available = false;
  
  if (xSemaphoreTake(s_sd_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    available = s_sd_card_available;
    xSemaphoreGive(s_sd_mutex);
  }
  
  return available;
}

esp_err_t sd_card_register_availability_callback(sd_card_availability_callback_t callback)
{
  if (callback == NULL) {
    ERROR_HARDWARE(&s_sd_error_handler, ESP_ERR_INVALID_ARG, k_error_severity_low, 
                  "NULL callback provided");
    return ESP_ERR_INVALID_ARG;
  }
  
  if (xSemaphoreTake(s_sd_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    s_availability_callback = callback;
    xSemaphoreGive(s_sd_mutex);
    
    /* Call the callback with the current state */
    callback(s_sd_card_available);
    
    return ESP_OK;
  }
  
  ERROR_HARDWARE(&s_sd_error_handler, ESP_ERR_TIMEOUT, k_error_severity_low, 
                "Failed to acquire mutex for registering callback");
  return ESP_ERR_TIMEOUT;
}

esp_err_t sd_card_cleanup(void)
{
  esp_err_t ret = ESP_OK;
  
  log_info(sd_card_tag, "Cleanup Start", "Beginning SD card system shutdown");
  
  /* Take the mutex to ensure exclusive access */
  if (s_sd_mutex != NULL) {
    if (xSemaphoreTake(s_sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
      ERROR_HARDWARE(&s_sd_error_handler, ESP_ERR_TIMEOUT, k_error_severity_low, 
                    "Could not acquire mutex for SD card cleanup");
      log_warn(sd_card_tag, "Mutex Warning", "Could not acquire mutex for SD card cleanup");
      ret = ESP_ERR_TIMEOUT;
      /* Continue with cleanup anyway */
    }
  }
  
  /* Remove the ISR handler for the CD pin */
  if (s_sd_card_initialized) {
    esp_err_t isr_ret = gpio_isr_handler_remove(sd_card_cd);
    if (isr_ret != ESP_OK) {
      ERROR_HARDWARE(&s_sd_error_handler, isr_ret, k_error_severity_low, 
                    "Failed to remove ISR handler");
      log_warn(sd_card_tag, 
               "ISR Warning", 
               "Failed to remove ISR handler: %s", 
               esp_err_to_name(isr_ret));
      ret = ESP_FAIL;
    }
  }
  
  /* Delete the mount task if it exists */
  if (s_mount_task_handle != NULL) {
    vTaskDelete(s_mount_task_handle);
    s_mount_task_handle = NULL;
    log_info(sd_card_tag, "Task Cleanup", "SD card mount task deleted");
  }
  
  /* Unmount the SD card and free the SPI bus */
  if (s_sd_card_available || s_card != NULL) {
    priv_sd_card_cleanup();
    s_sd_card_available = false;
  }
  
  /* Reset the availability callback */
  s_availability_callback = NULL;
  
  /* Release the mutex if we acquired it */
  if (s_sd_mutex != NULL) {
    if (ret == ESP_OK) {
      /* Only give the mutex if we successfully took it */
      xSemaphoreGive(s_sd_mutex);
    }
    
    /* Clean up the mutex using common_cleanup */
    common_cleanup_mutex(&s_sd_mutex, sd_card_tag);
  }
  
  /* Clean up the error handler */
  error_handler_cleanup(&s_sd_error_handler);
  error_handler_unregister_component("sd_card");
  
  s_sd_card_initialized = false;
  
  log_info(sd_card_tag, "Cleanup Complete", "SD card system shutdown %s", 
           (ret == ESP_OK) ? "successful" : "completed with warnings");
  
  return ret;
}

esp_err_t sd_card_detection_cleanup(void)
{
  if (!s_sd_card_initialized) {
    log_warn(sd_card_tag, 
             "Cleanup Skip", 
             "SD card detection system not initialized");
    return ESP_OK;
  }

  log_info(sd_card_tag, 
           "Detection Cleanup", 
           "Cleaning up SD card detection system");

  esp_err_t ret = ESP_OK;

  /* Remove the ISR handler for the CD pin */
  esp_err_t temp_ret = gpio_isr_handler_remove(sd_card_cd);
  if (temp_ret != ESP_OK) {
    ERROR_HARDWARE(&s_sd_error_handler, temp_ret, k_error_severity_low, 
                  "Failed to remove ISR handler");
    log_warn(sd_card_tag, 
             "ISR Warning", 
             "Failed to remove ISR handler: %s", 
             esp_err_to_name(temp_ret));
    ret = ESP_FAIL;
  }

  /* Delete the mount task if it exists */
  if (s_mount_task_handle != NULL) {
    vTaskDelete(s_mount_task_handle);
    s_mount_task_handle = NULL;
    log_info(sd_card_tag, "Task Cleanup", "SD card mount task deleted");
  }

  /* Clean up the mutex using common_cleanup */
  if (s_sd_mutex != NULL) {
    common_cleanup_mutex(&s_sd_mutex, sd_card_tag);
  }

  /* Clean up the error handler */
  error_handler_cleanup(&s_sd_error_handler);
  error_handler_unregister_component("sd_card");

  s_sd_card_initialized = false;
  s_sd_card_available = false;
  s_availability_callback = NULL;

  log_info(sd_card_tag, 
           "Detection Cleanup Complete", 
           "SD card detection system cleanup %s", 
           (ret == ESP_OK) ? "successful" : "completed with warnings");

  return ret;
}

