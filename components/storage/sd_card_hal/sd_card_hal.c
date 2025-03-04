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

/* Constants ******************************************************************/

const char             *sd_card_tag                  = "SD_CARD";
const char             *sd_card_mount_path           = "/sdcard";
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

static sdmmc_card_t     *s_card                         = NULL;  /**< Pointer to hold SD card descriptor */
static error_handler_t   s_sd_card_error_handler        = { 0 };
static SemaphoreHandle_t s_sd_mutex                     = NULL;  /**< Mutex for thread-safe access */
static bool              s_sd_card_available            = false; /**< Flag to track SD card availability */
static bool              s_sd_card_initialized          = false; /**< Flag to track initialization status */
static void            (*s_availability_callback)(bool) = NULL;  /**< Callback for SD card availability changes */
static TaskHandle_t      s_mount_task_handle            = NULL;  /**< Handle for the mount/unmount task */

/* Private (Static) Functions *************************************************/

/**
 * @brief Internal function to unmount and cleanup resources.
 */
static void priv_sd_card_cleanup(void)
{
  if (s_card) {
    esp_vfs_fat_sdcard_unmount(sd_card_mount_path, s_card);
    s_card = NULL;
    log_info(sd_card_tag, "Cleanup", "SD card unmounted successfully");
  }
  spi_bus_free(sd_card_spi_host);
}

/**
 * @brief Task to handle SD card mounting/unmounting
 * 
 * This task is triggered by the CD pin interrupt and handles the actual
 * mounting or unmounting of the SD card.
 */
static void priv_sd_card_mount_task(void *arg)
{
  while (1) {
    /* Wait for notification from ISR */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    /* Debounce */
    vTaskDelay(pdMS_TO_TICKS(sd_card_debounce_ms));
    
    /* Get current card state (CD pin is active LOW - LOW when card is inserted) */
    bool card_present = !gpio_get_level(sd_card_cd);
    
    if (xSemaphoreTake(s_sd_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      bool state_changed = (card_present != s_sd_card_available);
      
      if (state_changed) {
        if (card_present) {
          /* Card inserted - try to mount */
          log_info(sd_card_tag, "Card Detected", "SD card inserted, attempting to mount");
          
          /* Initialize and mount the card */
          esp_err_t ret = sd_card_init();
          if (ret == ESP_OK) {
            s_sd_card_available = true;
            log_info(sd_card_tag, "Mount Success", "SD card mounted successfully");
          } else {
            s_sd_card_available = false;
            log_error(sd_card_tag, "Mount Failed", "Failed to mount SD card: %s", esp_err_to_name(ret));
          }
        } else {
          /* Card removed - unmount */
          log_info(sd_card_tag, "Card Removed", "SD card removed, unmounting");
          priv_sd_card_cleanup();
          s_sd_card_available = false;
        }
        
        /* Call the availability callback if registered */
        if (s_availability_callback != NULL) {
          s_availability_callback(s_sd_card_available);
        }
      }
      
      xSemaphoreGive(s_sd_mutex);
    }
  }
}

/**
 * @brief ISR for the CD pin
 * 
 * This function is called when the CD pin changes state.
 */
static void IRAM_ATTR priv_sd_card_isr_handler(void* arg)
{
  /* Notify the mount task */
  if (s_mount_task_handle != NULL) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(s_mount_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }
}

/**
 * @brief Initialize the CD pin and interrupt
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t priv_sd_card_init_cd_pin(void)
{
  /* Configure CD pin */
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << sd_card_cd),
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_ANYEDGE,
  };
  
  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    log_error(sd_card_tag, "GPIO Error", "Failed to configure CD pin: %s", esp_err_to_name(ret));
    return ESP_FAIL;
  }
  
  /* Install GPIO ISR service */
  ret = gpio_install_isr_service(0);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    /* ESP_ERR_INVALID_STATE means the service is already installed, which is fine */
    log_error(sd_card_tag, "ISR Error", "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
    return ESP_FAIL;
  }
  
  /* Add ISR handler for CD pin */
  ret = gpio_isr_handler_add(sd_card_cd, priv_sd_card_isr_handler, NULL);
  if (ret != ESP_OK) {
    log_error(sd_card_tag, "ISR Error", "Failed to add ISR handler for CD pin: %s", esp_err_to_name(ret));
    return ESP_FAIL;
  }
  
  /* Create mount task */
  BaseType_t task_created = xTaskCreate(
    priv_sd_card_mount_task,
    "sd_mount_task",
    4096,
    NULL,
    5,
    &s_mount_task_handle
  );
  
  if (task_created != pdPASS) {
    log_error(sd_card_tag, "Task Error", "Failed to create SD card mount task");
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

    /* Configure SPI bus */
    spi_bus_config_t bus_cfg = {
      .mosi_io_num     = sd_card_data_to_card,
      .miso_io_num     = sd_card_data_from_card,
      .sclk_io_num     = sd_card_clk,
      .quadwp_io_num   = -1, /* Not used */
      .quadhd_io_num   = -1, /* Not used */
      .max_transfer_sz = sd_card_max_transfer_sz
    };

    ret = spi_bus_initialize(sd_card_spi_host, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
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
      return ESP_OK;
    } else {
      log_error(sd_card_tag, "Mount Error", "Failed to mount filesystem: %s", esp_err_to_name(ret));
      priv_sd_card_cleanup();
      vTaskDelay(pdMS_TO_TICKS(sd_card_retry_delay_ms));
      retry_count++;
    }
  }

  log_error(sd_card_tag, "Init Error", "SD card initialization failed after %u retries", sd_card_max_retries);
  return ESP_FAIL;
}

/**
 * @brief Initialize the SD card detection system
 * 
 * This function initializes the CD pin and interrupt, and creates the mutex.
 * 
 * @return ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t sd_card_detection_init(void)
{
  if (s_sd_card_initialized) {
    return ESP_OK;
  }
  
  /* Create mutex */
  s_sd_mutex = xSemaphoreCreateMutex();
  if (s_sd_mutex == NULL) {
    log_error(sd_card_tag, "Mutex Error", "Failed to create SD card mutex");
    return ESP_FAIL;
  }
  
  /* Initialize CD pin and interrupt */
  esp_err_t ret = priv_sd_card_init_cd_pin();
  if (ret != ESP_OK) {
    vSemaphoreDelete(s_sd_mutex);
    s_sd_mutex = NULL;
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

esp_err_t sd_card_register_availability_callback(void (*callback)(bool available))
{
  if (callback == NULL) {
    return ESP_FAIL;
  }
  
  if (xSemaphoreTake(s_sd_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    s_availability_callback = callback;
    xSemaphoreGive(s_sd_mutex);
    
    /* Call the callback with the current state */
    callback(s_sd_card_available);
    
    return ESP_OK;
  }
  
  return ESP_FAIL;
}

