/* components/pstar_storage_hal/sd_card_hal/include/pstar_sd_card_types.h */

#ifndef PSTAR_SD_CARD_TYPES_H
#define PSTAR_SD_CARD_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_bus_manager_types.h"
#include "pstar_error_handler.h"

#include "driver/gpio.h" /* Add this include for gpio_num_t */
#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "sdmmc_cmd.h"

/* Enums **********************************************************************/

/**
 * @brief SD card interface types
 *
 * Defines the possible communication interfaces for the SD card.
 */
typedef enum : uint8_t {
  k_sd_interface_none = 0, /* No interface selected or initialized */
  k_sd_interface_spi,      /* SPI interface */
  k_sd_interface_sdio,     /* SDIO (SDMMC) interface */
  k_sd_interface_count,    /* Total number of interface types */
} sd_interface_type_t;

/**
 * @brief SD card bus width options (Primarily for SDIO)
 */
typedef enum : uint8_t {
  k_sd_bus_width_1bit = 1, /* 1-bit bus width for maximum compatibility */
  k_sd_bus_width_4bit = 4, /* 4-bit bus width for higher performance */
} sd_bus_width_t;

/**
 * @brief SD card state machine states
 */
typedef enum {
  k_sd_state_idle,                /* No SD card inserted or detection task not running */
  k_sd_state_card_inserted,       /* Card inserted, interface discovery needed */
  k_sd_state_interface_discovery, /* Trying to find a working interface */
  k_sd_state_interface_ready,     /* Working interface found, card mounted */
  k_sd_state_error,               /* Error state, retries possible */
  k_sd_state_failed,              /* Maximum retries reached, permanent failure */
} sd_state_t;

/* Typedefs *******************************************************************/

/**
 * @brief Callback function type for SD card availability notifications.
 *
 * @param available True if the SD card is now available (inserted and mounted),
 *                  false otherwise.
 */
typedef void (*sd_availability_cb_t)(bool available);

/* Structs ********************************************************************/

/**
 * @brief Task configuration for SD card HAL
 */
typedef struct sd_card_task_config {
  uint32_t    stack_size;    /**< Stack size for the mount task */
  UBaseType_t priority;      /**< Priority for the mount task */
  TickType_t  mutex_timeout; /**< Timeout for acquiring mutex */
} sd_card_task_config_t;

/**
 * @brief SD card pin configuration structure
 */
typedef struct sd_card_pin_config {
  /* GPIO pins */
  gpio_num_t gpio_det_pin; /* Card detect pin */

  /* SPI pins */
  gpio_num_t spi_di_pin;   /* SPI DI (Data In) */
  gpio_num_t spi_do_pin;   /* SPI DO (Data Out) */
  gpio_num_t spi_sclk_pin; /* SPI SCLK (Clock) */
  gpio_num_t spi_cs_pin;   /* SPI CS (Chip Select) */

  /* SDIO pins */
  gpio_num_t sdio_clk_pin; /* SDIO CLK */
  gpio_num_t sdio_cmd_pin; /* SDIO CMD */
  gpio_num_t sdio_d0_pin;  /* SDIO D0 (mandatory for both 1-bit and 4-bit modes) */
  gpio_num_t sdio_d1_pin;  /* SDIO D1 (only for 4-bit mode) */
  gpio_num_t sdio_d2_pin;  /* SDIO D2 (only for 4-bit mode) */
  gpio_num_t sdio_d3_pin;  /* SDIO D3 (only for 4-bit mode) */
} sd_card_pin_config_t;

/**
 * @brief Information about an SD card interface configuration
 */
typedef struct sd_interface_info {
  bool     attempted;         /**< Whether this interface has been attempted */
  bool     successful;        /**< Whether this interface has worked successfully */
  uint32_t error_count;       /**< Number of errors encountered with this interface */
  int64_t  last_success_time; /**< Last time this interface was successfully used */
} sd_interface_info_t;

/**
 * @brief Performance metrics for SD card
 */
typedef struct sd_card_performance {
  int64_t last_measured;      /**< Timestamp of last measurement in microseconds */
  float   read_speed_kbps;    /**< Read speed in kilobits per second */
  float   write_speed_kbps;   /**< Write speed in kilobits per second */
  bool    measurement_needed; /**< Flag indicating if measurement is needed */
} sd_card_performance_t;

/**
 * @brief Structure containing all SD card HAL configuration and state
 *
 * This structure encapsulates all configuration parameters and runtime state
 * for the SD card hardware abstraction layer. It provides a unified interface
 * for SD card operations with automatic card detection, error handling, and
 * thread-safe access.
 */
typedef struct sd_card_hal {
  /* Constants */
  const char*    tag;                  /**< Logging tag for identification in logs */
  const char*    mount_path;           /**< Filesystem mount path (e.g., "/sdcard") */
  const uint8_t  max_files;            /**< Maximum number of simultaneously open files */
  const uint32_t allocation_unit_size; /**< Filesystem allocation unit size in bytes */

  /* Configuration */
  bool
    card_detect_low_active; /**< If true, card detect pin is low-active (0=card present), if false, high-active (1=card present) */
  sd_bus_width_t bus_width; /**< SD card bus width (1-bit or 4-bit mode) - Primarily for SDIO */
  sd_card_pin_config_t  pin_config;          /**< Pin configuration for all SD card interfaces */
  sd_interface_type_t   preferred_interface; /**< Preferred interface to try first */
  bool                  enable_fallback;     /**< Whether to try other interfaces on failure */
  bool                  sdio_mode_enabled;   /**< Cache if SDIO mode is enabled */
  bool                  spi_mode_enabled;    /**< Cache if SPI mode is enabled */
  sd_card_task_config_t task_config;         /**< Task configuration parameters */

  /* State machine */
  sd_state_t state;                  /**< Current state of the SD card state machine */
  uint32_t   error_count;            /**< Number of errors encountered */
  int64_t    last_state_change_time; /**< Timestamp of last state change */

  /* Interface tracking */
  sd_interface_type_t current_interface;                    /**< Currently active interface type */
  sd_interface_info_t interface_info[k_sd_interface_count]; /**< Information about each interface */
  bool
    interface_discovery_complete;   /**< Whether interface discovery is complete for current card */
  uint32_t interface_attempt_count; /**< Number of interface discovery attempts */

  /* State variables */
  sdmmc_card_t*       card;           /**< SD card descriptor for ESP-IDF SD card API */
  SemaphoreHandle_t   mutex;          /**< Mutex for thread-safe access to card state */
  _Atomic bool        card_available; /**< Atomic: Indicates if card is mounted and ready */
  bool                initialized;    /**< Indicates if the HAL has been initialized */
  volatile bool       mount_task_exit_requested; /**< Flag to request exit from mount task */
  TaskHandle_t        mount_task_handle;         /**< Handle for the automatic mount/unmount task */
  error_handler_t     error_handler; /**< Error handler for fault detection and recovery */
  const char*         component_id;  /**< Component ID for error handler registration */
  pstar_bus_manager_t bus_manager;   /**< Bus manager for SPI and GPIO */

  /* Performance monitoring */
  sd_card_performance_t performance; /**< Performance metrics for the SD card */

  sd_availability_cb_t availability_callback; /**< Callback function for availability changes */

} sd_card_hal_t;

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_SD_CARD_TYPES_H */
