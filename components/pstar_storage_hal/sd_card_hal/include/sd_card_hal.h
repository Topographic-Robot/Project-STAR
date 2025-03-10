/* components/pstar_storage_hal/sd_card_hal/include/sd_card_hal.h */

#ifndef PSTAR_SD_CARD_HAL_H
#define PSTAR_SD_CARD_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "error_handler.h"
#include "sdmmc_cmd.h"
#include "bus_manager.h"

/* Enums **********************************************************************/

/**
 * @brief SD card interface types in order of preference
 * 
 * The enum values represent the clock speeds in Hz
 */
typedef enum : uint32_t {
  k_sd_interface_sdio       = 40000000, /* 40 MHz SDIO (highest performance) */
  k_sd_interface_fast_spi   = 20000000, /* 20 MHz SPI */
  k_sd_interface_normal_spi = 4000000,  /* 4 MHz SPI */
  k_sd_interface_slow_spi   = 400000,   /* 400 kHz SPI (lowest performance but most compatible) */
  k_sd_interface_count      = 4,        /* Total number of interfaces */
} sd_interface_type_t;

/**
 * @brief SD card bus width options
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
  bool    measurement_needed;  /**< Flag indicating if measurement is needed */
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
  const char*           tag;                                  /**< Logging tag for identification in logs */
  const char*           mount_path;                           /**< Filesystem mount path (e.g., "/sdcard") */
  const uint8_t         max_files;                            /**< Maximum number of simultaneously open files */
  const uint32_t        allocation_unit_size;                 /**< Filesystem allocation unit size in bytes */
  
  /* Configuration */
  bool                  card_detect_low_active;               /**< If true, card detect pin is low-active (0=card present), if false, high-active (1=card present) */
  sd_bus_width_t        bus_width;                            /**< SD card bus width (1-bit or 4-bit mode) */
  sd_card_pin_config_t  pin_config;                           /**< Pin configuration for all SD card interfaces */
  sd_card_task_config_t task_config;                          /**< Task configuration parameters */
  
  /* State machine */
  sd_state_t            state;                                /**< Current state of the SD card state machine */
  uint32_t              error_count;                          /**< Number of errors encountered */
  int64_t               last_state_change_time;               /**< Timestamp of last state change */
  
  /* Interface tracking */
  sd_interface_type_t   current_interface;                    /**< Currently active interface type */
  sd_interface_info_t   interface_info[k_sd_interface_count]; /**< Information about each interface */
  bool                  interface_discovery_complete;         /**< Whether interface discovery is complete for current card */
  uint32_t              interface_attempt_count;              /**< Number of interface discovery attempts */
  
  /* State variables */
  sdmmc_card_t*         card;                                 /**< SD card descriptor for ESP-IDF SD card API */
  SemaphoreHandle_t     mutex;                                /**< Mutex for thread-safe access to card state */
  bool                  card_available;                       /**< Indicates if a card is currently inserted and mounted */
  bool                  initialized;                          /**< Indicates if the HAL has been initialized */
  volatile bool         mount_task_exit_requested;            /**< Flag to request exit from mount task */
  TaskHandle_t          mount_task_handle;                    /**< Handle for the automatic mount/unmount task */
  error_handler_t       error_handler;                        /**< Error handler for fault detection and recovery */
  const char*           component_id;                         /**< Component ID for error handler registration */
  pstar_bus_manager_t   bus_manager;                          /**< Bus manager for SPI and GPIO */
  
  /* Performance monitoring */
  sd_card_performance_t performance;                          /**< Performance metrics for the SD card */
} sd_card_hal_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initializes an SD card HAL structure with default values
 * 
 * This function sets up the SD card HAL structure with default values optimized
 * for typical use cases. It configures the SPI interface, GPIO pins, and system
 * parameters needed for SD card operation. The caller must allocate memory for
 * the structure before calling this function.
 * 
 * @note This function only initializes the structure; it does not initialize the
 *       hardware or mount the SD card. Call sd_card_hal_init() after this function.
 * 
 * Default configuration:
 * - SPI2_HOST for SPI host
 * - GPIO 5 for CS (Chip Select)
 * - GPIO 23 for DO (Data Out)
 * - GPIO 19 for DI (Data In)
 * - GPIO 14 for CLK (Clock)
 * - GPIO 13 for CD (Card Detect)
 * - 1 MHz SPI frequency
 * - 5 maximum open files
 * - 16KB allocation unit size
 * 
 * @param[out] sd_card      Pointer to the SD card HAL structure to initialize
 * @param[in]  tag          Tag for logging and identification (should be a static string)
 * @param[in]  mount_path   Mount path for the SD card (e.g., "/sdcard")
 * @param[in]  component_id Component ID for error handler registration
 * @param[in]  bus_width    SD card bus width (1-bit or 4-bit mode)
 * @return ESP_OK if successful, ESP_ERR_INVALID_ARG if any parameter is NULL
 */
esp_err_t sd_card_init_default(sd_card_hal_t* sd_card, 
                               const char*    tag, 
                               const char*    mount_path, 
                               const char*    component_id,
                               sd_bus_width_t bus_width);

/**
 * @brief Initializes an SD card HAL structure with custom pin assignments
 * 
 * This function extends the default initialization by allowing custom pin
 * assignments for all SD card interfaces. This is useful when the default
 * GPIO pins conflict with other peripherals or when using different hardware.
 * 
 * @param[out] sd_card      Pointer to the SD card HAL structure to initialize
 * @param[in]  tag          Tag for logging and identification (should be a static string)
 * @param[in]  mount_path   Mount path for the SD card (e.g., "/sdcard")
 * @param[in]  component_id Component ID for error handler registration
 * @param[in]  bus_width    SD card bus width (1-bit or 4-bit mode)
 * @param[in]  pin_config   Custom pin configuration for all SD card interfaces
 * @return ESP_OK if successful, ESP_ERR_INVALID_ARG if any parameter is NULL
 */
esp_err_t sd_card_init_with_pins(sd_card_hal_t*                  sd_card, 
                                 const char*                     tag, 
                                 const char*                     mount_path, 
                                 const char*                     component_id,
                                 sd_bus_width_t                  bus_width,
                                 const sd_card_pin_config_t*     pin_config);

/**
 * @brief Configures task and synchronization parameters for the SD card HAL
 * 
 * This function allows customization of task stack size, priority and mutex timeout.
 * Must be called after sd_card_init_default() and before sd_card_init().
 * 
 * @param[in,out] sd_card       Pointer to the SD card HAL instance
 * @param[in]     stack_size    Stack size for the mount task (in bytes)
 * @param[in]     priority      Priority for the mount task
 * @param[in]     mutex_timeout Timeout for acquiring mutex (in ticks)
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t sd_card_set_task_config(sd_card_hal_t* sd_card, 
                                  uint32_t       stack_size,
                                  UBaseType_t    priority,
                                  TickType_t     mutex_timeout);

/**
 * @brief Initializes the SD card hardware and detection system
 * 
 * This function performs a complete initialization of the SD card system, including:
 * 1. Setting up the error handler with appropriate retry parameters
 * 2. Initializing the card detection system with interrupt handling
 * 3. Automatically mounting the SD card if present
 * 
 * The function sets up a background task that monitors card insertion/removal
 * events and automatically mounts/unmounts the card as needed. This provides
 * a robust hot-plug capability for the SD card.
 * 
 * @note This function should be called after sd_card_init_default().
 * 
 * @param[in,out] sd_card   Pointer to the initialized SD card HAL instance
 * @param[in]     parent_id Parent component ID for error propagation hierarchy (can be NULL)
 * @param[in]     priority  Priority for error handling (lower values = higher priority)
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t sd_card_init(sd_card_hal_t* sd_card, 
                       const char*    parent_id, 
                       uint32_t       priority);

/**
 * @brief Checks if the SD card is currently available for use
 *
 * This function provides a thread-safe way to check if an SD card is currently
 * inserted and mounted. It should be called before attempting any file operations
 * to avoid errors due to missing media.
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return true if the SD card is inserted and mounted, false otherwise
 */
bool sd_card_is_available(sd_card_hal_t* sd_card);

/**
 * @brief Gets the current state of the SD card state machine
 * 
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return The current state, or k_sd_state_idle if not initialized
 */
sd_state_t sd_card_get_state(sd_card_hal_t* sd_card);

/**
 * @brief Gets the current interface type being used
 * 
 * This function returns the type of interface currently being used to
 * communicate with the SD card.
 * 
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return The current interface type, or k_sd_interface_count if not initialized
 */
sd_interface_type_t sd_card_get_current_interface(sd_card_hal_t* sd_card);

/**
 * @brief Gets the current bus width of the SD card interface
 * 
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return The current bus width, or k_sd_bus_width_1bit if not initialized
 */
sd_bus_width_t sd_card_get_bus_width(sd_card_hal_t* sd_card);

/**
 * @brief Gets a string representation of the interface type
 * 
 * @param[in] interface_type The interface type to convert to string
 * @return String representation of the interface type
 */
const char* sd_card_interface_to_string(sd_interface_type_t interface_type);

/**
 * @brief Gets performance metrics for the SD card
 * 
 * @param[in]  sd_card     Pointer to the SD card HAL instance
 * @param[out] performance Pointer to structure to receive performance metrics
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t sd_card_get_performance(sd_card_hal_t*         sd_card, 
                                  sd_card_performance_t* performance);

/**
 * @brief Forces a remount of the SD card
 * 
 * This function unmounts and remounts the SD card, which can be useful to
 * refresh the connection or reset the SD card after an error.
 * 
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t sd_card_force_remount(sd_card_hal_t* sd_card);

/**
 * @brief Sets the SD card bus width
 * 
 * This function changes the bus width used for SDIO mode. If the card is
 * currently mounted, it will be remounted to apply the new bus width.
 * 
 * @param[in] sd_card   Pointer to the SD card HAL instance
 * @param[in] bus_width New bus width to use
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t sd_card_set_bus_width(sd_card_hal_t* sd_card, sd_bus_width_t bus_width);

/**
 * @brief Cleans up the SD card system and releases all resources
 *
 * This function performs a complete cleanup of the SD card system, including:
 * 1. Unmounting the SD card if currently mounted
 * 2. Stopping the card detection system and associated task
 * 3. Releasing all hardware resources (SPI bus, GPIO pins)
 * 4. Cleaning up the error handler and unregistering from the system
 * 
 * This function should be called during system shutdown or when the SD card
 * functionality is no longer needed to ensure proper resource cleanup.
 * 
 * @param[in,out] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK on success, ESP_FAIL or other error code on failure
 */
esp_err_t sd_card_cleanup(sd_card_hal_t* sd_card);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_SD_CARD_HAL_H */