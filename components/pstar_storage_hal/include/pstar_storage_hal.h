/* components/pstar_storage_hal/include/pstar_storage_hal.h */

#ifndef PSTAR_STORAGE_HAL_H
#define PSTAR_STORAGE_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_storage_types.h"
#include "esp_err.h"

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
 *       hardware or mount the SD card. Call sd_card_init() after this function.
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
esp_err_t sd_card_init_with_pins(sd_card_hal_t*              sd_card,
                                 const char*                 tag,
                                 const char*                 mount_path,
                                 const char*                 component_id,
                                 sd_bus_width_t              bus_width,
                                 const sd_card_pin_config_t* pin_config);

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
 * @param[in,out] sd_card Pointer to the initialized SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t sd_card_init(sd_card_hal_t* sd_card);

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
 * @return The current interface type, or k_sd_interface_none if not initialized
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
esp_err_t sd_card_get_performance(sd_card_hal_t* sd_card, sd_card_performance_t* performance);

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

/**
 * @brief Registers a callback function to be notified of SD card availability changes.
 *
 * @param[in] sd_card  Pointer to the SD card HAL instance.
 * @param[in] callback Function pointer to the callback function (takes a bool argument: true if available, false otherwise).
 * @return
 *  - ESP_OK on success.
 *  - ESP_ERR_INVALID_ARG if sd_card is NULL.
 *  - ESP_ERR_INVALID_STATE if the HAL is not initialized.
 */
esp_err_t sd_card_register_availability_callback(sd_card_hal_t*       sd_card,
                                                 sd_availability_cb_t callback);

/* Additional Functions *******************************************************/

/**
 * @brief Mounts the SD card
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t sd_card_mount(sd_card_hal_t* sd_card);

/**
 * @brief Unmounts the SD card
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t sd_card_unmount(sd_card_hal_t* sd_card);

/**
 * @brief Sets up card detection using GPIO
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t sd_card_setup_detection(sd_card_hal_t* sd_card);

/**
 * @brief Try mounting the SD card with available interfaces according to config
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if any interface worked, appropriate error code otherwise
 */
esp_err_t sd_card_try_interfaces(sd_card_hal_t* sd_card);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_STORAGE_HAL_H */
