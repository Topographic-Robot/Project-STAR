/* components/pstar_storage_hal/include/pstar_storage_common.h */

#ifndef PSTAR_STORAGE_COMMON_H
#define PSTAR_STORAGE_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pstar_storage_types.h"

#include "esp_err.h"

/**
 * @brief Safely releases a mutex if it was taken
 *
 * @param[in]     sd_card     Pointer to the SD card HAL instance
 * @param[in,out] mutex_taken Pointer to the mutex taken flag to update
 */
void storage_release_mutex_if_taken(sd_card_hal_t* sd_card, bool* mutex_taken);

/**
 * @brief Updates the state machine state with proper logging
 *
 * @param[in] sd_card   Pointer to the SD card HAL instance
 * @param[in] new_state New state to transition to
 */
void storage_update_state_machine(sd_card_hal_t* sd_card, sd_state_t new_state);

/**
 * @brief Saves the current working SD card configuration to NVS
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t storage_save_working_config(sd_card_hal_t* sd_card);

/**
 * @brief Loads the previously working SD card configuration from NVS
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t storage_load_working_config(sd_card_hal_t* sd_card);

/**
 * @brief Checks if an SD card is physically inserted
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @return true if card is inserted, false otherwise
 */
bool storage_sd_card_is_inserted(sd_card_hal_t* sd_card);

/**
 * @brief Notifies registered callback about SD card availability
 *
 * @param[in] sd_card    Pointer to the SD card HAL instance
 * @param[in] available  Whether the card is available or not
 */
void storage_notify_availability(sd_card_hal_t* sd_card, bool available);

/**
 * @brief Get string representation of bus width
 *
 * @param bus_width The bus width value to convert to string
 * @return const char* String representation
 */
const char* storage_bus_width_to_string(sd_bus_width_t bus_width);

/**
 * @brief Creates a directory if it doesn't exist
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @param[in] path Directory path to create
 * @return ESP_OK if successful, appropriate error code otherwise
 */
esp_err_t storage_create_directory_if_needed(const sd_card_hal_t* sd_card, const char* path);

/**
 * @brief Checks if a path is safe to use (no directory traversal attacks)
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @param[in] path Path to check
 * @return true if path is safe, false otherwise
 */
bool storage_path_is_safe(const sd_card_hal_t* sd_card, const char* path);

/**
 * @brief Simplified path safety check without requiring an SD card instance
 *
 * @param[in] path Path to check
 * @return true if path is safe, false otherwise
 */
bool storage_path_is_safe_simple(const char* path);

/**
 * @brief Validates if a path is within the SD card's mount path
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 * @param[in] path Path to validate
 * @return true if within mount path, false otherwise
 */
bool storage_is_path_within_mount(const sd_card_hal_t* sd_card, const char* path);

/**
 * @brief Measures the performance of the SD card
 *
 * @param[in] sd_card Pointer to the SD card HAL instance
 */
void storage_measure_card_performance(sd_card_hal_t* sd_card);

/**
 * @brief Resets the SD card hardware and remounts if needed
 *
 * @param context Pointer to the SD card HAL instance
 * @return esp_err_t ESP_OK if successful, error code otherwise
 */
esp_err_t storage_sd_card_reset(void* context);

/**
 * @brief Register all SD card pins with the pin validator
 *
 * @param sd_card Pointer to the SD card HAL instance
 * @return esp_err_t ESP_OK if successful, error code otherwise
 */
esp_err_t storage_register_sd_card_pins(sd_card_hal_t* sd_card);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_STORAGE_COMMON_H */
