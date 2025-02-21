/* components/common/include/error_handler.h */

#ifndef TOPOROBO_ERROR_HANDLER_H
#define TOPOROBO_ERROR_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/**
 * @brief Data structure for managing component state and recovery.
 *
 * Contains essential data for implementing exponential backoff and retry logic
 * when errors occur, as well as tracking successful states in the system.
 * Each component can configure its own retry policy and reset behavior.
 */
typedef struct {
  uint8_t     retry_count;                /**< Counter for consecutive retry attempts */
  uint32_t    retry_interval;             /**< Current interval between retry attempts */
  uint32_t    initial_interval;           /**< Initial retry interval in ticks */
  uint32_t    initial_backoff_interval;   /**< Initial backoff interval in ticks */
  uint32_t    max_interval;               /**< Maximum backoff interval in ticks */
  uint32_t    max_backoff_interval;       /**< Maximum backoff interval in ticks */
  uint8_t     max_retries;                /**< Maximum number of retry attempts before giving up */
  TickType_t  last_attempt_ticks;         /**< Tick count of the last retry attempt */
  esp_err_t   last_status;                /**< Last status code encountered */
  bool        in_error_state;             /**< Whether the component is in an error state */
  const char *tag;                        /**< Tag for ESP_LOG messages */
  void       *context;                    /**< Context pointer for the reset function */
  esp_err_t (*reset_func)(void *context); /**< Function to call during reset, returns ESP_OK on success */
} error_handler_t;

/**
 * @brief Initializes the state handler structure.
 *
 * Sets up initial values for retry management and state tracking. The reset_func
 * is initialized to NULL and can be set after initialization if needed.
 *
 * @param[out] handler Pointer to the error_handler_t structure to initialize
 * @param[in] tag Tag to use for logging messages (usually component name)
 * @param[in] max_retries Maximum number of retry attempts
 * @param[in] initial_interval Initial retry interval in ticks
 * @param[in] max_interval Maximum backoff interval in ticks
 * @param[in] reset_func Pointer to the reset function to call when an error occurs
 * @param[in] context Context pointer to pass to reset_func
 * @param[in] initial_backoff_interval Initial backoff interval in ticks
 * @param[in] max_backoff_interval Maximum backoff interval in ticks
 */
void error_handler_init(error_handler_t *handler, 
                        const char *tag,
                        uint8_t     max_retries, 
                        uint32_t    initial_interval,
                        uint32_t    max_interval, 
                        esp_err_t (*reset_func)(void *context),
                        void       *context, 
                        uint32_t    initial_backoff_interval,
                        uint32_t    max_backoff_interval);

/**
 * @brief Records a state change in the handler and triggers reset if needed.
 *
 * Updates the state and triggers the reset function if in error state and enough 
 * time has passed since the last attempt. Implements exponential backoff for 
 * repeated errors. For success states, resets error counters and intervals.
 *
 * @param[in,out] handler Pointer to the error_handler_t structure
 * @param[in] status The status code to record (ESP_OK for success)
 * @return
 * - ESP_OK if status is success or reset was successful
 * - ESP_ERR_INVALID_STATE if in backoff period
 * - Other error codes from the reset function
 */
esp_err_t error_handler_record_status(error_handler_t *handler, esp_err_t status);

/**
 * @brief Resets the handler to its initial state.
 *
 * Resets all tracking fields to their initial values and calls the
 * reset_func if one is set. This is useful for recovering from error states
 * or reinitializing a component.
 *
 * @param[in,out] handler Pointer to the error_handler_t structure to reset
 * @return
 * - ESP_OK if reset was successful or no reset needed
 * - Other error codes from the reset function
 */
esp_err_t error_handler_reset(error_handler_t *handler);

/* For backward compatibility */
#define error_handler_record_error(handler, error) error_handler_record_status(handler, error)

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_ERROR_HANDLER_H */