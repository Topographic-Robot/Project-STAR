/* components/pstar_error_handling/include/error_handler.h */

#ifndef PSTAR_ERROR_HANDLER_H
#define PSTAR_ERROR_HANDLER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Macros *********************************************************************/

/**
 * @brief Record an error and manage retry/reset logic
 * 
 * @param[in] handler     Pointer to error handler structure
 * @param[in] error       Error code
 * @param[in] description Error description
 * @param[in] file        Source file
 * @param[in] line        Line number
 */
#define RECORD_ERROR(handler, code, desc)                                          \
    do {                                                                           \
        error_handler_record_error((handler), (code), (desc), __FILE__, __LINE__); \
    } while (0)

/* Structs ********************************************************************/

/**
 * @brief Error handler structure for managing error states and recovery
 * 
 * This structure maintains error state information and provides retry
 * mechanisms for components that encounter errors. It tracks error counts,
 * manages exponential backoff for retries, and can execute custom reset
 * functions when needed.
 */
typedef struct error_handler {
  uint32_t          error_count;              /**< Total number of errors recorded */
  uint32_t          max_retries;              /**< Maximum number of retry attempts allowed */
  uint32_t          current_retry;            /**< Current retry attempt counter */
  uint32_t          base_retry_delay;         /**< Base delay between retries (ms) */
  uint32_t          max_retry_delay;          /**< Maximum retry delay (ms) */
  uint32_t          current_retry_delay;      /**< Current retry delay with backoff applied */
  esp_err_t         last_error;               /**< Last recorded error code */
  bool              in_error_state;           /**< Whether component is in error state */
  esp_err_t       (*reset_fn)(void* context); /**< Optional function to reset component */
  void*             reset_context;            /**< Context passed to reset function */
  SemaphoreHandle_t mutex;                    /**< Mutex for thread-safe operations */
} error_handler_t;

/* Functions ******************************************************************/

/**
 * @brief Initialize an error handler
 * 
 * @param[in] handler          Pointer to error handler structure
 * @param[in] max_retries      Maximum number of retry attempts
 * @param[in] base_retry_delay Initial delay between retries (ms)
 * @param[in] max_retry_delay  Maximum retry delay (ms)
 * @param[in] reset_fn         Optional reset function
 * @param[in] reset_context    Context for reset function
 */
void error_handler_init(error_handler_t* handler, 
                        uint32_t         max_retries,
                        uint32_t         base_retry_delay,
                        uint32_t         max_retry_delay,
                        esp_err_t      (*reset_fn)(void* context),
                        void*            reset_context);

/**
 * @brief Record an error and manage retry/reset logic
 * 
 * @param[in] handler     Pointer to error handler structure
 * @param[in] error       Error code
 * @param[in] description Error description
 * @param[in] file        Source file
 * @param[in] line        Line number
 * @return ESP_OK if handled, error code otherwise
 */
esp_err_t error_handler_record_error(error_handler_t* handler, 
                                     esp_err_t        error, 
                                     const char*      description,
                                     const char*      file, 
                                     int              line); /* Its an int since __LINE__ expands to an int */

/**
 * @brief Check if a retry should be attempted
 * 
 * @param[in] handler Pointer to error handler structure
 * @return true if retry is possible, false otherwise
 */
bool error_handler_can_retry(error_handler_t* handler);

/**
 * @brief Reset the error handler state
 * 
 * @param[in] handler Pointer to error handler structure
 */
void error_handler_reset_state(error_handler_t* handler);

#endif /* PSTAR_ERROR_HANDLER_H */
