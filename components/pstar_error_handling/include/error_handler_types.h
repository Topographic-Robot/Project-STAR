/* components/pstar_error_handling/include/error_handler_types.h */

#ifndef PSTAR_ERROR_HANDLER_TYPES_H
#define PSTAR_ERROR_HANDLER_TYPES_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

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

#endif /* PSTAR_ERROR_HANDLER_TYPES_H */
