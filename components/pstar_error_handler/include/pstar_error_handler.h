/* components/pstar_error_handler/include/pstar_error_handler.h */

#ifndef PSTAR_COMPONENT_ERROR_HANDLER_H
#define PSTAR_COMPONENT_ERROR_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- Macros --- */

/**
 * @brief Record an error and manage retry/reset logic
 */
#define RECORD_ERROR(handler, code, desc)                                                          \
  do {                                                                                             \
    error_handler_record_error((handler), (code), (desc), __FILE__, __LINE__, __func__);           \
  } while (0)

/* --- Structs --- */

/**
 * @brief Error handler structure for managing error states and recovery
 *
 * This structure maintains error state information and provides retry
 * mechanisms for components that encounter errors. It tracks error counts,
 * manages exponential backoff for retries, and can execute custom reset
 * functions when needed.
 */
typedef struct error_handler {
  uint32_t          error_count;         /**< Total number of errors recorded */
  uint32_t          max_retries;         /**< Maximum number of retry attempts allowed */
  uint32_t          current_retry;       /**< Current retry attempt counter */
  uint32_t          base_retry_delay;    /**< Base delay between retries (ms) */
  uint32_t          max_retry_delay;     /**< Maximum retry delay (ms) */
  uint32_t          current_retry_delay; /**< Current retry delay with backoff applied */
  esp_err_t         last_error;          /**< Last recorded error code */
  bool              in_error_state;      /**< Whether component is in error state */
  void*             reset_context;       /**< Context passed to reset function */
  SemaphoreHandle_t mutex;               /**< Mutex for thread-safe operations */

  esp_err_t (*reset_fn)(void* context); /**< Optional function to reset component */
} error_handler_t;

/* --- Functions --- */

/**
 * @brief Initialize an error handler
 *
 * Initializes an error handler structure with specified retry parameters and
 * reset function. This prepares the error handler for use in detecting and
 * recovering from errors. Creates a mutex for thread-safe operations.
 *
 * @param[in,out] handler          Pointer to error handler structure to initialize
 * @param[in]     max_retries      Maximum number of retry attempts before permanent failure
 * @param[in]     base_retry_delay Initial delay between retries (ms)
 * @param[in]     max_retry_delay  Maximum retry delay (ms) after exponential backoff
 * @param[in]     reset_fn         Optional reset function to attempt recovery
 * @param[in]     reset_context    Context passed to reset function when called
 * @return ESP_OK if successful, ESP_ERR_NO_MEM if mutex creation fails, ESP_ERR_INVALID_ARG if handler is NULL.
 */
/* clang-format off */
esp_err_t error_handler_init(error_handler_t* handler,
                             uint32_t         max_retries,
                             uint32_t         base_retry_delay,
                             uint32_t         max_retry_delay,
                             esp_err_t      (*reset_fn)(void* context),
                             void*            reset_context);
/* clang-format on */

/**
 * @brief Deinitialize an error handler and release resources.
 *
 * Deletes the mutex associated with the error handler. Safe to call even if
 * handler or mutex is NULL or already deinitialized.
 *
 * @param[in,out] handler Pointer to the error handler structure to deinitialize.
 */
void error_handler_deinit(error_handler_t* handler);

/**
 * @brief Record an error and manage retry/reset logic
 *
 * Records an error occurrence, updates error state, and manages the retry
 * mechanism including exponential backoff. If a reset function was provided
 * during initialization and retries are configured, it may be called based
 * on the retry strategy.
 *
 * @param[in,out] handler     Pointer to error handler structure
 * @param[in]     error       Error code that occurred
 * @param[in]     description Human-readable description of the error
 * @param[in]     file        Source file where error occurred (usually __FILE__)
 * @param[in]     line        Line number where error occurred (usually __LINE__)
 * @param[in]     func        Function name where error occurred (usually __func__)
 *
 * @return ESP_OK if error was handled and recovery succeeded (reset function worked),
 *         ESP_ERR_TIMEOUT if mutex couldn't be acquired,
 *         or the original error code if handling was unsuccessful or retries exhausted.
 */
esp_err_t error_handler_record_error(error_handler_t* handler,
                                     esp_err_t        error,
                                     const char*      description,
                                     const char*      file,
                                     int line, /* Its an int since __LINE__ expands to an int */
                                     const char* func);

/**
 * @brief Check if a retry should be attempted
 *
 * Determines if another retry attempt should be made based on the current
 * error state and retry count compared to the maximum retries.
 *
 * @param[in] handler Pointer to error handler structure
 *
 * @return true if a retry is possible and should be attempted,
 *         false if max retries reached, handler is NULL, not in error state, or mutex fails.
 */
bool error_handler_can_retry(error_handler_t* handler);

/**
 * @brief Reset the error handler state
 *
 * Resets all error tracking fields in the error handler to their initial values.
 * This should be called when error recovery has succeeded or when preparing for
 * a new operational phase.
 *
 * @param[in,out] handler Pointer to error handler structure to reset
 * @return ESP_OK if successful, ESP_ERR_TIMEOUT if mutex couldn't be acquired,
 *         ESP_ERR_INVALID_STATE if mutex not init.
 */
esp_err_t error_handler_reset_state(error_handler_t* handler);

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_COMPONENT_ERROR_HANDLER_H */
