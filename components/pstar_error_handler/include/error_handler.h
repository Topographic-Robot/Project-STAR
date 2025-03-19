/* components/pstar_error_handler/include/error_handler.h */

#ifndef PSTAR_ERROR_HANDLER_H
#define PSTAR_ERROR_HANDLER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "error_handler_macros.h"
#include "error_handler_types.h"

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
