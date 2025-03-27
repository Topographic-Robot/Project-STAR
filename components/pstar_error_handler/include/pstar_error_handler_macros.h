/* components/pstar_error_handler/include/error_handler_macros.h */

#ifndef PSTAR_ERROR_HANDLER_MACROS_H
#define PSTAR_ERROR_HANDLER_MACROS_H

#include "pstar_error_handler.h" 

/* Macros *********************************************************************/

/**
 * @brief Record an error and manage retry/reset logic
 *
 * This macro simplifies error recording by automatically passing
 * the current source file, line number, and function name to the error handler.
 *
 * @param[in] handler Pointer to error handler structure
 * @param[in] code    Error code to record
 * @param[in] desc    Human-readable description of the error
 */
#define RECORD_ERROR(handler, code, desc)                                                \
  do {                                                                                   \
    error_handler_record_error((handler), (code), (desc), __FILE__, __LINE__, __func__); \
  } while (0)

#endif /* PSTAR_ERROR_HANDLER_MACROS_H */