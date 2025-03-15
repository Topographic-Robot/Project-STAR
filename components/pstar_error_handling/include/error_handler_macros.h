/* components/pstar_error_handling/include/error_handler_macros.h */

#ifndef PSTAR_ERROR_HANDLER_MACROS_H
#define PSTAR_ERROR_HANDLER_MACROS_H

#include "error_handler.h"

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

#endif /* PSTAR_ERROR_HANDLER_MACROS_H */
