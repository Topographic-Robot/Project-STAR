/* components/pstar_logging/include/log_macros.h */

#ifndef PSTAR_LOG_MACROS_H
#define PSTAR_LOG_MACROS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Macros *********************************************************************/

#define PSTAR_LOGGING_MAX_MESSAGE_LENGTH         (256)                                  /* Maximum length of log messages */
#define PSTAR_LOGGING_MAX_TAG_LENGTH             (32)                                   /* Maximum length of log tags */
#define PSTAR_LOGGING_SEPARATOR                  (" - ")                                /* Separator between log components */
#define PSTAR_LOGGING_TASK_NAME_LENGTH           (16)                                   /* Maximum length of task name to display */
#define PSTAR_LOGGING_BUFFER_SIZE                (10)                                   /* Size of the log buffer for temporary storage */
#define PSTAR_LOGGING_TIMESTAMP_BUFFER_SIZE      (64)                                   /* Buffer size for formatted timestamp strings */
#define PSTAR_LOGGING_DATE_STRING_BUFFER_SIZE    (32)                                   /* Buffer size for date strings */
#define PSTAR_LOGGING_MAX_FORMATTED_ENTRY_LENGTH (PSTAR_LOGGING_MAX_MESSAGE_LENGTH * 2) /* Formatted log entry buffer size */

#ifdef __cplusplus
}
#endif

#endif /* PSTAR_LOG_MACROS_H */
