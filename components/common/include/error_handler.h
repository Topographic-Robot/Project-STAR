/* components/common/include/error_handler.h */

#ifndef TOPOROBO_ERROR_HANDLER_H
#define TOPOROBO_ERROR_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "log_handler.h"

/* Constants ******************************************************************/

extern const char* const error_handler_tag; /**< Tag for logging messages */

/* Macros *********************************************************************/

#define MAX_COMPONENTS     (32) /**< Maximum number of registered components */
#define MAX_CORRELATIONS   (32) /**< Maximum number of error correlations to track */
#define MAX_ROOT_CAUSES    (16) /**< Maximum number of root causes to track */
#define MAX_RELATED_ERRORS (8)  /**< Maximum number of related errors per correlation */

/**
 * @brief Records an error with the specified handler.
 */
#define ERROR_RECORD(handler, code, category, severity, desc) \
  do { \
    error_handler_record_error((handler), (code), (category), (severity), __FILE__, __LINE__, __func__, (desc)); \
  } while (0)

/**
 * @brief Records a hardware error with the specified handler.
 */
#define ERROR_HARDWARE(handler, code, severity, desc) \
  do { \
    ERROR_RECORD((handler), (code), k_error_category_hardware, (severity), (desc)); \
  } while (0)

/**
 * @brief Records a communication error with the specified handler.
 */
#define ERROR_COMMUNICATION(handler, code, severity, desc) \
  do { \
    ERROR_RECORD((handler), (code), k_error_category_communication, (severity), (desc)); \
  } while (0)

/**
 * @brief Records a memory error
 */
#define ERROR_MEMORY(handler, code, severity, desc) \
  do { \
    ERROR_RECORD((handler), (code), k_error_category_memory, (severity), (desc)); \
  } while (0)

/**
 * @brief Records a system error
 */
#define ERROR_SYSTEM(handler, code, severity, desc) \
  do { \
    ERROR_RECORD((handler), (code), k_error_category_system, (severity), (desc)); \
  } while (0)

/**
 * @brief Records an application error
 */
#define ERROR_APPLICATION(handler, code, severity, desc) \
  do { \
    ERROR_RECORD((handler), (code), k_error_category_application, (severity), (desc)); \
  } while (0)

/* Backward compatibility macros for error categories */
#define ERROR_CATEGORY_NONE k_error_category_none
#define ERROR_CATEGORY_HARDWARE k_error_category_hardware
#define ERROR_CATEGORY_COMMUNICATION k_error_category_communication
#define ERROR_CATEGORY_MEMORY k_error_category_memory
#define ERROR_CATEGORY_SYSTEM k_error_category_system
#define ERROR_CATEGORY_APPLICATION k_error_category_application
#define ERROR_CATEGORY_COUNT k_error_category_count

/* Backward compatibility macros for error severity levels */
#define ERROR_SEVERITY_LOW k_error_severity_low
#define ERROR_SEVERITY_MEDIUM k_error_severity_medium
#define ERROR_SEVERITY_HIGH k_error_severity_high
#define ERROR_SEVERITY_CRITICAL k_error_severity_critical
#define ERROR_SEVERITY_FATAL k_error_severity_fatal
#define ERROR_SEVERITY_COUNT k_error_severity_count

/* Enums **********************************************************************/

/**
 * @brief Error categories.
 */
typedef enum error_category {
  k_error_category_none = 0,         /**< No category */
  k_error_category_hardware,         /**< Hardware error */
  k_error_category_communication,    /**< Communication error */
  k_error_category_memory,           /**< Memory error */
  k_error_category_system,           /**< System error */
  k_error_category_application,      /**< Application error */
  k_error_category_count,            /**< Number of categories */
} error_category_t;

/**
 * @brief Error severity levels
 */
typedef enum error_severity {
  k_error_severity_low = 0,      /**< Low severity error, can be ignored */
  k_error_severity_medium,       /**< Medium severity error, should be addressed */
  k_error_severity_high,         /**< High severity error, must be addressed */
  k_error_severity_critical,     /**< Critical error, system may be unstable */
  k_error_severity_fatal,        /**< Fatal error, system cannot continue */
  k_error_severity_count,        /**< Number of severity levels */
} error_severity_t;

/**
 * @brief Recovery strategy types
 */
typedef enum : uint8_t {
  k_recovery_strategy_none,      /**< No recovery strategy defined */
  k_recovery_strategy_retry,     /**< Simple retry with optional backoff */
  k_recovery_strategy_reset,     /**< Reset the component */
  k_recovery_strategy_alternate, /**< Use alternate method/component */
  k_recovery_strategy_degrade,   /**< Continue with degraded functionality */
  k_recovery_strategy_shutdown,  /**< Shut down the component safely */
  k_recovery_strategy_custom,    /**< Custom recovery strategy */
  k_recovery_strategy_count,     /**< Number of recovery strategies */
} recovery_strategy_t;

/* Forward Declarations *******************************************************/

struct error_info;
struct recovery_context;
struct error_correlation;
struct root_cause_info;
struct error_handler;
struct component_info;

typedef struct error_info error_info_t;
typedef struct recovery_context recovery_context_t;
typedef struct error_correlation error_correlation_t;
typedef struct root_cause_info root_cause_info_t;
typedef struct error_handler error_handler_t;
typedef struct component_info component_info_t;

/* Function Types *************************************************************/

typedef esp_err_t (*error_handler_func_t)(error_info_t* error_info, void* context); /**< Function type for custom error handlers */
typedef esp_err_t (*recovery_func_t)(recovery_context_t* context);                  /**< Function type for custom recovery functions */
typedef esp_err_t (*reset_func_t)(void* context);                                   /**< Function type for component reset */
typedef esp_err_t (*shutdown_func_t)(void* context);                                /**< Function type for component shutdown */

/* Structs ********************************************************************/

/**
 * @brief Extended error information structure
 */
struct error_info {
  esp_err_t        code;           /**< Error code */
  error_category_t category;       /**< Error category */
  error_severity_t severity;       /**< Error severity */
  const char*      file;           /**< Source file where error occurred */
  int              line;           /**< Line number where error occurred */
  const char*      func;           /**< Function where error occurred */
  const char*      description;    /**< Human-readable error description */
  uint64_t         timestamp;      /**< Timestamp when error occurred */
  uint32_t         count;          /**< Number of times this error has occurred */
  uint32_t         correlation_id; /**< Correlation ID for related errors */
  const char*      component_id;   /**< Component ID where error occurred */
  bool             is_filtered;    /**< Whether this error is filtered */
  uint32_t         priority;       /**< Error priority (lower is higher priority) */
};

/**
 * @brief Recovery context for custom recovery functions
 */
struct recovery_context {
  void*               context;      /**< User context for recovery function */
  recovery_strategy_t strategy;     /**< Recovery strategy to use */
  uint32_t            attempt;      /**< Current recovery attempt */
  uint32_t            max_attempts; /**< Maximum recovery attempts */
  TickType_t          timeout;      /**< Timeout for recovery attempt */
  bool                in_progress;  /**< Whether recovery is in progress */
  bool                in_degraded_mode; /**< Whether operating in degraded mode */
  uint32_t            retry_count;  /**< Number of retries attempted */
  esp_err_t           last_error;   /**< Last error encountered during recovery */
};

/**
 * @brief Error correlation information
 */
struct error_correlation {
  uint32_t        correlation_id;   /**< Unique correlation ID for related errors */
  uint32_t        related_count;    /**< Number of related errors */
  error_info_t*   related_errors;   /**< Array of related errors */
  uint64_t        first_occurrence; /**< Timestamp of first occurrence */
  uint64_t        last_occurrence;  /**< Timestamp of last occurrence */
  bool            is_root_cause;    /**< Whether this error is a root cause */
};

/**
 * @brief Root cause analysis information
 */
struct root_cause_info {
  error_info_t    root_error;       /**< The root cause error */
  uint32_t        dependent_count;  /**< Number of dependent errors */
  error_info_t*   dependent_errors; /**< Array of dependent errors */
  const char*     diagnosis;        /**< Diagnostic information */
  float           confidence;       /**< Confidence level (0.0-1.0) */
};

/**
 * @brief Data structure for managing component state and recovery.
 *
 * Contains essential data for implementing exponential backoff and retry logic
 * when errors occur, as well as tracking successful states in the system.
 * Each component can configure its own retry policy and reset behavior.
 * 
 * The structure is organized into four main sections:
 * 1. Core error handling and retry logic - manages retry attempts, intervals, and component state
 * 2. Error recovery mechanisms - handles strategies and functions for recovering from errors
 * 3. Error filtering and correlation - manages error categorization, filtering, and relationship tracking
 * 4. Root cause analysis - provides capabilities for identifying and tracking error origins
 */
struct error_handler {
  /* Core error handling and retry logic */
  uint8_t              retry_count;              /**< Counter for consecutive retry attempts */
  uint32_t             retry_interval;           /**< Current interval between retry attempts */
  uint32_t             initial_interval;         /**< Initial retry interval in ticks */
  uint32_t             initial_backoff_interval; /**< Initial backoff interval in ticks */
  uint32_t             max_interval;             /**< Maximum backoff interval in ticks */
  uint32_t             max_backoff_interval;     /**< Maximum backoff interval in ticks */
  uint8_t              max_retries;              /**< Maximum number of retry attempts before giving up */
  TickType_t           last_attempt_ticks;       /**< Tick count of the last retry attempt */
  esp_err_t            last_status;              /**< Last status code encountered */
  bool                 in_error_state;           /**< Whether the component is in an error state */
  bool                 permanently_failed;       /**< Whether the component has permanently failed */
  const char*          tag;                      /**< Tag for log_handler messages */
  void*                context;                  /**< Context pointer for the reset function */
  reset_func_t         reset_func;               /**< Function to call during reset, returns ESP_OK on success */
  shutdown_func_t      shutdown_func;            /**< Function to call during shutdown */
  SemaphoreHandle_t    mutex;                    /**< Mutex for thread safety */
  error_info_t         last_error;               /**< Detailed information about the last error */
  uint32_t             error_count;              /**< Total number of errors encountered */
  uint32_t             recovery_count;           /**< Total number of successful recoveries */
  bool                 propagate_errors;         /**< Whether to propagate errors to parent components */
  
  /* Error recovery mechanisms */
  recovery_strategy_t  recovery_strategy;        /**< Current recovery strategy */
  recovery_func_t      recovery_func;            /**< Custom recovery function */
  recovery_context_t   recovery_context;         /**< Recovery context */
  error_handler_func_t error_callback;           /**< Callback for error notifications */
  error_handler_func_t permanent_failure_callback; /**< Callback for permanent failure notifications */
  void*                callback_context;         /**< Context for error callback */
  
  /* Error filtering and correlation */
  error_severity_t     min_severity_to_handle;   /**< Minimum severity level to handle */
  error_category_t     categories_to_filter;     /**< Bitmap of categories to filter */
  uint32_t             next_correlation_id;      /**< Next correlation ID to assign */
  error_correlation_t* correlations;             /**< Array of error correlations */
  uint32_t             correlation_count;        /**< Number of correlations */
  uint32_t             max_correlations;         /**< Maximum number of correlations to track */
  
  /* Root cause analysis */
  root_cause_info_t*   root_causes;              /**< Array of root causes */
  uint32_t             root_cause_count;         /**< Number of root causes */
  uint32_t             max_root_causes;          /**< Maximum number of root causes to track */
};

/**
 * @brief Component registration information
 */
struct component_info {
  const char*      component_id; /**< Unique component identifier */
  error_handler_t* handler;      /**< Pointer to component's error handler */
  const char*      parent_id;    /**< Parent component ID for error propagation */
  uint32_t         priority;     /**< Component priority for error handling */
};

/* Public Functions ***********************************************************/

/**
 * @brief Initializes the error handling system
 * 
 * Must be called before any other error handling functions.
 * 
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_system_init(void);

/**
 * @brief Initializes an error handler.
 *
 * @param handler Pointer to the error handler to initialize.
 * @param tag Tag for log messages.
 * @param max_retries Maximum number of retry attempts.
 * @param initial_interval Initial retry interval in ticks.
 * @param max_interval Maximum retry interval in ticks.
 * @param reset_func Function to call during reset.
 * @param context Context pointer for the reset function.
 * @param initial_backoff_interval Initial backoff interval in ticks.
 * @param max_backoff_interval Maximum backoff interval in ticks.
 */
void error_handler_init(error_handler_t*  handler, 
                        const char* const tag,
                        uint8_t           max_retries, 
                        uint32_t          initial_interval,
                        uint32_t          max_interval, 
                        reset_func_t      reset_func,
                        void*             context, 
                        uint32_t          initial_backoff_interval,
                        uint32_t          max_backoff_interval);

/**
 * @brief Records a new status and updates the handler state.
 *
 * This function handles the exponential backoff and retry logic based on the
 * status provided. It will apply the following rules:
 *   - If the status is ESP_OK, reset the retry counter and state
 *   - If in an error state, increase the backoff interval exponentially
 *   - If the max retries is reached, put the component in an error state
 *
 * @param[in,out] handler Pointer to the error_handler_t structure to update
 * @param[in]     status  The status code to record
 * @return
 * - ESP_OK                if the status was handled successfully
 * - ESP_ERR_INVALID_ARG   if handler is NULL
 * - ESP_ERR_INVALID_STATE if in backoff period
 * - Other error codes from the reset function
 */
esp_err_t error_handler_record_status(error_handler_t* const handler, 
                                      esp_err_t              status);

/**
 * @brief Records a detailed error with context information
 * 
 * @param[in,out] handler    Pointer to the error_handler_t structure
 * @param[in]     code       Error code
 * @param[in]     category   Error category
 * @param[in]     severity   Error severity
 * @param[in]     file       Source file where error occurred
 * @param[in]     line       Line number where error occurred
 * @param[in]     func       Function where error occurred
 * @param[in]     desc       Human-readable error description
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_record_error(error_handler_t* const handler,
                                     esp_err_t              code,
                                     error_category_t       category,
                                     error_severity_t       severity,
                                     const char*            file,
                                     int                    line,
                                     const char*            func,
                                     const char*            desc);

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
esp_err_t error_handler_reset(error_handler_t* const handler);

/**
 * @brief Registers a component with the error handling system
 * 
 * @param[in] component_info Component registration information
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_register_component(const component_info_t* component_info);

/**
 * @brief Sets a callback function for error notifications
 * 
 * @param[in,out] handler  Pointer to the error_handler_t structure
 * @param[in]     callback Callback function
 * @param[in]     context  Context for callback function
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_set_callback(error_handler_t* const handler,
                                     error_handler_func_t   callback,
                                     void*                  context);

/**
 * @brief Sets a custom recovery function
 * 
 * @param[in,out] handler  Pointer to the error_handler_t structure
 * @param[in]     strategy Recovery strategy to use
 * @param[in]     func     Recovery function
 * @param[in]     context  Context for recovery function
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_set_recovery(error_handler_t* const handler,
                                     recovery_strategy_t    strategy,
                                     recovery_func_t        func,
                                     void*                  context);

/**
 * @brief Starts the recovery process for a component
 * 
 * @param[in,out] handler Pointer to the error_handler_t structure
 * @return ESP_OK if recovery started, error code otherwise
 */
esp_err_t error_handler_start_recovery(error_handler_t* const handler);

/**
 * @brief Gets the current status of the error handler
 * 
 * @param[in]  handler Pointer to the error_handler_t structure
 * @param[out] info    Pointer to store error information
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_get_status(const error_handler_t* const handler,
                                   error_info_t*                info);

/**
 * @brief Processes pending errors in the system
 * 
 * This function should be called periodically to process errors
 * and attempt recovery.
 * 
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_process(void);

/**
 * @brief Converts an error category to a string
 * 
 * @param[in] category Error category
 * @return String representation of the category
 */
const char* error_category_to_string(error_category_t category);

/**
 * @brief Converts an error severity to a string
 * 
 * @param[in] severity Error severity
 * @return String representation of the severity
 */
const char* error_severity_to_string(error_severity_t severity);

/**
 * @brief Converts a recovery strategy to a string
 * 
 * @param[in] strategy Recovery strategy
 * @return String representation of the strategy
 */
const char* recovery_strategy_to_string(recovery_strategy_t strategy);


/**
 * @brief Sets error filtering parameters
 * 
 * @param[in,out] handler      Pointer to the error_handler_t structure
 * @param[in]     min_severity Minimum severity level to handle
 * @param[in]     categories   Bitmap of categories to filter
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_set_filtering(error_handler_t* const handler,
                                      error_severity_t       min_severity,
                                      uint32_t               categories);

/**
 * @brief Correlates related errors
 * 
 * @param[in,out] handler       Pointer to the error_handler_t structure
 * @param[in]     error_info    Error information to correlate
 * @param[in]     related_error Related error information
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_correlate_errors(error_handler_t* const handler,
                                         error_info_t*          error_info,
                                         error_info_t*          related_error);

/**
 * @brief Gets correlated errors for a given error
 * 
 * @param[in]  handler        Pointer to the error_handler_t structure
 * @param[in]  correlation_id Correlation ID
 * @param[out] correlation    Pointer to store correlation information
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_get_correlation(const error_handler_t* const handler,
                                        uint32_t                     correlation_id,
                                        error_correlation_t*         correlation);

/**
 * @brief Analyzes root cause of errors
 * 
 * @param[in,out] handler    Pointer to the error_handler_t structure
 * @param[in]     error_info Error information to analyze
 * @param[out]    root_cause Pointer to store root cause information
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_analyze_root_cause(error_handler_t* const handler,
                                           error_info_t*          error_info,
                                           root_cause_info_t*     root_cause);

/**
 * @brief Gets the root cause for a given error
 * 
 * @param[in]  handler    Pointer to the error_handler_t structure
 * @param[in]  error_info Error information
 * @param[out] root_cause Pointer to store root cause information
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_get_root_cause(const error_handler_t* const handler,
                                       const error_info_t*          error_info,
                                       root_cause_info_t*           root_cause);

/**
 * @brief Sets a shutdown function for the component
 * 
 * @param[in,out] handler  Pointer to the error_handler_t structure
 * @param[in]     shutdown_func Shutdown function
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_set_shutdown_func(error_handler_t* const handler,
                                          shutdown_func_t        shutdown_func);

/**
 * @brief Cleans up resources used by the error handler
 * 
 * @param[in,out] handler Pointer to the error_handler_t structure
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_cleanup(error_handler_t* const handler);

/**
 * @brief Cleans up the error handling system
 * 
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_system_cleanup(void);

/**
 * @brief Purges old correlations to free memory
 * 
 * @param[in,out] handler Pointer to the error_handler_t structure
 * @param[in]     age_ms  Maximum age in milliseconds to keep
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_purge_old_correlations(error_handler_t* const handler, uint64_t age_ms);

/**
 * @brief Sets a callback for permanent failure notification
 * 
 * @param[in,out] handler  Pointer to the error_handler_t structure
 * @param[in]     callback Callback function for permanent failures
 * @param[in]     context  Context for callback function
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_set_failure_callback(error_handler_t* const handler,
                                             error_handler_func_t   callback,
                                             void*                  context);

/**
 * @brief Checks if a component has permanently failed
 * 
 * @param[in] handler Pointer to the error_handler_t structure
 * @return true if permanently failed, false otherwise
 */
bool error_handler_is_permanently_failed(const error_handler_t* const handler);

/**
 * @brief Unregisters a component from the error handling system
 * 
 * @param[in] component_id Component ID to unregister
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_unregister_component(const char* component_id);

/**
 * @brief Gets statistics for a component's error handler
 * 
 * @param[in]  handler    Pointer to the error_handler_t structure
 * @param[out] error_count Pointer to store error count
 * @param[out] recovery_count Pointer to store recovery count
 * @param[out] in_error_state Pointer to store error state
 * @param[out] permanently_failed Pointer to store permanent failure state
 * @return ESP_OK if successful, error code otherwise
 */
esp_err_t error_handler_get_stats(const error_handler_t* const handler,
                                  uint32_t*                    error_count,
                                  uint32_t*                    recovery_count,
                                  bool*                        in_error_state,
                                  bool*                        permanently_failed);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_ERROR_HANDLER_H */