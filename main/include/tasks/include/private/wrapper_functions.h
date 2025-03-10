/* main/include/tasks/private/wrapper_functions.h */

#ifndef TOPOROBO_WRAPPER_FUNCTIONS_H
#define TOPOROBO_WRAPPER_FUNCTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "stdbool.h"

/* Constants ******************************************************************/

extern bool g_enable_sd_card_logging; /**< Enable SD card logging */
extern bool g_enable_log_compression; /**< Enable log compression */

/* Functions ******************************************************************/

/**
 * @brief Wrapper functions for system components
 * 
 * These functions adapt functions with parameters to the void parameter signature
 * required by the system component configuration structure. They internally call
 * the original functions with the appropriate global variables.
 */

/* Log wrapper functions */
esp_err_t wrapper_log_init(void);
esp_err_t wrapper_log_storage_set_compression(void);

/* Sensor wrapper functions */
esp_err_t wrapper_sensors_init(void);
esp_err_t wrapper_sensor_tasks(void);
esp_err_t wrapper_sensor_tasks_stop(void);
esp_err_t wrapper_sensors_cleanup(void);

/* Camera wrapper functions */
esp_err_t wrapper_ov7670_init(void);
esp_err_t wrapper_ov7670_cleanup(void);

/* Motor wrapper functions */
esp_err_t wrapper_motors_init(void);
esp_err_t wrapper_motor_tasks_start(void);
esp_err_t wrapper_motor_tasks_stop(void);
esp_err_t wrapper_motors_cleanup(void);

/* Gait wrapper functions */
esp_err_t wrapper_gait_init(void);
esp_err_t wrapper_gait_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_WRAPPER_FUNCTIONS_H */ 