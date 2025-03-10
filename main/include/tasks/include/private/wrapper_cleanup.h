/* main/include/tasks/include/private/wrapper_cleanup.h */

#ifndef TOPOROBO_WRAPPER_CLEANUP_H
#define TOPOROBO_WRAPPER_CLEANUP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/* Functions ******************************************************************/

/**
 * @brief Wrapper functions for system component cleanup
 * 
 * These functions provide a common interface for cleaning up different
 * subsystems and components. They internally call the appropriate cleanup
 * functions with the required parameters.
 */

/* Sensor cleanup wrapper functions */
esp_err_t wrapper_cleanup_all_sensors(void);

/* Individual sensor cleanup wrappers */
esp_err_t wrapper_qmc5883l_cleanup(void);
esp_err_t wrapper_mq135_cleanup(void);
esp_err_t wrapper_mpu6050_cleanup(void);
esp_err_t wrapper_bh1750_cleanup(void);
esp_err_t wrapper_dht22_cleanup(void);
esp_err_t wrapper_gy_neo6mv2_cleanup(void);
esp_err_t wrapper_ccs811_cleanup(void);

/* System cleanup wrapper functions */
esp_err_t wrapper_cleanup_all_system(void);
esp_err_t wrapper_log_cleanup(void);
esp_err_t wrapper_file_write_manager_cleanup(void);

/* Motor and movement cleanup wrapper functions */
esp_err_t wrapper_cleanup_all_motors(void);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_WRAPPER_CLEANUP_H */ 