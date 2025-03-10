/* main/include/tasks/private/wrapper_cleanup.c */

#include "private/wrapper_cleanup.h"
#include "private/wrapper_functions.h"
#include "system_tasks.h"
#include "log_handler.h"
#include "log_storage.h"
#include "sensor_tasks.h"
#include "motor_tasks.h"
#include "gait_movement.h"
#include "ov7670_hal.h"
#include "common/common_cleanup.h"
#include "file_write_manager.h"

/* Constants ******************************************************************/

/* Functions ******************************************************************/

/* Sensor cleanup wrapper functions */
esp_err_t wrapper_cleanup_all_sensors(void)
{
  log_info(system_tag, "Cleanup Start", "Beginning cleanup of all sensors");
  
  esp_err_t (*(cleanup_funcs[]))(void) = {
    wrapper_qmc5883l_cleanup,
    wrapper_mq135_cleanup,
    wrapper_mpu6050_cleanup,
    wrapper_bh1750_cleanup,
    wrapper_dht22_cleanup,
    wrapper_gy_neo6mv2_cleanup,
    wrapper_ccs811_cleanup
  };
  
  return common_cleanup_multiple(system_tag, 
                                 "Sensors", 
                                 cleanup_funcs, 
                                 sizeof(cleanup_funcs) / sizeof(cleanup_funcs[0]));
}

/* Individual sensor cleanup wrappers */
esp_err_t wrapper_qmc5883l_cleanup(void)
{
  return qmc5883l_cleanup(&(g_sensor_data.qmc5883l_data));
}

esp_err_t wrapper_mq135_cleanup(void)
{
  return mq135_cleanup(&(g_sensor_data.mq135_data));
}

esp_err_t wrapper_mpu6050_cleanup(void)
{
  return mpu6050_cleanup(&(g_sensor_data.mpu6050_data));
}

esp_err_t wrapper_bh1750_cleanup(void)
{
  return bh1750_cleanup(&(g_sensor_data.bh1750_data));
}

esp_err_t wrapper_dht22_cleanup(void)
{
  return dht22_cleanup(&(g_sensor_data.dht22_data));
}

esp_err_t wrapper_gy_neo6mv2_cleanup(void)
{
  return gy_neo6mv2_cleanup(&(g_sensor_data.gy_neo6mv2_data));
}

esp_err_t wrapper_ccs811_cleanup(void)
{
  return ccs811_cleanup(&(g_sensor_data.ccs811_data));
}

/* System cleanup wrapper functions */
esp_err_t wrapper_cleanup_all_system(void)
{
  log_info(system_tag, "Cleanup Start", "Beginning cleanup of all system components");
  
  esp_err_t (*cleanup_funcs[])(void) = {
    wrapper_log_cleanup,
    wrapper_file_write_manager_cleanup
  };
  
  return common_cleanup_multiple(system_tag, 
                                 "System", 
                                 cleanup_funcs, 
                                 sizeof(cleanup_funcs) / sizeof(cleanup_funcs[0]));
}

esp_err_t wrapper_log_cleanup(void)
{
  return log_cleanup();
}

esp_err_t wrapper_file_write_manager_cleanup(void)
{
  return file_write_manager_cleanup();
}

/* Motor and movement cleanup wrapper functions */
esp_err_t wrapper_cleanup_all_motors(void)
{
  log_info(system_tag, "Cleanup Start", "Beginning cleanup of all motor components");
  
  esp_err_t (*cleanup_funcs[])(void) = {
    wrapper_motors_cleanup,
    wrapper_gait_cleanup
  };
  
  return common_cleanup_multiple(system_tag, 
                                 "Motors", 
                                 cleanup_funcs, 
                                 sizeof(cleanup_funcs) / sizeof(cleanup_funcs[0]));
} 