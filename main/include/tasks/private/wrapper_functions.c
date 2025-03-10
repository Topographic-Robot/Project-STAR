/* main/include/tasks/private/wrapper_functions.c */

#include "private/wrapper_functions.h"
#include "system_tasks.h"
#include "log_handler.h"
#include "log_storage.h"
#include "sensor_tasks.h"
#include "motor_tasks.h"
#include "gait_movement.h"
#include "ov7670_hal.h"

/* Constants ******************************************************************/

bool g_enable_sd_card_logging = true;
bool g_enable_log_compression = true;

/* Functions ******************************************************************/

/* Log wrapper functions */
esp_err_t wrapper_log_init(void) {
  return log_init(g_enable_sd_card_logging);
}

esp_err_t wrapper_log_storage_set_compression(void) {
  return log_storage_set_compression(g_enable_log_compression);
}

/* Sensor wrapper functions */
esp_err_t wrapper_sensors_init(void) {
  return sensors_init(&g_sensor_data);
}

esp_err_t wrapper_sensor_tasks(void) {
  return sensor_tasks(&g_sensor_data);
}

esp_err_t wrapper_sensor_tasks_stop(void) {
  return sensor_tasks_stop(&g_sensor_data);
}

esp_err_t wrapper_sensors_cleanup(void) {
  return sensors_cleanup(&g_sensor_data);
}

/* Camera wrapper functions */
esp_err_t wrapper_ov7670_init(void) {
  return ov7670_init(&g_camera_data);
}

esp_err_t wrapper_ov7670_cleanup(void) {
  return ov7670_cleanup((void*)&g_camera_data);
}

/* Motor wrapper functions */
esp_err_t wrapper_motors_init(void) {
  return motors_init(&g_pwm_controller);
}

esp_err_t wrapper_motor_tasks_start(void) {
  return motor_tasks_start(g_pwm_controller);
}

esp_err_t wrapper_motor_tasks_stop(void) {
  return motor_tasks_stop(g_pwm_controller);
}

esp_err_t wrapper_motors_cleanup(void) {
  return motors_cleanup(&g_pwm_controller);
}

/* Gait wrapper functions */
esp_err_t wrapper_gait_init(void) {
  return gait_init(g_pwm_controller);
}

esp_err_t wrapper_gait_cleanup(void) {
  return gait_cleanup(g_pwm_controller);
} 