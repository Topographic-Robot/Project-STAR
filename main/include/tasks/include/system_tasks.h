/* main/include/tasks/include/system_tasks.h */

#ifndef TOPOROBO_SYSTEM_TASKS_H
#define TOPOROBO_SYSTEM_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "sensor_tasks.h"
#include "wifi_tasks.h"
#include "motor_tasks.h"
#include "pca9685_hal.h"
#include "gait_movement.h"
#include "ov7670_hal.h"

/* Constants ******************************************************************/

extern const char* const system_tag; /**< Logging tag */

/* Globals ********************************************************************/

extern sensor_data_t    g_sensor_data;    /**< Global variable that holds the sensor data */
extern pca9685_board_t* g_pwm_controller; /**< Global variable that holds the PWM controller linked list */
/* TODO: Make this support all 6 cameras */
extern ov7670_data_t    g_camera_data;    /**< Global variable that holds the camera data */

/* Structs ********************************************************************/

/**
 * @brief Configuration structure for system components
 * 
 * This structure defines the interface for system components that can be
 * initialized, started, stopped, and cleaned up. Each component has a name,
 * function pointers for lifecycle operations, and an enabled flag.
 * 
 * All function pointers must have a void parameter signature. For functions
 * that require parameters, wrapper functions must be created.
 */
typedef struct system_component_config {
  const char* component_name;          /**< Name of the component for logging */
  esp_err_t (*init_function)(void);    /**< Function to initialize the component */
  esp_err_t (*start_function)(void);   /**< Function to start the component */
  esp_err_t (*stop_function)(void);    /**< Function to stop the component */
  esp_err_t (*cleanup_function)(void); /**< Function to clean up the component */
  bool        enabled;                 /**< Whether the component is enabled */
} system_component_config_t;

/* Public Functions ***********************************************************/

/**
 * @brief Initializes system-level tasks for handling devices and communication.
 *
 * Prepares system tasks required for managing hardware devices and communication 
 * subsystems. This includes setting up resources for motor monitoring, sensor 
 * data collection, video streaming to the web server, and Wi-Fi operations. 
 * Initialization ensures that each task has the necessary prerequisites to start.
 *
 * @return 
 * - ESP_OK   if all system-level tasks are initialized successfully.
 * - ESP_FAIL if any initialization step fails.
 *
 * @note This function should be called once during system setup, prior to starting 
 *       the tasks with `system_tasks_start`.
 */
esp_err_t system_tasks_init(void);

/**
 * @brief Starts system-level tasks for handling devices and communication.
 *
 * Creates and starts tasks responsible for the following functions:
 * - Monitoring and controlling motors.
 * - Collecting and processing sensor data.
 * - Relaying video streams to the web server.
 * - Managing Wi-Fi connectivity and related operations.
 *
 * Tasks are pinned to the appropriate ESP32 cores to optimize performance and 
 * ensure efficient resource utilization:
 * - Core 0: Wi-Fi handling.
 * - Core 1: Motor monitoring, sensor data collection, and webserver video relay.
 *
 * @return 
 * - ESP_OK   if all tasks start successfully.
 * - ESP_FAIL if any task fails to start.
 *
 * @note Ensure that `system_tasks_init` has been called successfully before invoking 
 *       this function. Task creation may fail if system resources are insufficient 
 *       or prerequisites are unmet.
 */
esp_err_t system_tasks_start(void);

/**
 * @brief Stops all system-level tasks and cleans up associated resources
 * 
 * Performs a graceful shutdown of all system components in the reverse order
 * they were initialized:
 * 1. Stops motor control tasks
 * 2. Stops sensor monitoring tasks
 * 3. Stops camera monitoring tasks (if enabled)
 * 4. Stops WiFi tasks
 * 5. Cleans up file write manager
 * 6. Cleans up gait system
 * 7. Cleans up motor controllers
 * 8. Cleans up camera subsystem
 * 9. Cleans up sensor subsystem
 * 10. Finalizes logging system
 * 
 * @return ESP_OK if all components shut down successfully, or ESP_FAIL if any component
 *         failed to shut down properly
 */
esp_err_t system_tasks_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* TOPOROBO_SYSTEM_TASKS_H */

