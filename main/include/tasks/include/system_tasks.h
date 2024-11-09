/* main/include/tasks/include/system_tasks.h */

#ifndef TOPOROBO_SYSTEM_TASKS_H
#define TOPOROBO_SYSTEM_TASKS_H

#include <esp_err.h>
#include "wifi_tasks.h"
#include "sensor_tasks.h"
#include "motor_tasks.h"
#include "webserver_tasks.h"

/* Constants ******************************************************************/

/** 
 * @brief Global logging tag used across the project for consistent logging.
 */
extern const char *system_tag;

/* Public Functions ***********************************************************/

/**
 * @brief Initializes system-level tasks for handling devices and communication.
 * 
 * This function initializes tasks that monitor motors, collect sensor data, 
 * relay video to the web server, and handle Wi-Fi operations. 
 * 
 * @return ESP_OK if initialization succeeds; ESP_FAIL otherwise.
 */
esp_err_t system_tasks_init(void);

/**
 * @brief Starts system-level tasks for handling devices and communication.
 * 
 * This function creates and starts tasks that monitor motors, collect sensor data,
 * relay video to the web server, and handle Wi-Fi operations. Tasks are pinned 
 * to the appropriate cores based on their functionality.
 * 
 * - Wi-Fi handling is pinned to Core 0.
 * - Motor monitoring is pinned to Core 1.
 * - Sensor data collection is pinned to Core 1.
 * - Webserver video relay is pinned to Core 1.
 * 
 * @return ESP_OK if all tasks start successfully; ESP_FAIL if any task fails.
 */
esp_err_t system_tasks_start(void);

#endif /* TOPOROBO_SYSTEM_TASKS_H */

