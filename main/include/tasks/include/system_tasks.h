#ifndef TOPOROBO_SYSTEM_TASKS_H
#define TOPOROBO_SYSTEM_TASKS_H

#include "wifi_tasks.h"
#include "sensor_tasks.h"
#include "motor_tasks.h"
#include "webserver_tasks.h"

/* Constants ******************************************************************/

extern const char *system_tag;

/* Public Functions ***********************************************************/

/**
 * @brief Starts system-level tasks for handling devices and communication.
 * 
 * This function creates and starts tasks that monitor motors, collect sensor data,
 * relay video to the webserver, and handle Wi-Fi operations. Tasks are pinned 
 * to the appropriate cores based on their functionality.
 * 
 * - Wi-Fi handling is pinned to Core 0.
 * - Motor monitoring is pinned to Core 1.
 * - Sensor data collection is pinned to Core 1.
 * - Webserver video relay is pinned to Core 1.
 */
void system_tasks_start(void);

/**
 * @brief Initialize system-level tasks for handling devices and communication.
 * 
 * This function creates and initializes tasks that monitor motors, collect 
 * sensor data, relay video to the webserver, and handle Wi-Fi operations. 
 */
void system_tasks_init(void);

#endif /* TOPOROBO_SYSTEM_TASKS_H */
