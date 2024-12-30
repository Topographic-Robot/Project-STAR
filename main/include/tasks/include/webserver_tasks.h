/* main/include/tasks/include/webserver_tasks.h */

#ifndef TOPOROBO_WEBSERVER_TASKS_H
#define TOPOROBO_WEBSERVER_TASKS_H

#include "esp_err.h"

/* Public Functions ***********************************************************/

/**
 * @brief Sends a JSON string to the web server.
 *
 * @param json_string Pointer to the JSON string to send.
 * @return esp_err_t ESP_OK if data sent successfully, error code otherwise.
 */
esp_err_t send_sensor_data_to_webserver(const char *json_string);

#endif /* TOPOROBO_WEBSERVER_TASKS_H */
