#include "system_tasks.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* Constants ******************************************************************/

const char *system_tag = "Topographic-Robot";

/* Public Functions ***********************************************************/

void start_system_tasks(void)
{
  /* 1. Start Wi-Fi handling task pinned to Core 0 */
  xTaskCreatePinnedToCore(wifi_tasks, "wifi_tasks", 4096, NULL, 5, NULL, 0);

  /* 2. Start motor monitoring task pinned to Core 1 */
  xTaskCreatePinnedToCore(motor_tasks, "motor_tasks", 2048, NULL, 5, NULL, 1);

  /* 3. Start sensor data collection task pinned to Core 1 */
  xTaskCreatePinnedToCore(sensor_tasks, "sensor_tasks", 2048, NULL, 5, NULL, 1);

  /* 4. Start webserver video relay task pinned to Core 1 */
  xTaskCreatePinnedToCore(webserver_tasks, "webserver_tasks", 2048, NULL, 5, NULL, 1);

  ESP_LOGI(system_tag, "System tasks started");
}
