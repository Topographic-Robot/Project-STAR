idf_component_register(
  SRCS
    "file_write_manager.c"
    "time_manager.c"
  INCLUDE_DIRS
    "include"
  REQUIRES
    freertos
    driver
    esp_common
    esp_timer
    esp_netif
    pstar_storage_hal
)

message(STATUS "pstar_managers: All managers enabled")