idf_component_register(
  SRCS
    "time_manager.c"
    "file_write_manager.c"
  INCLUDE_DIRS
    "include"
  REQUIRES
    pstar_logging
    pstar_storage_hal
    pstar_bus
    esp_netif
)

