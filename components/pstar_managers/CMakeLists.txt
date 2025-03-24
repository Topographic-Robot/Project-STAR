# Define the base list of required components
set(REQS
    freertos
    driver
    esp_common
    esp_timer
    esp_netif
)

# Conditionally add the pstar_storage_hal dependency based on its Kconfig flag.
if(CONFIG_PSTAR_KCONFIG_STORAGE_HAL_ENABLED)
  list(APPEND REQS pstar_storage_hal)
  message(STATUS "pstar_managers: Including pstar_storage_hal dependency")
else()
  message(STATUS "pstar_managers: Skipping dependency pstar_storage_hal as it is disabled via Kconfig")
endif()

idf_component_register(
  SRCS
    "file_write_manager.c"
    "time_manager.c"
  INCLUDE_DIRS "include"
  REQUIRES ${REQS}
)

message(STATUS "pstar_managers: All managers enabled")

