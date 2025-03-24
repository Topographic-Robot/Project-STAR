# Only register this component if logging is enabled via Kconfig.
if(NOT CONFIG_PSTAR_KCONFIG_LOGGING_ENABLED)
  message(STATUS "pstar_logger: Component disabled via Kconfig")
  return()
endif()

idf_component_register(
  SRCS
    "log_handler.c"
    "log_storage.c"
  INCLUDE_DIRS "include"
  REQUIRES
    driver
    esp_common
    freertos
    pstar_storage_hal
    nvs_flash
    esp_timer
    pstar_managers
  PRIV_REQUIRES
    zlib
)

message(STATUS "pstar_logger: Component enabled with all features")

