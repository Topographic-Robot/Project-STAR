# Only register the error handler component if enabled via Kconfig.
if(NOT CONFIG_PSTAR_KCONFIG_ERROR_HANDLER_ENABLED)
  message(STATUS "pstar_error_handler: Component disabled via Kconfig")
  return()
endif()

idf_component_register(
  SRCS "error_handler.c"
  INCLUDE_DIRS "include"
  REQUIRES
    driver
    freertos
)

message(STATUS "pstar_error_handler: Component enabled")

