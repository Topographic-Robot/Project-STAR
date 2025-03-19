idf_component_register(
  SRCS
    "error_handler.c"
  INCLUDE_DIRS
    "include"
  REQUIRES
    driver
    freertos
)

message(STATUS "pstar_error_handler: Component enabled")