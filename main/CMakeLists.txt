# main/CMakeLists.txt

idf_component_register(
  SRCS
    "main.c"
  REQUIRES
    pstar_pin_validator
    pstar_logger
    pstar_error_handler
    pstar_managers
    pstar_storage_hal
)
