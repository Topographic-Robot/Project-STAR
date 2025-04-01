# components/pstar_error_handler/CMakeLists.txt

idf_component_register(
  SRCS "pstar_error_handler.c"
  INCLUDE_DIRS "include"
  REQUIRES
    driver
    freertos
    pstar_logger
)
