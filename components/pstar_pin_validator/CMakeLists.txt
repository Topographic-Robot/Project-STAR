# components/pstar_pin_validator/CMakeLists.txt

idf_component_register(
  SRCS "pstar_pin_validator.c"
  INCLUDE_DIRS "include"
  REQUIRES
    driver
    freertos
    pstar_logger
)
