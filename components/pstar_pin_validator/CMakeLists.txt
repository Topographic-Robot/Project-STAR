idf_component_register(
  SRCS 
    "pin_validator.c"
  INCLUDE_DIRS 
    "include"
  REQUIRES 
    driver
    freertos
)

message(STATUS "pstar_pin_validator: Component enabled")