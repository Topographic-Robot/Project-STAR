idf_component_register(
  SRCS 
    "main.c"
  INCLUDE_DIRS 
    "."
    "include"
  REQUIRES 
    driver
    freertos
    esp_common
    nvs_flash
    pstar_error_handler
    pstar_logger
)

message(STATUS "Project-Star: Building main component")