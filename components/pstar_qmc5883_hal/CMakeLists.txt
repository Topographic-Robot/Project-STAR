# components/pstar_qmc5883_hal/CMakeLists.txt

idf_component_register(
  SRCS
    "pstar_qmc5883_hal.c"
  INCLUDE_DIRS
    "include"
  REQUIRES
    pstar_bus
    pstar_pin_validator
    freertos
    driver
    esp_common
)