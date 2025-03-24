# Only register the pin validator component if enabled via Kconfig.
if(NOT CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED)
  message(STATUS "pstar_pin_validator: Component disabled via Kconfig")
  return()
endif()

idf_component_register(
  SRCS "pin_validator.c"
  INCLUDE_DIRS "include"
  REQUIRES
    driver
    freertos
)

message(STATUS "pstar_pin_validator: Component enabled")

