# Create a list for main component dependencies
set(MAIN_REQUIRES
    driver
    freertos
    esp_common
    nvs_flash
)

# Conditionally require pstar_error_handler if enabled via Kconfig.
if(CONFIG_PSTAR_KCONFIG_ERROR_HANDLER_ENABLED)
  list(APPEND MAIN_REQUIRES pstar_error_handler)
  message(STATUS "main: Including pstar_error_handler in component dependencies")
else()
  message(STATUS "main: Skipping pstar_error_handler as it is disabled via Kconfig")
endif()

# Conditionally require pstar_logger if enabled via Kconfig.
if(CONFIG_PSTAR_KCONFIG_LOGGING_ENABLED)
  list(APPEND MAIN_REQUIRES pstar_logger)
  message(STATUS "main: Including pstar_logger in component dependencies")
else()
  message(STATUS "main: Skipping pstar_logger as it is disabled via Kconfig")
endif()

idf_component_register(
  SRCS "main.c"
  INCLUDE_DIRS
    "."
    "include"
  REQUIRES ${MAIN_REQUIRES}
)

message(STATUS "Project-Star: Building main component")

