# components/pstar_logger/CMakeLists.txt

# Initialize lists
set(logger_srcs
  "pstar_log_handler.c"
  "pstar_log_storage.c"
)
# Base requirements
set(logger_requires
    driver
    esp_common
    freertos
    nvs_flash
    esp_timer
    # Add storage_hal conditionally
)
# Base private requires
set(logger_priv_requires
    pstar_managers
    espressif__zlib
    # Add storage_hal conditionally
)

# Conditionally require storage_hal based on Kconfig
if(CONFIG_PSTAR_KCONFIG_LOGGING_SD_CARD_ENABLED)
  list(APPEND logger_requires pstar_storage_hal)
  list(APPEND logger_priv_requires pstar_storage_hal)
endif()

# Ensure no duplicates
list(REMOVE_DUPLICATES logger_requires)
list(REMOVE_DUPLICATES logger_priv_requires)

idf_component_register(
  SRCS ${logger_srcs}
  INCLUDE_DIRS "include"
  REQUIRES ${logger_requires}
  PRIV_REQUIRES ${logger_priv_requires}
)

# Diagnostic print statements
message(STATUS "pstar_logger - Final REQUIRES: ${logger_requires}")
message(STATUS "pstar_logger - Final PRIV_REQUIRES: ${logger_priv_requires}")