# components/pstar_managers/CMakeLists.txt

set(manager_srcs "")
set(manager_includes "include")
# Base requires - Add storage_hal unconditionally
set(manager_requires
    freertos
    driver
    esp_common
    esp_timer
    pstar_logger
    pstar_storage_hal # <-- Keep unconditional for now
    esp_netif
)
# Base private requires - Add storage_hal unconditionally
set(manager_priv_requires
    pstar_storage_hal # <-- Keep unconditional for now
)

# Conditionally add Time Manager sources AND its specific dependencies
if(CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED)
  list(APPEND manager_srcs "pstar_time_manager.c")
  # Add network components to REQUIRES only when time manager is enabled
  list(APPEND manager_requires esp_netif esp_event esp_sntp lwip) # <-- Added lwip requirement
endif()

# Conditionally add File Write Manager sources
if(CONFIG_PSTAR_KCONFIG_FILE_MANAGER_ENABLED)
    # Check if SD card is also enabled, as the file manager needs it
    if(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED)
      list(APPEND manager_srcs "pstar_file_write_manager.c")
      # The requirement for pstar_storage_hal is handled unconditionally above
    endif()
endif()

# Ensure no duplicates
list(REMOVE_DUPLICATES manager_requires)
list(REMOVE_DUPLICATES manager_priv_requires)

idf_component_register(
  SRCS ${manager_srcs}
  INCLUDE_DIRS ${manager_includes}
  REQUIRES ${manager_requires}
  PRIV_REQUIRES ${manager_priv_requires}
)

# Diagnostic prints
message(STATUS "pstar_managers - Final SRCS: ${manager_srcs}")
message(STATUS "pstar_managers - Final REQUIRES: ${manager_requires}")
message(STATUS "pstar_managers - Final PRIV_REQUIRES: ${manager_priv_requires}")
