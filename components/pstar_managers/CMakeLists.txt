# Initialize lists
set(manager_srcs "")
set(manager_includes "include")
# Base requires: Add esp_event for network events if Time Manager uses wifi
set(manager_requires freertos driver esp_common esp_timer pstar_logger)
set(manager_priv_requires "")

# Conditionally add Time Manager
if(CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED)
  list(APPEND manager_srcs "pstar_time_manager.c")
  # Add specific requires for Time Manager
  # esp_sntp depends on lwip which depends on esp_netif and esp_event
  list(APPEND manager_requires esp_netif esp_event esp_sntp)
endif()

# Conditionally add File Write Manager
if(CONFIG_PSTAR_KCONFIG_FILE_MANAGER_ENABLED)
  list(APPEND manager_srcs "pstar_file_write_manager.c")
  # File manager requires storage_hal because its Kconfig depends on SD card
  # storage_hal already requires pstar_logger, pstar_bus etc.
  list(APPEND manager_requires pstar_storage_hal)
endif()

idf_component_register(
  SRCS ${manager_srcs}
  INCLUDE_DIRS ${manager_includes}
  REQUIRES ${manager_requires}
  PRIV_REQUIRES ${manager_priv_requires}
)