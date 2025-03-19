# Initialize lists
set(manager_srcs "")
set(manager_includes "include")
set(manager_requires freertos driver esp_common esp_timer esp_netif pstar_logger) # Base requires
set(manager_priv_requires "")

# Conditionally add Time Manager
if(CONFIG_PSTAR_KCONFIG_TIME_MANAGER_ENABLED)
  list(APPEND manager_srcs "pstar_time_manager.c")
  # Add specific requires for Time Manager if any (e.g., esp_sntp if not covered by esp_netif)
endif()

# Conditionally add File Write Manager
if(CONFIG_PSTAR_KCONFIG_FILE_MANAGER_ENABLED)
  list(APPEND manager_srcs "pstar_file_write_manager.c")
  # File manager requires storage_hal because its Kconfig depends on SD card
  list(APPEND manager_requires pstar_storage_hal)
endif()

idf_component_register(
  SRCS ${manager_srcs}
  INCLUDE_DIRS ${manager_includes}
  REQUIRES ${manager_requires}
  PRIV_REQUIRES ${manager_priv_requires}
)
