# components/pstar_storage_hal/CMakeLists.txt

set(hal_srcs "")
set(hal_includes "")

# Conditionally add SD card sources and includes
if(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED)
  list(APPEND hal_srcs "sd_card_hal/pstar_sd_card_hal.c")
  list(APPEND hal_includes "sd_card_hal/include") # Keep conditional include dir
endif()

# Base requirements for the storage HAL
set(storage_hal_requires
  driver
  esp_common
  freertos
  nvs_flash # Needed for saving working config
  fatfs
  sdmmc # Keep sdmmc dependency here
  pstar_error_handler
  pstar_logger
  pstar_bus
)

# Add pstar_bus dependency since we use pstar_bus_manager_fwd.h header
# This ensures proper include path resolution without circular dependencies
list(APPEND storage_hal_requires pstar_bus)

# Register the component
idf_component_register(
  SRCS ${hal_srcs}
  INCLUDE_DIRS ${hal_includes} # Use potentially conditional list
  REQUIRES ${storage_hal_requires}
)

# Diagnostic prints
message(STATUS "pstar_storage_hal - Final SRCS: ${hal_srcs}")
message(STATUS "pstar_storage_hal - Final REQUIRES: ${storage_hal_requires}")
