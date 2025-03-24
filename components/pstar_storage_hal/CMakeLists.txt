# Only register the storage HAL component if enabled via Kconfig.
if(NOT CONFIG_PSTAR_KCONFIG_STORAGE_HAL_ENABLED)
  message(STATUS "pstar_storage_hal: Component disabled via Kconfig")
  return()
endif()

# Define source files and include directories based on Kconfig settings.
set(SRCS "")
set(INCLUDE_DIRS "include")

# Conditionally include SD card HAL implementation if enabled.
if(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED)
  list(APPEND SRCS "sd_card_hal/sd_card_hal.c")
  list(APPEND INCLUDE_DIRS "sd_card_hal/include")
endif()

# Define core required components.
# Note: 'fatfs' must always be included because sd_card_hal.c includes esp_vfs_fat.h from fatfs.
set(REQS
  driver
  esp_common
  freertos
  nvs_flash
  esp_partition
  vfs
  pstar_error_handler
  pstar_bus
  fatfs
)

# Append SD-specific components if SD card is enabled.
if(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED)
  list(APPEND REQS sdmmc wear_levelling)
endif()

idf_component_register(
  SRCS ${SRCS}
  INCLUDE_DIRS ${INCLUDE_DIRS}
  REQUIRES ${REQS}
)

# Log SD card HAL interface mode.
if(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED)
  if(CONFIG_PSTAR_KCONFIG_SD_CARD_4BIT_MODE)
    set(INTERFACE_TYPE " (4-bit mode)")
  else()
    set(INTERFACE_TYPE " (1-bit mode)")
  endif()
  message(STATUS "pstar_storage_hal: SD card HAL enabled${INTERFACE_TYPE}")
else()
  message(STATUS "pstar_storage_hal: SD card HAL disabled")
endif()

