# components/pstar_storage_hal/CMakeLists.txt

set(hal_srcs
    "pstar_storage_hal.c"
    "pstar_storage_common.c"
    "pstar_storage_sd_card_hal.c"
)

set(hal_includes
    "include"
    # Removed backward compatibility include directory
)

# Conditionally add SPI implementation if enabled
if(CONFIG_PSTAR_KCONFIG_SD_CARD_SPI_MODE_ENABLED)
  list(APPEND hal_srcs "pstar_storage_spi_hal.c")
endif()

# Keep SDIO Kconfig options for future but don't include the source file
# if(CONFIG_PSTAR_KCONFIG_SD_CARD_SDIO_MODE_ENABLED)
#   list(APPEND hal_srcs "pstar_storage_sdio_hal.c")
# endif()

# Base requirements for the storage HAL
set(storage_hal_requires
  driver
  esp_common
  freertos
  nvs_flash # Needed for saving working config
  fatfs
  sdmmc # Keep sdmmc dependency for FATFS
  pstar_error_handler
  pstar_logger
  pstar_bus
)

# Register the component
idf_component_register(
  SRCS ${hal_srcs}
  INCLUDE_DIRS ${hal_includes}
  REQUIRES ${storage_hal_requires}
)

# Diagnostic prints
message(STATUS "pstar_storage_hal - Final SRCS: ${hal_srcs}")
message(STATUS "pstar_storage_hal - Final REQUIRES: ${storage_hal_requires}")
