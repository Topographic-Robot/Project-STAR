# Initialize lists for conditional sources and includes
set(hal_srcs "")
set(hal_includes "")

# Conditionally add SD card sources and includes
if(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED)
  list(APPEND hal_srcs "sd_card_hal/pstar_sd_card_hal.c")
  list(APPEND hal_includes "sd_card_hal/include")
endif()

# Register the component with potentially empty lists if SD card is disabled
idf_component_register(
  SRCS ${hal_srcs}
  INCLUDE_DIRS ${hal_includes}
  REQUIRES
    driver
    esp_common
    freertos
    nvs_flash
    fatfs
    pstar_error_handler
    pstar_bus
    pstar_logger
)
