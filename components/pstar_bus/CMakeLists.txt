# Initialize lists for conditional sources and includes
set(bus_srcs 
    "pstar_bus_types.c"
    "pstar_bus_manager.c"
    "pstar_bus_config.c"
    "pstar_bus_i2c.c"
    "pstar_bus_spi.c"
    "pstar_bus_uart.c"
    "pstar_bus_gpio.c")

# Conditionally add SDIO bus source if SD card is enabled
if(CONFIG_PSTAR_KCONFIG_SD_CARD_ENABLED)
    list(APPEND bus_srcs "pstar_bus_sdio.c")
endif()

idf_component_register(
  SRCS ${bus_srcs}
  INCLUDE_DIRS
    "include"
  REQUIRES
    driver
    esp_common
    freertos
    esp_timer
    pstar_pin_validator
    sdmmc
    nvs_flash
    pstar_storage_hal
)