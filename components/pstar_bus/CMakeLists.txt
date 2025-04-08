# components/pstar_bus/CMakeLists.txt

set(bus_srcs
    "pstar_bus_types.c"
    "pstar_bus_manager.c"
    "pstar_bus_config.c"
    "pstar_bus_i2c.c"
    "pstar_bus_spi.c"
    "pstar_bus_uart.c"
    "pstar_bus_gpio.c")

# Conditionally add SDIO bus source based on BUS Kconfig
if(CONFIG_PSTAR_KCONFIG_BUS_SDIO_ENABLED)
  list(APPEND bus_srcs "pstar_bus_sdio.c")
endif()

idf_component_register(
  SRCS ${bus_srcs}
  INCLUDE_DIRS
    "include"
  REQUIRES # <-- Removed pstar_storage_hal from here
    driver
    esp_common
    freertos
    esp_timer
    pstar_pin_validator
    pstar_logger
    sdmmc # Needed for sdmmc types used in pstar_bus_protocol_types.h
)
