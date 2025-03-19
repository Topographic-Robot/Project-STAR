# Register the component with ESP-IDF
idf_component_register(
  SRCS
    "bus_types.c"
    "bus_manager.c"
    "bus_config.c"
    "bus_i2c.c"
    "bus_spi.c"
    "bus_gpio.c"
    "bus_sdio.c"
    "bus_uart.c"
  INCLUDE_DIRS
    "include"
  REQUIRES
    driver
    esp_common
    freertos
    pstar_storage_hal
    esp_timer
)

message(STATUS "pstar_bus: Component enabled with all bus types")