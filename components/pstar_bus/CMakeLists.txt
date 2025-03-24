# Only register this component if the bus component is enabled via Kconfig.
if(NOT CONFIG_PSTAR_KCONFIG_BUS_COMPONENT_ENABLED)
  message(STATUS "pstar_bus: Component disabled via Kconfig")
  return()
endif()

# Define source files list based on Kconfig.
set(SRCS
  "bus_types.c"
  "bus_manager.c"
  "bus_config.c"
)

# Conditionally append bus implementations based on configuration.
if(CONFIG_PSTAR_KCONFIG_BUS_I2C_ENABLED)
  list(APPEND SRCS "bus_i2c.c")
endif()

if(CONFIG_PSTAR_KCONFIG_BUS_SPI_ENABLED)
  list(APPEND SRCS "bus_spi.c")
endif()

if(CONFIG_PSTAR_KCONFIG_BUS_UART_ENABLED)
  list(APPEND SRCS "bus_uart.c")
endif()

if(CONFIG_PSTAR_KCONFIG_BUS_GPIO_ENABLED)
  list(APPEND SRCS "bus_gpio.c")
endif()

if(CONFIG_PSTAR_KCONFIG_BUS_SDIO_ENABLED)
  list(APPEND SRCS "bus_sdio.c")
endif()

idf_component_register(
  SRCS ${SRCS}
  INCLUDE_DIRS "include"
  REQUIRES
    driver
    esp_common
    freertos
    pstar_storage_hal
    esp_timer
)

# Build logging message for enabled bus types.
set(ENABLED_BUS_TYPES "")
if(CONFIG_PSTAR_KCONFIG_BUS_I2C_ENABLED)
  set(ENABLED_BUS_TYPES "${ENABLED_BUS_TYPES} I2C")
endif()
if(CONFIG_PSTAR_KCONFIG_BUS_SPI_ENABLED)
  set(ENABLED_BUS_TYPES "${ENABLED_BUS_TYPES} SPI")
endif()
if(CONFIG_PSTAR_KCONFIG_BUS_UART_ENABLED)
  set(ENABLED_BUS_TYPES "${ENABLED_BUS_TYPES} UART")
endif()
if(CONFIG_PSTAR_KCONFIG_BUS_GPIO_ENABLED)
  set(ENABLED_BUS_TYPES "${ENABLED_BUS_TYPES} GPIO")
endif()
if(CONFIG_PSTAR_KCONFIG_BUS_SDIO_ENABLED)
  set(ENABLED_BUS_TYPES "${ENABLED_BUS_TYPES} SDIO")
endif()

message(STATUS "pstar_bus: Component enabled with bus types:${ENABLED_BUS_TYPES}")

