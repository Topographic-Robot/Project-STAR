# components/pstar_bus/CMakeLists.txt

idf_component_register(
  SRCS
		"pstar_bus_types.c"
		"pstar_bus_manager.c"
		"pstar_bus_config.c"
		"pstar_bus_i2c.c"
		"pstar_bus_spi.c"
  INCLUDE_DIRS
    "include"
  REQUIRES
    driver
    esp_common
    freertos
)