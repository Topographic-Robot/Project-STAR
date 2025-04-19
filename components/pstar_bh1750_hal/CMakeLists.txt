# components/pstar_bh1750_hal/CMakeLists.txt

idf_component_register(
	SRCS
		"pstar_bh1750_hal.c"
	INCLUDE_DIRS
		"include"
  REQUIRES
  	pstar_bus
  	pstar_pin_validator
    freertos
)
