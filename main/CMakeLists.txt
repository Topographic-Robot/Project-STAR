# main/CMakeLists.txt

idf_component_register(
	SRCS
		main.c
	INCLUDE_DIRS
	PRIV_INCLUDE_DIRS
	REQUIRES
		pstar_pin_validator
	  pstar_jtag
	  pstar_bus
	  pstar_bh1750_hal
	  pstar_pca9685_hal
	  pstar_examples
	PRIV_REQUIRES
)
