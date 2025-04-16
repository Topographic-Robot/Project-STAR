# main/CMakeLists.txt

idf_component_register(
	SRCS main.c
	INCLUDE_DIRS
	PRIV_INCLUDE_DIRS
	REQUIRES           pstar_pin_validator
	                   pstar_jtag
	PRIV_REQUIRES
)
