# components/pstar_pin_validator/CMakeLists.txt

idf_component_register(
	SRCS
		"pstar_pin_validator.c"
	INCLUDE_DIRS
		"include"
	PRIV_INCLUDE_DIRS
	REQUIRES
		driver
	  pstar_pin_validator
	PRIV_REQUIRES
)
