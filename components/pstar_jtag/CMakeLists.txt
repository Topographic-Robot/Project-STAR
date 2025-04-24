# components/pstar_jtag/CMakeLists.txt

idf_component_register(
	SRCS
		pstar_jtag.c
  INCLUDE_DIRS
  	include
  REQUIRES
		pstar_pin_validator
)
