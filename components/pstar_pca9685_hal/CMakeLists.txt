# components/pstar_pca9685_hal/CMakeLists.txt

idf_component_register(
	SRCS
		"pstar_pca9685_hal.c"
	INCLUDE_DIRS
		"include"
	REQUIRES
		pstar_bus
		pstar_pin_validator
		freertos
		driver
		esp_common
		log
)

