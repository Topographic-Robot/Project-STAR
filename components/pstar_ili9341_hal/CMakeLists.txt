# components/pstar_ili9341_hal/CMakeLists.txt

idf_component_register(
	SRCS
		"pstar_ili9341_hal.c"
	INCLUDE_DIRS
		"include"
	REQUIRES
		pstar_bus             # Requires the bus manager (now with SPI)
		pstar_pin_validator   # For pin validation
		driver                # For SPI driver, GPIO
		esp_common            # For ESP_LOG, ESP_ERROR_CHECK
		freertos              # For mutex, task delay
)