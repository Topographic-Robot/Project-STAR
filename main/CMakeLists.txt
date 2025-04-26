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
    pstar_dht22_hal
    pstar_mq135_hal
    pstar_hd44780_hal
    pstar_error_handler
    pstar_mpu6050_hal
    pstar_qmc5883_hal
    pstar_ili9341_hal
    pstar_sdmmc_hal   # Added now
    pstar_examples
	PRIV_REQUIRES
)