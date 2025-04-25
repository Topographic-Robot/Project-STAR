# components/pstar_hd44780_hal/CMakeLists.txt

idf_component_register(
    SRCS
        "pstar_hd44780_hal.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        pstar_pin_validator
        pstar_bus
        driver
        freertos
)