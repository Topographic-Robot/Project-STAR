# components/pstar_mq135_hal/CMakeLists.txt

idf_component_register(
    SRCS
        "pstar_mq135_hal.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        pstar_pin_validator
        esp_adc
        freertos
)