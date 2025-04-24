# components/pstar_dht22_hal/CMakeLists.txt

idf_component_register(
    SRCS
        "pstar_dht22_hal.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        pstar_pin_validator
        driver
        freertos
        esp_timer
        log
)