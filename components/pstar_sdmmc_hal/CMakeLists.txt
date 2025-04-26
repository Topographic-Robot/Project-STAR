# components/pstar_sdmmc_hal/CMakeLists.txt

idf_component_register(
  SRCS
    "pstar_sdmmc_hal.c"
  INCLUDE_DIRS
    "include"
  REQUIRES
    pstar_bus
    pstar_pin_validator
    driver
    esp_common
    freertos
    sdmmc
    fatfs
)