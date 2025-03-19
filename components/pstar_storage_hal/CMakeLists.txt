idf_component_register(
  SRCS
    "sd_card_hal/sd_card_hal.c"
  INCLUDE_DIRS
    "include"
    "sd_card_hal/include"
  REQUIRES
    driver
    esp_common
    freertos
    nvs_flash
    fatfs
    sdmmc
    wear_levelling
    esp_partition
    vfs
    pstar_error_handler
    pstar_bus
)

message(STATUS "pstar_storage_hal: SD card HAL enabled")