# pstar_examples/CMakeLists.txt

idf_component_register(
  SRCS
    "pstar_examples.c"
    "bh1750_example.c"
    "pca9685_example.c"
    "dht22_example.c"
    "mq135_example.c"
    "hd44780_example.c"
    "pin_validator_example.c"
    "error_handler_example.c"
    "jtag_example.c"
    "mpu6050_example.c"
    "qmc5883_example.c"
    "ili9341_example.c"
    "sdmmc_example.c" # Added
  INCLUDE_DIRS
    "."
    "include"
  PRIV_INCLUDE_DIRS
  REQUIRES
    # --- List ALL potential dependencies for ANY example ---
    # Core IDF
    driver
    freertos
    esp_timer
    esp_adc
    esp_common
    sdmmc
    # Project Components
    pstar_pin_validator
    pstar_bus
    pstar_bh1750_hal
    pstar_pca9685_hal
    pstar_dht22_hal
    pstar_mq135_hal
    pstar_hd44780_hal
    pstar_error_handler
    pstar_jtag
    pstar_mpu6050_hal
    pstar_qmc5883_hal
    pstar_ili9341_hal
    pstar_sdmmc_hal
  PRIV_REQUIRES
)