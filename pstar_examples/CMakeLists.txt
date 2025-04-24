# pstar_examples/CMakeLists.txt

idf_component_register(
  SRCS
    "pstar_examples.c"
    "bh1750_example.c"
    "pca9685_example.c"
#    "dht22_example.c"
#    "mq135_example.c"
#    "hd44780_example.c"
    "pin_validator_example.c"
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
    # Project Components
    pstar_pin_validator
    pstar_bus
    pstar_bh1750_hal
    pstar_pca9685_hal
    pstar_dht22_hal
    pstar_mq135_hal
    pstar_hd44780_hal
  PRIV_REQUIRES
)
