idf_component_register(
  SRCS
    "pca9685_hal/pca9685_hal.c"
  INCLUDE_DIRS
    "include"
    "pca9685_hal/include"
  PRIV_REQUIRES
    driver
    common
)

