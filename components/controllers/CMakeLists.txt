idf_component_register(
  SRCS
    "pca9685_hal/pca9685_hal.c"
    "ec11_hal/ec11_hal.c"
    "common_motor_setup.c"
  INCLUDE_DIRS
    "include"
    "pca9685_hal/include"
    "ec11_hal/include"
  PRIV_REQUIRES
    main
    driver
    common
)

