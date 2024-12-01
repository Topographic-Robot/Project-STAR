idf_component_register(
  SRCS
    "i2c.c"
    "uart.c"
    "spi.c"
  INCLUDE_DIRS
    "include"
  PRIV_REQUIRES
    driver
)

