idf_component_register(
  SRCS 
    "i2c.c"
    "gpio.c"
    "uart.c"
  INCLUDE_DIRS 
    "include"
  PRIV_REQUIRES 
    driver
)

