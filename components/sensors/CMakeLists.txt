idf_component_register(
  SRCS 
    "bh1750_hal/bh1750_hal.c"
    "dht22_hal/dht22_hal.c"
    "mpu6050_hal/mpu6050_hal.c"
  INCLUDE_DIRS 
    "include"
    "bh1750_hal/include"
    "dht22_hal/include"
    "mpu6050_hal/include"
  PRIV_REQUIRES 
    driver 
    common
)

