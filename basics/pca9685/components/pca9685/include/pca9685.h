#ifndef __PCA9685_H_
#define __PCA9685_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**********************/

/* The base address for board 0, for board1 its 0x41 and board2 is 0x42 */
#define PCA9685_ADDR_BASE 0x40

#define PCA9685_MAX_FREQ 1600 /* This is in Hz */
#define PCA9685_MAX_PWM  4096 /* This is in ticks */

/* An enum for every channel on each board */
typedef enum {
  PCA9685_CHANNEL_0 = 0,
  PCA9685_CHANNEL_1,
  PCA9685_CHANNEL_2,
  PCA9685_CHANNEL_3,
  PCA9685_CHANNEL_4,
  PCA9685_CHANNEL_5,
  PCA9685_CHANNEL_6,
  PCA9685_CHANNEL_7,
  PCA9685_CHANNEL_8,
  PCA9685_CHANNEL_9,
  PCA9685_CHANNEL_10,
  PCA9685_CHANNEL_11,
  PCA9685_CHANNEL_12,
  PCA9685_CHANNEL_13,
  PCA9685_CHANNEL_14,
  PCA9685_CHANNEL_15,
  PCA9685_CHANNEL_ALL,
} pca9685_chanel_e;

typedef struct {
} i2c_pca9685_bus_config_t;


/**********************/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PCA9685_H_ */
