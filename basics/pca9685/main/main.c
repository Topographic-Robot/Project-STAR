#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define I2C_MASTER_SCL_IO         GPIO_NUM_22
#define I2C_MASTER_SDA_IO         GPIO_NUM_21
#define I2C_MASTER_NUM            I2C_NUM_0
#define I2C_MASTER_FREQ_HZ        100000 /* 1MHz */
#define I2C_MASTER_TX_BUF_DISABLE 0      /* I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0      /* I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS     1000

/* These values are from the datasheet */
#define PCA9685_ADDR         0x40     /* Slave address for PCA9685 */
#define PCA9685_INTER_OSC    25000000 /* Internal Oscillator (25MHz) */
#define PCA9685_RESOLUTION   4096     /* PCA9685 Resolution (2^12) */
#define PCA9685_SLEEP_MODE   0x10
#define PCA9685_RESTART_MODE 0x80
#define PCA9685_OUTPUT_MODE  0x04 /* Output Logic Mode */
#define PCA9685_MODE1        0x00
#define PCA9685_MODE2        0x01
#define PCA9685_PRESCALE     0xFE
#define PCA9685_LED0_ON_L    0x06
#define PCA9685_LED0_ON_H    0x07
#define PCA9685_LED0_OFF_L   0x08
#define PCA9685_LED0_OFF_H   0x09

static const char *TAG = "pca9685_servo";

/* Structure to keep track of servo positions */
typedef struct {
  uint8_t channel;
  float   current_degree;
} servo_t;

/* Array to keep track of all servos */
static servo_t servos[16]; /* Assuming we have up to 16 servos connected */

/* Initialize I2C */
static esp_err_t i2c_master_init(void) {
  i2c_config_t conf = {
      .mode             = I2C_MODE_MASTER,
      .sda_io_num       = I2C_MASTER_SDA_IO,
      .sda_pullup_en    = GPIO_PULLUP_ENABLE,
      .scl_io_num       = I2C_MASTER_SCL_IO,
      .scl_pullup_en    = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C param config failed");
    return err;
  }
  return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

/* Write to PCA9685 register */
static esp_err_t pca9685_write(uint8_t reg_addr, uint8_t data) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_write_byte(cmd, data, true);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  return err;
}

/* Set PWM frequency */
static esp_err_t pca9685_set_pwm_freq(float freq) {
  uint8_t prescale_val =
      (uint8_t)(PCA9685_INTER_OSC / (PCA9685_RESOLUTION * freq) - 1);
  /* Enter sleep mode */
  ESP_ERROR_CHECK(pca9685_write(PCA9685_MODE1, PCA9685_SLEEP_MODE));
  ESP_ERROR_CHECK(pca9685_write(PCA9685_PRESCALE, prescale_val));
  /* Restart */
  ESP_ERROR_CHECK(pca9685_write(PCA9685_MODE1, PCA9685_RESTART_MODE));
  /* Output logic */
  ESP_ERROR_CHECK(pca9685_write(PCA9685_MODE2, PCA9685_OUTPUT_MODE));
  return ESP_OK;
}

/* Set PWM value for a specific channel */
static esp_err_t pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
  /* 4 * channel is to calculate the channel offset, 4 is because of 4 registers
   * (shown below) */

  /* Write the low byte of the ON time to the register for the specified channel
   */
  ESP_ERROR_CHECK(pca9685_write(PCA9685_LED0_ON_L + 4 * channel, on & 0xFF));

  /* Write the high byte of the ON time to the register for the specified
   * channel */
  ESP_ERROR_CHECK(pca9685_write(PCA9685_LED0_ON_H + 4 * channel, on >> 8));

  /* Write the low byte of the OFF time to the register for the specified
   * channel */
  ESP_ERROR_CHECK(pca9685_write(PCA9685_LED0_OFF_L + 4 * channel, off & 0xFF));

  /* Write the high byte of the OFF time to the register for the specified
   * channel */
  ESP_ERROR_CHECK(pca9685_write(PCA9685_LED0_OFF_H + 4 * channel, off >> 8));

  return ESP_OK;
}

/* Move servo to a specified degree without any speed control */
static esp_err_t pca9685_set_servo_angle(uint8_t channel, float degree) {
  /* Ensure the degree is within the valid range */
  if (degree < 0) {
    degree = 0;
  }
  if (degree > 180) {
    degree = 180;
  }

  /* Calculate the corresponding off value for the given degree
   *
   * Explanation of the magic numbers:
   * - The PCA9685 has a resolution of 4096 steps (0-4095).
   * - The PWM frequency is set to 50Hz, corresponding to a 20ms period.
   * - For a standard hobby servo, the pulse width varies from ~1ms (0 degrees)
   *   to ~2ms (180 degrees).
   * - Each step represents approximately 0.00488ms (20ms / 4096).
   * - For 0 degrees: 1ms / 0.00488ms ≈ 204 steps.
   * - For 180 degrees: 2ms / 0.00488ms ≈ 409 steps.
   * - The range from 0 to 180 degrees thus spans 204 to 409 steps (409 - 204 =
   *   205 steps).
   *
   * The formula `(degree / 180.0 * 205.0) + 204.0` converts the servo angle
   * (0-180 degrees) to the corresponding PWM step value (204-409 steps) */
  uint16_t target_off = (uint16_t)((degree / 180.0 * 205.0) + 204.0);

  /* Set the PWM value for the servo */
  esp_err_t err = pca9685_set_pwm(channel, 0, target_off);
  if (err != ESP_OK) {
    return err;
  }

  /* Update the current degree in the servo structure */
  servos[channel].current_degree = degree;

  return ESP_OK;
}

/* Gradually move servo to a specified degree with speed control
 *
 * NOTE: you must set the current_degree for the motor before running this
 * function */
static esp_err_t pca9685_set_servo_angle_smooth(uint8_t  channel,
                                                float    target_degree,
                                                uint16_t step_delay_ms) {
  /* Ensure the degree is within the valid range */
  if (target_degree < 0) {
    target_degree = 0;
  }
  if (target_degree > 180) {
    target_degree = 180;
  }

  /* Get the current degree from the servo structure */
  float current_degree = servos[channel].current_degree;
  float step_size      = 1.0; /* Define the step size for each movement */

  /* Calculate the direction of movement */
  int direction = (target_degree > current_degree) ? 1 : -1;

  /* Gradually move the servo */
  while ((direction == 1 && current_degree < target_degree) ||
         (direction == -1 && current_degree > target_degree)) {
    current_degree += direction * step_size;
    /* Ensure the degree is within the valid range */
    if (current_degree < 0) {
      current_degree = 0;
    }
    if (current_degree > 180) {
      current_degree = 180;
    }

    /* Calculate the corresponding off value for the given degree */
    uint16_t target_off = (uint16_t)((current_degree / 180.0 * 205.0) + 204.0);

    /* Set the PWM value for the servo */
    ESP_ERROR_CHECK(pca9685_set_pwm(channel, 0, target_off));

    /* Update the current degree in the servo structure */
    servos[channel].current_degree = current_degree;

    /* Delay to control the speed of movement */
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
  }
  return ESP_OK;
}

/* Task to move a motor */
void motor_task(void *arg) {
  uint8_t channel = *(uint8_t *)arg;
  while (1) {
    /* Move servo to 0 degrees smoothly */
    ESP_LOGI(TAG, "Moving servo %d to 0 degrees smoothly", channel);
    ESP_ERROR_CHECK(pca9685_set_servo_angle_smooth(channel, 0, 10));
    vTaskDelay(pdMS_TO_TICKS(1000)); /* Wait for a second */

    /* Move servo to 180 degrees smoothly */
    ESP_LOGI(TAG, "Moving servo %d to 180 degrees smoothly", channel);
    ESP_ERROR_CHECK(pca9685_set_servo_angle_smooth(channel, 180, 10));
    vTaskDelay(pdMS_TO_TICKS(1000)); /* Wait for a second */
  }
}

void app_main(void) {
  ESP_ERROR_CHECK(i2c_master_init());
  /* Set frequency to 50Hz for servos */
  ESP_ERROR_CHECK(pca9685_set_pwm_freq(50));

  /* Initialize servos (manual initialization) */
  for (uint8_t i = 0; i < 16; i++) {
    servos[i].channel        = i;
    /* Assuming starting at 90 degrees for safety */
    servos[i].current_degree = 90;
    pca9685_set_servo_angle(i, 90);
  }

  /* Create tasks for all 16 motors */
  for (uint8_t i = 0; i < 16; i++) {
    xTaskCreate(motor_task, "motor_task", 2048, &servos[i].channel, 5, NULL);
  }

  while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}
