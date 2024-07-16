#include "low_level_approach.h"
#include <stdint.h>
#include <stdio.h>

/* Define the base address for GPIO registers */
#define DR_REG_GPIO_BASE 0x3FF44000

/* Write 1 to Set (W1TS): Sets the pin direction to output */
#define GPIO_ENABLE_W1TS_REG (DR_REG_GPIO_BASE + 0x0020)

/* Write 1 to Clear (W1TC): Clears the pin direction to input */
#define GPIO_ENABLE_W1TC_REG (DR_REG_GPIO_BASE + 0x0024)

/* Write 1 to Set (W1TS): Sets the pin output high */
#define GPIO_OUT_W1TS_REG (DR_REG_GPIO_BASE + 0x0008)

/* Write 1 to Clear (W1TC): Sets the pin output low */
#define GPIO_OUT_W1TC_REG (DR_REG_GPIO_BASE + 0x000C)

/* Define the GPIO pin number for the LED */
#define LED_PIN 2

/* Simple delay function (busy-wait loop) */
void delay(int millis) {
  /* Adjust this value based on the CPU frequency (240 MHz for ESP32) */
  int count = millis * 24000;
  while (count--) {
    asm volatile("nop"); /* No operation instruction to prevent the compiler */
                         /* from optimizing this loop away */
  }
}

void low_level_approach(void) {
  /* Set LED_PIN as output */
  *(volatile uint32_t *)(GPIO_ENABLE_W1TS_REG) = (1 << LED_PIN);

  while (1) {
    /* Turn LED on */
    *(volatile uint32_t *)(GPIO_OUT_W1TS_REG) = (1 << LED_PIN);
    delay(1000); /* Delay for 1 second */

    /* Turn LED off */
    *(volatile uint32_t *)(GPIO_OUT_W1TC_REG) = (1 << LED_PIN);
    delay(1000); /* Delay for 1 second */
  }
}
