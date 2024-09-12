#include "system_tasks.h"
#include <esp_log.h>
#include <nvs_flash.h>

static const char *toporobo_tag = "Topographic-Robot";

static void clear_nvs_flash(void);
static void fpga_comm_init(void);
static void motors_comm_init(void);

void app_main(void) 
{
  /* Initialize the system, clear the nvs flash */
  clear_nvs_flash();

  /* Establish communication with FPGA */
  fpga_comm_init();

  /* Establish communication with Motors */
  motors_comm_init();

  /* Establish communication with Sensors */
  sensors_comm_init();

  /* Start System-Level Tasks (motor, sensors, webserver, etc) */
  start_system_tasks();
}

/**
 * @brief Clears and initializes the ESP32's Non-Volatile Storage (NVS) flash.
 * 
 * This function attempts to initialize the NVS flash. If no free pages are found 
 * or a new version of NVS is detected, it erases the flash and reinitializes it.
 * 
 * @note This function will log an error and terminate the program if NVS 
 * initialization fails.
 */
static void clear_nvs_flash(void) 
{
  /* Attempt to initialize NVS (Non-Volatile Storage) */
  esp_err_t ret = nvs_flash_init();

  /* If NVS flash has no free pages or a new version is found, erase and reinitialize */
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    /* Erase NVS flash if initialization failed due to no free pages or version mismatch */
    ESP_ERROR_CHECK(nvs_flash_erase());
    /* Reinitialize NVS flash after erasing */
    ret = nvs_flash_init();
  }

  /* Ensure NVS initialization was successful; terminate if an error occurs */
  ESP_ERROR_CHECK(ret);

  /* Log the successful initialization of NVS flash */
  ESP_LOGI(toporobo_tag, "ESP32 NVS flash cleared and initialized");
}

/**
 * @brief Initializes communication with the FPGA (DE-10 Lite).
 * 
 * This function will set up SPI communication between the ESP32 and the FPGA. 
 * It ensures proper configuration of SPI parameters and confirms the 
 * connection with the FPGA by sending an initial command.
 * 
 * @note This function assumes that the FPGA is preconfigured to accept 
 * SPI commands and is connected to the ESP32's SPI bus.
 */
static void fpga_comm_init(void)
{
  /* Pseudo code: */
  /* 1. Configure the SPI interface for FPGA communication */
  /* 2. Send an initialization signal to the FPGA */
  /* 3. Log the success or failure of the communication setup */
}

/**
 * @brief Initializes communication with the motor control boards (PCA9685).
 * 
 * This function will initialize I2C communication with two PCA9685 boards 
 * controlling MG946R servo motors. It will ensure the correct I2C settings are 
 * applied and verify the boards' responsiveness.
 * 
 * @note This function will configure I2C settings and prepare the motor drivers for control.
 */
static void motors_comm_init(void)
{
  /* Pseudo code: */
  /* 1. Initialize I2C communication with the PCA9685 motor driver boards */
  /* 2. Configure the PWM settings for the MG946R motors */
  /* 3. Log the success or failure of the motor driver initialization */
}

