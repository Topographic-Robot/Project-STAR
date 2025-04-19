# Project-STAR: Survey and Terrain Analysis Robot (STAR)

[![Documentation Status](https://img.shields.io/badge/docs-latest-brightgreen.svg)](https://topographic-robot.github.io/Topographic-Robot-Documentation/html/index.html)

**STAR (Survey and Terrain Analysis Robot)** is an advanced terrain-mapping robot designed to autonomously navigate and create detailed 3D mesh topology maps of various landscapes, including hills and valleys. This repository contains the firmware for the ESP32 microcontroller that powers STAR. You can find comprehensive information, including hardware specifications, software architecture, and API documentation, on the project's dedicated documentation site hosted on GitHub Pages.

## Table of Contents

- [Project-STAR: Survey and Terrain Analysis Robot (STAR)](#project-star-survey-and-terrain-analysis-robot-star)
  - [Table of Contents](#table-of-contents)
  - [Features](#features)
  - [Hardware](#hardware)
  - [Software](#software)
    - [Architecture Diagrams](#architecture-diagrams)
  - [Installation](#installation)
  - [Configuration](#configuration)
    - [Using menuconfig](#using-menuconfig)
    - [Validating Kconfig Files](#validating-kconfig-files)
    - [Fixing Kconfig Warnings (Step 2)](#fixing-kconfig-warnings-step-2)
    - [Kconfig Validation and Update Script](#kconfig-validation-and-update-script)
    - [ESP-IDF Kconfig Documentation](#esp-idf-kconfig-documentation)
    - [Additional ESP-IDF Resources](#additional-esp-idf-resources)
  - [JTAG Debugging](#jtag-debugging)
  - [Documentation](#documentation)
  - [Contributing](#contributing)
  - [License](#license)

## Features

- **Terrain Mapping:** Generates detailed 3D mesh topology maps of diverse terrains.
- **Autonomous Navigation:** Designed to traverse hills, valleys, and other challenging landscapes.
- **Sensor Suite:** Equipped with a comprehensive set of sensors for data acquisition:
  - **Environmental:** DHT22 (Temperature & Humidity), BH1750 (Ambient Light), SEN-CCS811 & MQ135 (Air Quality).
  - **Motion & Orientation:** MPU6050 (Accelerometer & Gyroscope), QMC5883L (Magnetometer).
  - **Localization:** GY-NEO6MV2 (GPS).
  - **Imaging:** OV7670 Camera (controlled by DE10-Lite FPGA).
- **Data Storage:** Utilizes a Micro-SD card for onboard data logging.
- **Remote Communication:** Implements Wi-Fi connectivity for data transmission and potential remote control.
- **Modular Design:** Built using ESP-IDF with a modular component structure for easy maintenance and expansion.
- **JTAG Debugging:** Supports JTAG debugging for development and troubleshooting.
- **Interrupt-Driven Sensor Handling:** The MPU6050 sensor implementation uses interrupts for efficient data acquisition.
- **SD Card Hot-Swap:** Supports SD card hot-swapping with automatic detection using the CD pin.

## Hardware

The core of STAR is an ESP32 microcontroller. It interacts with the following hardware components:

- **Microcontroller:** ESP32
- **Sensors:**
  - DHT22 (Temperature & Humidity)
  - MPU6050 (Accelerometer & Gyroscope)
  - BH1750 (Ambient Light)
  - QMC5883L (Magnetometer)
  - OV7670 (Camera, controlled by DE10-Lite)
  - SEN-CCS811 (Air Quality - eCO2, TVOC)
  - MQ135 (Air Quality - various gases)
  - GY-NEO6MV2 (GPS)
- **Storage:** Micro-SD Card Module
- **Actuators:** Motors controlled via PCA9685 PWM Driver
- **Communication:** Wi-Fi (ESP32 built-in)
- **Power:** 3.3V power supply for most components, with potential external power for motors
- **FPGA (External):** DE10-Lite (for camera control)
- **JTAG Debugger:** CJMCU-4232 (optional, for debugging)

**Schematics:**

Refer to the images below for the schematics of the robot:

![Print Schematic-2](https://github.com/user-attachments/assets/91e3658f-626d-4d3d-8d84-060d94f8161d)
![Print Schematic-3](https://github.com/user-attachments/assets/b480c20d-7b98-4bb1-b207-027b9d4634cc)
![Print Schematic-4](https://github.com/user-attachments/assets/fd70dec2-0874-4dc0-99f8-d9a695c5698a)
![Print Schematic-5](https://github.com/user-attachments/assets/cfb79a64-f3b6-4450-9d41-8d787df71902)

## Software

The firmware is developed using the **ESP-IDF** framework and written primarily in **C**. It utilizes **FreeRTOS** for task management.

**Software Components:**

- **`components/`:** Contains modular components:
  - **`camera/`:** Hardware Abstraction Layer (HAL) for the OV7670 camera.
  - **`common/`:** I2C, SPI, and UART communication drivers.
  - **`controllers/`:** HAL for the PCA9685 PWM driver.
  - **`sensors/`:** HALs for each sensor (BH1750, DHT22, MPU6050, QMC5883L, GY-NEO6MV2, CCS811, MQ135).
  - **`storage/`:** HAL for SD card interactions.
- **`main/`:**
  - **`include/managers/`:** Time management and file writing utilities.
  - **`include/tasks/`:** Task definitions for motor control, sensor data acquisition, Wi-Fi management, and web server communication.
  - `main.c`: Main application entry point.

### Architecture Diagrams

The project includes architectural diagrams to help visualize the system design:

- **ESP32 Bus Manager Functional Block Diagram:** The `esp32_bus_manager_fbd.svg` file illustrates the communication architecture between the ESP32 and its peripheral components, showing how I2C, SPI, and UART buses are managed to connect various sensors and devices. This diagram is essential for understanding the hardware interfaces and data flow within the system.

![ESP32 Bus Manager Block Diagram](./docs/diagrams/esp32_bus_manager_fbd.svg)

**Dependencies:**

- **ESP-IDF:** v4.4 or later (check `idf.py --version`)
- **cJSON:** For JSON data formatting.
- **FATFS:** For SD card file system operations.

## Installation

1. **Install ESP-IDF:**

   Follow the official ESP-IDF installation instructions:
   [https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)

2. **Ensure Latest `clang-format` (Optional but Recommended):**

   If you are contributing to the project and use `clang-format` for formatting code, make sure it is up to date (preferably version 20.1.1 or later).
   On macOS, run the following:
