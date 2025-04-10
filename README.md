# Project-STAR: Survey and Terrain Analysis Robot (STAR)

[![Documentation Status](https://img.shields.io/badge/docs-latest-brightgreen.svg)](https://topographic-robot.github.io/Topographic-Robot-Documentation/html/index.html)

**STAR (Survey and Terrain Analysis Robot)** is an advanced terrain-mapping robot designed to autonomously navigate and create detailed 3D mesh topology maps of various landscapes, including hills and valleys. This repository contains the firmware for the ESP32 microcontroller that powers STAR. You can find comprehensive information, including hardware specifications, software architecture, and API documentation, on the project's dedicated documentation site hosted on GitHub Pages.

## Table of Contents

- [Project-STAR: Survey and Terrain Analysis Robot (STAR)](#project-star-survey-and-terrain-analysis-robot-star)
  - [Table of Contents](#table-of-contents)
  - [Features](#features)
  - [Hardware](#hardware)
  - [Software](#software)
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

   ```bash
   brew update
   brew upgrade llvm
   ```

   You can check the version of `clang-format` installed:

   ```bash
   clang-format --version
   ```

   > ⚠️ **Note:** If `clang-format` is not in your path after upgrading LLVM, you may need to link it manually or use the full path (e.g., `/opt/homebrew/opt/llvm/bin/clang-format`).

3. **Clone the Repository:**

   ```bash
   git clone https://github.com/Topographic-Robot/Project-Star.git
   cd Project-Star
   ```

4. **Configure Wi-Fi Credentials:**

   - Copy the `main/include/tasks/include/wifi_credentials.txt` file to `main/include/tasks/include/wifi_credentials.h`.
   - Edit `wifi_credentials.h` and replace the placeholder values for `wifi_ssid` and `wifi_pass` with your actual Wi-Fi network credentials. **Do not commit this file to Git.**

5. **Configure Webserver URL (Optional):**

   - If you intend to send sensor data to a web server, copy the `main/include/tasks/include/webserver_info.txt` file to `main/include/tasks/include/webserver_info.h`.
   - Edit `webserver_info.h` and replace the placeholder value for `webserver_url` with your server's URL. **Do not commit this file to Git.**

6. **Configure Project Settings:**

   ```bash
   idf.py menuconfig
   ```

   - Navigate to the "Project STAR Configuration" section to configure sensor settings, sampling rates, and other project-specific options.
   - Ensure that all necessary features are enabled for your specific hardware configuration.

7. **Build and Flash:**

   ```bash
   idf.py set-target esp32
   idf.py build
   idf.py -p /dev/ttyUSB0 flash monitor  # Replace /dev/ttyUSB0 with your ESP32's port
   ```

## Configuration

STAR uses the ESP-IDF's Kconfig system to manage project configuration options. This provides a flexible way to enable/disable features and set various parameters without modifying the code.

### Using menuconfig

To access the configuration interface:

```bash
idf.py menuconfig
```

This will open a text-based configuration interface where you can navigate through different categories and adjust settings for your specific needs.

### Validating Kconfig Files

To check the syntax and validity of a specific Kconfig file:

```bash
python -m kconfcheck <path_to_kconfig_file>
```

To check all Kconfig files in the project:

```bash
find . -name "Kconfig*" -exec python -m kconfcheck {} \;
```

### Fixing Kconfig Warnings (Step 2)

**Issue:**
When loading `sdkconfig.defaults`, warnings like "unknown kconfig symbol 'PSTAR_KCONFIG_BUS_COMPONENT_ENABLED' assigned to 'y'" appear because the defaults file is processed before component Kconfig files are loaded.

**Solution:**
Create a root‑level `Kconfig` file that sources all your component Kconfig files. This ensures that all configuration symbols are defined before `sdkconfig.defaults` is processed.

**Steps:**

1. **Create the Root‑Level Kconfig File:**
   Place a file named `Kconfig` in the project's root directory (same level as `CMakeLists.txt`) with the following content:

   ```kconfig
   # Root Kconfig file for Project-Star

   # Include project-level configuration (if any)
   source "main/Kconfig.projbuild"

   # Include component Kconfig files to define all custom symbols
   source "components/pstar_logger/Kconfig"
   source "components/pstar_bus/Kconfig"
   source "components/pstar_managers/Kconfig"
   source "components/pstar_error_handler/Kconfig"
   source "components/pstar_pin_validator/Kconfig"
   source "components/pstar_storage_hal/Kconfig"
   ```

2. **Clean and Reconfigure:**
   - Run `idf.py fullclean` to remove previous build artifacts.
   - Then run `idf.py menuconfig` to load the new configuration.
   - Verify that the warnings about unknown Kconfig symbols no longer appear.

### Kconfig Validation and Update Script

An included shell script (`check_and_update_kconfig.sh`) automates the process of validating your Kconfig files and updating them if necessary. This script performs the following actions:

- **Runs `kconfcheck`:** It scans all Kconfig files in the project to validate their syntax.
- **Retries if Needed:** It will retry up to a set maximum number of attempts (default is 10) if `kconfcheck` fails.
- **Updates Files:** Upon successful validation, it moves any generated `Kconfig.new` files to replace the original ones.

**Script Content:**

```bash
#!/usr/bin/env bash
# check_and_update_kconfig.sh

max_attempts=10
attempt=0

while [ $attempt -lt $max_attempts ]; do
  echo "Attempt $((attempt + 1)) of $max_attempts: Running kconfcheck on all Kconfig files..."

  # Run kconfcheck on all Kconfig files
  find . -name "Kconfig*" -exec python -m kconfcheck {} \;

  # Capture exit code
  exit_code=$?

  if [ $exit_code -eq 0 ]; then
    echo "kconfcheck completed successfully!"

    # Move Kconfig.new files to replace originals
    find . -name "Kconfig.new" -exec sh -c 'echo "Moving $0 to ${0%.new}"; mv "$0" "${0%.new}"' {} \;

    if [ $? -eq 0 ]; then
      echo "All Kconfig files checked and updated successfully."
      exit 0
    else
      echo "Error moving Kconfig.new files."
      exit 1
    fi
  fi

  echo "kconfcheck failed (exit code $exit_code), retrying..."
  attempt=$((attempt + 1))
done

echo "Reached maximum attempts ($max_attempts). kconfcheck did not succeed."
exit 1
```

**Usage:**

1. Make the script executable:

   ```bash
   chmod +x check_and_update_kconfig.sh
   ```

2. Run the script from the project's root directory:

   ```bash
   ./check_and_update_kconfig.sh
   ```

This script helps ensure that your Kconfig files are valid and that any updates (if `kconfcheck` generates `.new` files) are applied automatically.

### ESP-IDF Kconfig Documentation

For detailed information about the Kconfig system and how to create or modify configuration options, refer to:
- [ESP-IDF Kconfig Reference](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/kconfig.html)
- [ESP-IDF Configuration Options Reference](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/config.html)

### Additional ESP-IDF Resources

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
- [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)
- [ESP-IDF API Reference](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/index.html)
- [ESP-IDF Examples](https://github.com/espressif/esp-idf/tree/master/examples)

## JTAG Debugging

JTAG (Joint Test Action Group) debugging is crucial for in-depth troubleshooting and development. It allows you to step through code, set breakpoints, inspect memory, and diagnose issues that might be difficult to identify otherwise. This project supports JTAG debugging using a CJMCU-4232 debugger, OpenOCD (Open On-Chip Debugger), and GDB.

**Hardware Setup:**

1. **CJMCU-4232 Debugger:** This project uses the CJMCU-4232, a low-cost FT2232-based USB to JTAG interface.
2. **Connections:** Connect the CJMCU-4232 to the ESP32 according to the following table, which is also found in the `JTAG-info.txt` file:

   | CJMCU-4232 Pin | ESP32 Pin | JTAG Signal            |
   | -------------- | --------- | ---------------------- |
   | AD0            | D13       | TCK (Test Clock)       |
   | AD1            | D12       | TDI (Test Data In)     |
   | AD2            | D15       | TDO (Test Data Out)    |
   | AD3            | D14       | TMS (Test Mode Select) |
   | GND            | GND       | Ground                 |
   | 3V3            | 3V3       | Power                  |

**Software Setup:**

1. **Install OpenOCD:** If you don't have OpenOCD installed, follow the installation instructions for your operating system. You can usually find it in your system's package manager (e.g., `apt-get` on Ubuntu/Debian, `brew` on macOS). If you installed the ESP-IDF, you should already have OpenOCD.

2. **Install GDB:** You need the Xtensa-specific GDB for debugging the ESP32. If you installed the ESP-IDF, then it should be in your path. Test this by running `xtensa-esp32-elf-gdb --version`

**Debugging Steps:**

1. **Identify the ESP32 (Optional but Recommended):**
   - Run `lsusb` (on Linux/macOS) or the equivalent command on your OS to list connected USB devices.
   - Look for the entry corresponding to the CJMCU-4232 or a similar FTDI device.
   - Note the Vendor ID (VID) and Product ID (PID). You might need these values for OpenOCD configuration. Example output, which identifies the `VID:PID` as `0403:6011`:
     ```
     Bus 000 Device 004: ID 0403:6011 Future Technology Devices International Limited Quad RS232-HS
     ```
2. **Start OpenOCD:**

   - Open a terminal and run the following command:

     ```bash
     openocd -f board/esp32-wrover-kit-3.3v.cfg -c "ftdi_vid_pid 0x0403 0x6011"
     ```

     - **`-f board/esp32-wrover-kit-3.3v.cfg`:** This specifies the configuration file for the ESP32-WROVER-KIT board, which has similar JTAG settings. You can adapt this to other ESP32 boards if needed.
     - **`-c "ftdi_vid_pid 0x0403 0x6011"`:** This sets the VID and PID for the CJMCU-4232. Replace `0x0403` and `0x6011` with the values you identified in step 1 if they are different.

   - You should see output indicating that OpenOCD has started successfully and is listening on port 3333 for GDB connections.

3. **Connect with GDB:**

   - Open another terminal.
   - Navigate to your project directory.
   - Run the following command to start GDB and connect to OpenOCD:

     ```bash
     xtensa-esp32-elf-gdb -ex "target remote localhost:3333" build/Topographic-Robot.elf
     ```

     - Assuming the project's ELF file is named `Topographic-Robot.elf` and is located in the `build` directory. Adjust the path if necessary.

   - You should now be in the GDB prompt, connected to your ESP32.

**Basic GDB Commands:**

- `monitor reset halt`: Reset and halt the ESP32.
- `b <function_name or file:line_number>`: Set a breakpoint.
- `c`: Continue execution.
- `p <variable_name>`: Print the value of a variable.
- `n`: Step to the next line.
- `s`: Step into a function.
- `q`: Quit GDB.

**Example Debugging Session:**

1. Start OpenOCD (as described above).
2. Connect with GDB (as described above).
3. Set a breakpoint at the beginning of your `app_main` function:

   ```
   (gdb) b app_main
   Breakpoint 1 at 0x400d1234: file main/main.c, line 123.
   ```

4. Continue execution:

   ```
   (gdb) c
   Continuing.
   ```

5. The program will run until it hits the breakpoint at `app_main`.
6. You can now step through the code, inspect variables, and use other GDB commands to debug your program.

**Important Notes:**

- **Make sure your ESP32 is not being flashed or monitored by another program** when using JTAG, as this can cause conflicts.
- **Consult the OpenOCD documentation** for more advanced configuration options and troubleshooting: [http://openocd.org/](http://openocd.org/)
- **Refer to the GDB documentation** for a comprehensive list of commands: [https://sourceware.org/gdb/current/onlinedocs/gdb/](https://sourceware.org/gdb/current/onlinedocs/gdb/)

## Documentation

For more detailed information about the project, including hardware specifications, software architecture, and API documentation, please refer to the online documentation hosted on GitHub Pages:

[https://topographic-robot.github.io/Topographic-Robot-Documentation/html/index.html](https://topographic-robot.github.io/Topographic-Robot-Documentation/html/index.html)

## Contributing

Contributions to Project-Star are welcome! Please follow these guidelines:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Write clear and concise code with proper comments.
4. Test your changes thoroughly.
5. Submit a pull request with a detailed description of your changes.

## License

This project is licensed under the [MIT License](LICENSE) - see the `LICENSE` file for details.