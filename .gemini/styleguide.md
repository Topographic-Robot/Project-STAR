# Project-Star Codebase Style Guide

This document outlines the coding style and conventions to be followed within the Project-Star codebase. Adhering to these guidelines will ensure consistency, readability, and maintainability across the project.

## General Principles

-   **Clarity:** Code should be easy to understand, even for someone unfamiliar with the specific module.
-   **Consistency:** Maintain a uniform style across all files.
-   **Readability:** Prioritize code that is easy to read and comprehend.
-   **Maintainability:** Design code that is easy to modify and extend.
-   **Efficiency:** Strive for efficient algorithms and data structures, without sacrificing readability.
-   **Testability**: Code should be written with testability in mind, making it easy to write unit and integration tests.

## Naming Conventions

-   **Variables:**
    -   Use `snake_case` for variable names (e.g., `sensor_data`, `motor_speed`).
    -   Be descriptive and concise. Avoid single-letter variables unless they are loop counters.
    -   Prefix static variables with `s_`.
    -   Prefix global variables with `g_`.
    -   Constants should be all uppercase with underscores (e.g., `MAX_SPEED`, `BUFFER_SIZE`).
-   **Functions:**
    -   Use `snake_case` for function names (e.g., `read_sensor_data`, `control_motor`).
    -   Function names should clearly indicate the function's purpose.
    -   Use verbs for function names.
-   **Types:**
    -   Use `CamelCase` for struct, enum, and type names (e.g., `SensorData`, `MotorStatus`).
    -   When using a struct to provide a set of configurations, name it with a Config suffix (e.g. `WifiConfig`)
-   **Files:**
    -   Use `snake_case` for file names (e.g., `sensor_manager.c`, `motor_driver.h`).
    -   Header files should have the `.h` extension.
    -   Source files should have the `.c` extension.
- **Macros:**
    - Use all caps and underscores (e.g. `MAX_SENSORS`)

## Code Formatting

-   **Indentation:**
    -   Use 4 spaces for indentation.
    -   Do not use tabs.
-   **Line Length:**
    -   Limit lines to 100 characters.
-   **Braces:**
    -   Use braces for all control flow statements, even single-line ones (e.g., `if`, `else`, `for`, `while`).
    -   Place the opening brace on the same line as the statement, and the closing brace on a new line:

    ```c
    if (condition) {
        // Code block
    } else {
        // Another code block
    }
    ```

-   **Spacing:**
    -   Use spaces around operators (e.g., `x = a + b`, not `x=a+b`).
    -   Use spaces after commas (e.g., `function(a, b, c)`).
    -   No spaces inside parentheses, brackets, or braces (e.g., `function(a)`).
    -   Put a space after keywords such as `if`, `else`, `while`, etc. (e.g. `if (true)`).
- **Empty lines**
    - Use empty lines to separate logical blocks of code.
    - Use empty lines to separate functions.
-   **Comments:**
    -   Use `//` for single-line comments.
    -   Use `/* ... */` for multi-line comments.
    -   Comments should explain *why* the code is doing something, not just *what* it's doing.
    -   Use detailed comments at the top of each file, describing the module's purpose, author, and creation date.
    -   Comment complex logic thoroughly.
    -   Use Doxygen-style comments for public APIs.
-   **Includes:**
    -   Include files should be listed alphabetically and grouped into system libraries and project-specific includes.
    -   Use angle brackets `<>` for system header files (e.g., `#include <stdio.h>`).
    -   Use double quotes `""` for project header files (e.g., `#include "sensor_data.h"`).
    -  Protect header files with include guards to prevent multiple inclusion.
    ```c
    #ifndef SENSOR_MANAGER_H
    #define SENSOR_MANAGER_H
    // ... content of the header file ...
    #endif // SENSOR_MANAGER_H
    ```

## Code Structure

-   **Modular Design:**
    -   Divide the codebase into logical modules (e.g., `sensors`, `controllers`, `storage`).
    -   Each module should have a clear responsibility.
    -   Modules should interact with each other through well-defined interfaces.
-   **Header Files:**
    -   Each source file should have a corresponding header file.
    -   Header files should contain:
        -   Function prototypes.
        -   Struct and enum definitions.
        -   Constant declarations.
        -   Macro definitions.
    - Header files should not contain function implementations.
-   **Source Files:**
    -   Source files should contain the implementation of functions declared in their corresponding header files.
    - Should include its corresponding header file.
-   **Error Handling:**
    -   Implement robust error handling throughout the codebase.
    -   Use return codes to indicate the success or failure of functions.
    -   Log errors to provide context for debugging.
    -   Implement error recovery mechanisms where appropriate.
- **Tasks and Threads**
    - When creating a FreeRTOS task, always provide a meaningful name to the task.
- **Interrupts**
    - Minimize the work done inside an Interrupt Service Routine.
    - If the amount of work is substantial, defer it to a separate task or thread.
- **Synchronization**
    - When using FreeRTOS, use the proper synchronization primitives, such as Mutexes and Semaphores.
    - Avoid busy loops.

## Specific C/ESP-IDF Practices

-   **ESP-IDF API:**
    -   Use the official ESP-IDF APIs whenever possible.
    -   Refer to the ESP-IDF documentation for API usage.
-   **FreeRTOS:**
    -   Follow FreeRTOS best practices for task management, inter-task communication, and synchronization.
- **Memory Management:**
    - Be aware of memory usage, especially on embedded systems.
    - Release dynamically allocated memory when it's no longer needed.

## Documentation

-   **README:**
    -   The `README.md` file should provide a high-level overview of the project.
    -   It should explain the project's purpose, features, hardware, software, installation, usage, and contribution guidelines.
    -   All the schematics files should be referenced in the README.md file.
-   **CHANGELOG:**
    -   Maintain a `CHANGELOG.md` file to track changes made to the project.
    -   Use the "Keep a Changelog" format.
    -   Document added, changed, fixed, and removed features.
-   **Doxygen:**
    -   Use Doxygen-style comments for all public APIs.
    -   Generate HTML documentation using Doxygen.
- **Comments in code**
    - If a functionality change was needed, comment what was done before, what was changed and why.
- **Wiring**
    - Keep wiring and JTAG information up to date.

## Version Control (Git)

-   **Commit Messages:**
    -   Write clear and concise commit messages.
    -   Use the imperative mood (e.g., "Fix bug," "Add feature," not "Fixed bug" or "Adding feature").
    -   Describe *what* was changed and *why*.
-   **Branching:**
    -   Use feature branches for new development.
    -   Use descriptive branch names (e.g., `feature/sensor-integration`, `bugfix/wifi-stability`).
    -  Use pull request to merge feature branches.
- **Pull requests**
    - All code should be review by other developers before merging it to the main branch.

## Testing

-   **Unit Tests:**
    -   Write unit tests for all critical modules and functions.
    -   Use a testing framework (e.g., Unity).
-   **Integration Tests:**
    -   Write integration tests to ensure that different modules work together correctly.
-   **Code Coverage:**
    -   Aim for high code coverage to improve reliability.
-   **Testing Hardware**
    - When testing hardware, add comments or a README.md on how the hardware can be tested.

## Example

```c
/**
 * @file sensor_manager.c
 * @brief Manages the data acquisition from various sensors.
 *
 * This file contains functions to initialize and read data from the various
 * sensors installed on the robot.
 *
 * @author Brighton Sikarskie
 * @date 2024-07-19
 */

#include "sensor_manager.h"
#include <stdio.h> //System Includes first
#include "dht22.h"
#include "mpu6050.h"
// ... other includes ...

#define MAX_SENSORS 10

static int s_num_sensors = 0;

/**
 * @brief Initializes all sensors.
 *
 * This function initializes all the sensors, including the DHT22, MPU6050, etc.
 *
 * @return true if all sensors initialized successfully, false otherwise.
 */
bool initialize_sensors(void) {
    bool success = true;
    if (!dht22_init()) {
        ESP_LOGE("SENSOR_MANAGER", "Failed to initialize DHT22");
        success = false;
    }
    if (!mpu6050_init()) {
        ESP_LOGE("SENSOR_MANAGER", "Failed to initialize MPU6050");
        success = false;
    }
    // ... initialize other sensors ...

    return success;
}

/**
 * @brief Reads the current temperature and humidity from the DHT22 sensor.
 *
 * @param temperature Pointer to store the temperature value.
 * @param humidity Pointer to store the humidity value.
 * @return true if data was read successfully, false otherwise.
 */
bool read_dht22_data(float *temperature, float *humidity) {
    if (dht22_read(temperature, humidity)) {
        s_num_sensors ++;
        return true;
    }
    return false;
}

