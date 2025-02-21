# Changelog

All notable changes to Project Star will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Error handling system implementation (by @bsikar)

  - Added to BH1750 and CCS811 sensors
  - Consistent error handling pattern with exponential backoff
  - Error recovery mechanisms in all sensor modules
  - Unified error state management

- EC11 Encoder integration (by @bsikar)
  - Interrupt and polling support
  - Motor control integration
  - Individual encoder per motor support (planned)
  - Thread-safe access in ISR handlers

### Changed

- Documentation improvements (by @bsikar)
  - Added comprehensive doc comments
  - Code formatting updates
  - Improved README documentation
  - Updated wiring documentation

### Fixed

- WiFi connectivity and stability (by @bsikar)
  - Enhanced error handling in WiFi tasks
  - Improved connection retry logic
  - Better state management
- PCA9685 configuration and timing (by @bsikar)
- Motor task management and synchronization (by @bsikar)
- System task initialization sequence (by @bsikar)
- Line ending standardization to LF (by @bsikar)

## [0.2.0] - 2024-12-25

### Added

- Gait movement system (by @bsikar)

  - Helper functions implementation
  - Basic gait logic
  - Movement calculations
  - Rounding fixes for precision
  - Motor synchronization improvements
  - Position feedback integration

- Camera system (OV7670) (by @bsikar)
  - Basic configuration setup
  - Default initialization
  - Preparation for multi-camera support
  - I2C configuration management
  - Frame timing control

### Changed

- README updates (by @IMMZEK)
  - Added detailed installation instructions
  - Updated wiring documentation
  - Improved project description
  - Added JTAG debugging section

### Removed

- Deprecated gait implementation files
- Unused sensor configuration files

## [0.1.0] - 2024-11-14

### Added

- Web server implementation (by @bsikar)

  - Basic server setup
  - IP configuration management
  - Error handling
  - Data streaming support

- Sensor implementations (by @bsikar)

  - MPU6050 motion sensor with improved accuracy
  - GY-NEO6MV2 GPS module with UART optimization
  - QMC5883L compass with calibration
  - DHT22 temperature/humidity sensor with retry logic
  - BH1750 light sensor with power management
  - Air quality sensors (CCS811, MQ135) with filtering

- SD Card support (by @bsikar)

  - FATFS implementation
  - Writing queue system
  - Time server integration
  - File integrity checks
  - Automatic mount/unmount

- JTAG debugging support (by @bsikar)
  - Pin mapping documentation
  - Debugging configuration
  - Integration with development workflow
  - GDB script support

### Fixed

- GPS module functionality and timing (by @bsikar)
- SD Card FATFS stability issues (by @bsikar)
- I2C bus communication reliability (by @bsikar)
- Sensor value accuracy and calibration (by @bsikar)

## [0.0.1] - 2024-07-19

### Added

- Initial project setup (by @bsikar)
  - Basic LED functionality
  - PCA9685 motor driver support
  - EC11 encoder initial implementation
  - WiFi station mode
  - Basic sensor HAL structure
  - Initial I2C and UART abstractions

### Changed

- Development environment setup (by @Jhews6)
  - Cross-platform support (Windows and Mac)
  - Build system configuration
  - Toolchain setup
  - Development workflow documentation

### Fixed

- Motor current draw optimization (by @bsikar)
- Initial WiFi connectivity stability (by @bsikar)
- Build system compatibility issues (by @Jhews6)

## Contributors

- Brighton Sikarskie (@bsikar)
  - Core system architecture
  - Sensor integration
  - Motor control
  - Error handling
- Cesar Magana (@IMMZEK)
  - Documentation
  - Testing
- Jeremie Hews (@Jhews6)
  - Build system
  - Cross-platform support
- Matthew Manginelli (@Mattinelli)
  - Code review
  - Quality assurance

## Notes

- Dates are in YYYY-MM-DD format
- Version numbers are assigned based on major feature additions and breaking changes
- Some early development commits may be grouped for clarity
- Detailed commit history available in git logs
