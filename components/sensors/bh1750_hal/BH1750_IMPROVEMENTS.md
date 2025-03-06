# BH1750 Light Sensor Improvements

This document outlines the specific improvements that should be implemented for the BH1750 light sensor in the Project Star system.

## 1. Enhanced Error Handling and Recovery

- [ ] Implement robust error detection and recovery mechanism for I2C communication
- [ ] Add reset function that power cycles the BH1750 and reconfigures it
- [ ] Implement exponential backoff for retry attempts to avoid excessive resets
- [ ] Add tracking of consecutive errors and timeouts during I2C communication
- [ ] Implement automatic recovery from communication failures
- [ ] Add verification of sensor functionality through test measurements

## 2. Improved Data Processing

- [ ] Add moving average filter to reduce noise in light intensity readings
- [ ] Implement validation of readings against expected ranges
- [ ] Enhance data conversion process with better lux calculation
- [ ] Add outlier detection and filtering for sudden light intensity changes
- [ ] Implement signal processing algorithms for more accurate light readings
- [ ] Add compensation for different light sources (natural vs. artificial)
- [ ] Implement adaptive sensitivity based on light conditions

## 3. Calibration System

- [ ] Create comprehensive calibration process with reference light source
- [ ] Add functions to save and load calibration values to/from NVS
- [ ] Implement automatic loading of calibration values during initialization
- [ ] Add user-triggered calibration routine
- [ ] Implement periodic recalibration checks
- [ ] Add sensitivity adjustment features
- [ ] Implement cross-validation with other light sensors

## 4. Power Management

- [ ] Configure and utilize BH1750's power-down mode
- [ ] Implement adaptive sampling rate based on light stability
- [ ] Add proper handling of sensor state during power transitions
- [ ] Implement power consumption monitoring
- [ ] Add low-power modes for battery operation
- [ ] Optimize measurement time vs. resolution trade-offs

## 5. Sensor Health Monitoring

- [ ] Add tracking of consecutive errors and timeouts
- [ ] Implement watchdog mechanism to detect unresponsive sensor
- [ ] Add periodic logging of sensor health statistics
- [ ] Implement diagnostic routines for sensor troubleshooting
- [ ] Monitor sensor consistency and drift over time

## 6. Self-Test Functionality

- [ ] Create comprehensive self-test function that verifies BH1750 functionality
- [ ] Add checks for sensor communication and measurement validity
- [ ] Integrate self-test into the initialization process
- [ ] Implement periodic self-tests during operation
- [ ] Add detailed reporting of self-test results
- [ ] Verify operation in different sensitivity modes

## 7. Task Management

- [ ] Improve sensor task with better error handling and recovery
- [ ] Add tracking of successful readings and error rates
- [ ] Enhance handling of measurement timing and delays
- [ ] Implement priority-based processing of sensor data
- [ ] Add task statistics collection for performance monitoring
- [ ] Optimize timing between continuous and one-time measurements

## 8. Advanced Light Sensing Features

- [ ] Implement multiple measurement modes (high resolution, high sensitivity)
- [ ] Add light change detection with configurable thresholds
- [ ] Implement day/night transition detection
- [ ] Add light color temperature estimation (with additional sensors)
- [ ] Implement ambient light adaptive control systems
- [ ] Add light pattern detection (flashing, pulsing)

## Implementation Guidelines

1. **Consistent API**: Maintain a consistent API:
   - `bh1750_init()` - Initialize the sensor
   - `bh1750_read_lux()` - Read light intensity in lux
   - `bh1750_set_mode()` - Set measurement mode
   - `bh1750_set_mtreg()` - Set measurement time register
   - `bh1750_power_down()` - Put sensor in power-down mode
   - `bh1750_power_on()` - Wake sensor from power-down mode
   - `bh1750_reset()` - Reset the sensor
   - `bh1750_self_test()` - Perform self-test
   - `bh1750_calibrate()` - Calibrate the sensor

2. **Mode Management**: Create abstraction functions for different measurement modes:
   - `bh1750_continuous_high_resolution_mode()` - 1 lux resolution with 120ms
   - `bh1750_continuous_high_resolution_mode_2()` - 0.5 lux resolution with 120ms
   - `bh1750_continuous_low_resolution_mode()` - 4 lux resolution with 16ms
   - `bh1750_one_time_high_resolution_mode()` - 1 lux resolution, one-time
   - `bh1750_one_time_high_resolution_mode_2()` - 0.5 lux resolution, one-time
   - `bh1750_one_time_low_resolution_mode()` - 4 lux resolution, one-time

3. **Error Handling**: Use ESP-IDF error codes consistently:
   - Return `ESP_OK` for successful operations
   - Return appropriate error codes for failures
   - Log detailed error information

4. **Thread Safety**: Ensure all functions are thread-safe:
   - Use mutexes to protect shared resources
   - Implement critical section protection where necessary

5. **Measurement Timing**:
   - Implement proper timing for different measurement modes
   - Handle measurement delays in a non-blocking way
   - Provide functions for both synchronous and asynchronous measurements 