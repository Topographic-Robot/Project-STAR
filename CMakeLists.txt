cmake_minimum_required(VERSION 3.16)

# Set CMP0175 to NEW to enforce correct usage of add_custom_command
cmake_policy(SET CMP0175 NEW)

# Print build information
message(STATUS "Project-Star: Build started")
message(STATUS "Project-Star: ESP-IDF path: $ENV{IDF_PATH}")
message(STATUS "Project-Star: Build type: ${CMAKE_BUILD_TYPE}")

# Apply size optimization flag in non-Debug builds
if(NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  message(STATUS "Project-Star: Applying size optimization flag (-Os)")
  add_compile_options(-Os)
endif()

# Set extra component directories to resolve header dependencies.
set(EXTRA_COMPONENT_DIRS
  "components/pstar_bus"
  "components/pstar_error_handler"
  "components/pstar_logger"
  "components/pstar_storage_hal"
  "components/pstar_managers"
  "components/pstar_pin_validator"
)

# Include ESP-IDF project CMake and define the project.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Project-Star)

# Print selected Kconfig options for quick reference
message(STATUS "Configuration Summary:")
message(STATUS "  Logging Enabled:           ${CONFIG_PSTAR_KCONFIG_LOGGING_ENABLED}")
message(STATUS "  Bus Component Enabled:     ${CONFIG_PSTAR_KCONFIG_BUS_COMPONENT_ENABLED}")
message(STATUS "  Error Handler Enabled:     ${CONFIG_PSTAR_KCONFIG_ERROR_HANDLER_ENABLED}")
message(STATUS "  Pin Validator Enabled:     ${CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED}")
message(STATUS "  Storage HAL Enabled:       ${CONFIG_PSTAR_KCONFIG_STORAGE_HAL_ENABLED}")

# Define core ESP-IDF include directories.
set(ESP_IDF_INCLUDE_DIRS
  "$ENV{IDF_PATH}/components/driver/include"
  "$ENV{IDF_PATH}/components/soc/include"
  "$ENV{IDF_PATH}/components/esp_common/include"
  "$ENV{IDF_PATH}/components/esp_system/include"
  "$ENV{IDF_PATH}/components/freertos/FreeRTOS-Kernel/include"
  "$ENV{IDF_PATH}/components/freertos/esp_additions/include"
  "$ENV{IDF_PATH}/components/hal/include"
  "$ENV{IDF_PATH}/components/log/include"
  "$ENV{IDF_PATH}/components/nvs_flash/include"
  "$ENV{IDF_PATH}/components/sdmmc/include"
  "$ENV{IDF_PATH}/components/vfs/include"
  "$ENV{IDF_PATH}/components/fatfs/diskio"
  "$ENV{IDF_PATH}/components/fatfs/vfs"
  "$ENV{IDF_PATH}/components/fatfs/src"
  "$ENV{IDF_PATH}/components/wear_levelling/include"
  "$ENV{IDF_PATH}/components/esp_partition/include"
  "$ENV{IDF_PATH}/components/esp_timer/include"
)

# Append both project and ESP-IDF include directories.
idf_build_set_property(INCLUDE_DIRECTORIES 
  "${CMAKE_SOURCE_DIR}/components/pstar_logger/include"
  "${CMAKE_SOURCE_DIR}/components/pstar_storage_hal/include"
  "${CMAKE_SOURCE_DIR}/components/pstar_storage_hal/sd_card_hal/include"
  "${CMAKE_SOURCE_DIR}/components/pstar_bus/include"
  "${CMAKE_SOURCE_DIR}/components/pstar_error_handler/include"
  "${CMAKE_SOURCE_DIR}/components/pstar_managers/include"
  "${CMAKE_SOURCE_DIR}/components/pstar_pin_validator/include"
  "${CMAKE_SOURCE_DIR}/main/include"
  ${ESP_IDF_INCLUDE_DIRS}
  APPEND
)

# Define a custom target to print a build summary.
add_custom_target(build_summary ALL
  COMMAND ${CMAKE_COMMAND} -E echo "Project-Star: Build completed successfully"
  COMMAND ${CMAKE_COMMAND} -E echo "Project-Star: Enabled components (as registered):"
  VERBATIM
  DEPENDS app
)

# Append a custom command with an explicit POST_BUILD option for each extra component.
foreach(COMPONENT ${EXTRA_COMPONENT_DIRS})
  add_custom_command(TARGET build_summary POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "  - ${COMPONENT}"
    VERBATIM
  )
endforeach()

