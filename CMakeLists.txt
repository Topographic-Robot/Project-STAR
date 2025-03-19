cmake_minimum_required(VERSION 3.16)

# Print build information
message(STATUS "Project-Star: Build started")
message(STATUS "Project-Star: ESP-IDF path: $ENV{IDF_PATH}")
message(STATUS "Project-Star: Build type: ${CMAKE_BUILD_TYPE}")

# Configure components based on Kconfig settings
set(EXTRA_COMPONENT_DIRS)

# Include all components by default for now to resolve header dependencies
list(APPEND EXTRA_COMPONENT_DIRS 
  "components/pstar_bus"
  "components/pstar_error_handler"
  "components/pstar_logger"
  "components/pstar_storage_hal"
  "components/pstar_managers"
  "components/pstar_pin_validator"
)

# Include ESP-IDF project cmake
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Project-Star)

# Add ESP-IDF components to include paths explicitly
set(ESP_IDF_INCLUDE_DIRS
  # Core ESP-IDF components
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

# Add all component include directories for proper resolution of header files
idf_build_set_property(INCLUDE_DIRECTORIES 
  # Project components
  "${CMAKE_SOURCE_DIR}/components/pstar_logger/include"
  "${CMAKE_SOURCE_DIR}/components/pstar_storage_hal/include"
  "${CMAKE_SOURCE_DIR}/components/pstar_storage_hal/sd_card_hal/include"
  "${CMAKE_SOURCE_DIR}/components/pstar_bus/include"
  "${CMAKE_SOURCE_DIR}/components/pstar_error_handler/include"
  "${CMAKE_SOURCE_DIR}/components/pstar_managers/include"
  "${CMAKE_SOURCE_DIR}/components/pstar_pin_validator/include"
  "${CMAKE_SOURCE_DIR}/main/include"
  # ESP-IDF includes
  ${ESP_IDF_INCLUDE_DIRS}
  APPEND)

# Print summary after project is configured
add_custom_target(build_summary ALL
  COMMAND ${CMAKE_COMMAND} -E echo "Project-Star: Build completed successfully"
  COMMAND ${CMAKE_COMMAND} -E echo "Project-Star: Enabled components:"
  VERBATIM
  DEPENDS app
)

foreach(COMPONENT ${EXTRA_COMPONENT_DIRS})
  add_custom_command(TARGET build_summary
    COMMAND ${CMAKE_COMMAND} -E echo "  - ${COMPONENT}"
  )
endforeach()