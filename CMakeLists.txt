# CMakeLists.txt

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
# NOTE: This is generally not needed if components are in the standard 'components' directory.
# Removed pstar_pin_validator as it should be found automatically.
# set(EXTRA_COMPONENT_DIRS
# )

# Include ESP-IDF project CMake and define the project.
# The ESP-IDF build system handles core component includes automatically based on REQUIRES.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Project-Star)

# Print selected Kconfig options for quick reference
message(STATUS "Configuration Summary:")
message(STATUS "  Pin Validator Enabled:     ${CONFIG_PSTAR_KCONFIG_PIN_VALIDATOR_ENABLED}")

# Append project-specific include directories.
# Component includes are handled automatically by the build system via REQUIRES.
idf_build_set_property(INCLUDE_DIRECTORIES
  "${CMAKE_SOURCE_DIR}/main/include"
  APPEND
)

# Define a custom target to print a build summary.
# Simplified: Removed component listing as the previous method was inaccurate.
add_custom_target(build_summary ALL
  COMMAND ${CMAKE_COMMAND} -E echo "Project-Star: Build completed successfully"
  VERBATIM
  DEPENDS app
)

# Removed inaccurate component listing loop