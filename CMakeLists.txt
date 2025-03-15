cmake_minimum_required(VERSION 3.16)

# Set component dependencies order
set(EXTRA_COMPONENT_DIRS 
  "${CMAKE_SOURCE_DIR}/components/pstar_bus"
  "${CMAKE_SOURCE_DIR}/components/pstar_storage_hal"
  "${CMAKE_SOURCE_DIR}/components/pstar_logging"
  "${CMAKE_SOURCE_DIR}/main/pstar_managers"
)

# Add local components
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/components")

# add_compile_definitions(USE_OV7670_XCLK_GPIO_27)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Project-Star)

# Add component include directories to the global include path
idf_build_set_property(INCLUDE_DIRECTORIES 
  "${CMAKE_SOURCE_DIR}/components/pstar_logging/include"
  "${CMAKE_SOURCE_DIR}/components/pstar_storage_hal/sd_card_hal/include"
  "${CMAKE_SOURCE_DIR}/components/pstar_bus/include"
  "${CMAKE_SOURCE_DIR}/main/include"
  "${CMAKE_SOURCE_DIR}/main/pstar_managers/include"
  APPEND)

