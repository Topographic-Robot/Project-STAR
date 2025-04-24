# CMakeLists.txt

cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components ${CMAKE_CURRENT_SOURCE_DIR}/pstar_examples)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Project-Star)
