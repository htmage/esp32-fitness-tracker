# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/components/LovyanGFX)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/components/max30102/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/managed_components/espressif_mpu6050/include)
project(fitness-tracker)