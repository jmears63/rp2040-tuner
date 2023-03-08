cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(tuner_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_subdirectory(./lib/Config)
add_subdirectory(./lib/LCD)
add_subdirectory(./lib/Fonts)
add_subdirectory(./lib/GUI)
add_subdirectory(./lib/QMI8658)

include_directories(./examples)
include_directories(./lib/Config)
include_directories(./lib/Fonts)
include_directories(./lib/GUI)
include_directories(./lib/LCD)
include_directories(./lib/QMI8658)

add_executable(tuner
  tuner.c imu.c
)

# pico_enable_stdio_usb(tuner 1)
pico_enable_stdio_uart(tuner 1)

pico_add_extra_outputs(tuner)

target_link_libraries(tuner LCD QMI8658 GUI Fonts Config pico_stdlib hardware_spi hardware_i2c)
