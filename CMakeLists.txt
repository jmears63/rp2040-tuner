cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(tuner_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(tuner
  main.c
)
# pico_enable_stdio_usb(tuner 1)
pico_enable_stdio_uart(tuner 1)
pico_add_extra_outputs(tuner)
target_link_libraries(tuner pico_stdlib)
