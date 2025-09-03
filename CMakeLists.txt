cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(drone C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()
add_executable(drone drone.c)

target_link_libraries(drone pico_stdlib hardware_pwm)
pico_enable_stdio_usb(drone 1)
pico_enable_stdio_uart(drone 0)
pico_add_extra_outputs(drone)
