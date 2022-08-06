# What CMake to start at
cmake_minimum_required(VERSION 3.12)
 
set(PICO_BOARD pico_w)
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FreeRTOS-Kernel)


include(pico_sdk_import.cmake)
include(FreeRTOS_Kernel_import.cmake)
 
project(EnviroMqtt VERSION 1.0.0)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
 
add_subdirectory(thirdparty/fmt)

file(GLOB_RECURSE pico_sources CONFIGURE_DEPENDS
    src/*.cpp src/*.c thirdparty/BME68x-Sensor-API/bme68x.c)
file(GLOB_RECURSE pico_headers CONFIGURE_DEPENDS src/*.hpp src/*.h)

add_executable(EnviroMqtt ${pico_sources} ${pico_headers})
target_include_directories(EnviroMqtt PUBLIC src)
target_include_directories(EnviroMqtt PUBLIC thirdparty/BME68x-Sensor-API)
target_link_libraries(EnviroMqtt PRIVATE 
    pico_stdlib
    hardware_i2c
    pico_cyw43_arch_lwip_sys_freertos
    pico_lwip_iperf
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
    fmt)
 
pico_sdk_init()
pico_enable_stdio_usb(EnviroMqtt 1)
pico_enable_stdio_uart(EnviroMqtt 1)
pico_add_extra_outputs(EnviroMqtt)
