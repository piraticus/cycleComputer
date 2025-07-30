cmake_minimum_required(VERSION 3.12)

# Name project
SET(Project CycleComputer)
SET(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel)
SET(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)
SET(PICO_BOARD pico_w)
SET(CMKAKE_BUILD_TYPE Debug)
SET(CMAKE_EXPORT_COMPILE_COMMANDS 1)


# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(${Project} C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()
# FREERTOS: include FreeRTOS Kernel libraries
include(FreeRTOS_Kernel_import.cmake)
# Set any variables required for importing libraries

# Add subdirectories
add_subdirectory(source)


