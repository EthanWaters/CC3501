
cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)


# Pull in Raspberry Pi Pico SDK (must be before project)

#set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
project(CAN_TEST C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

set(ProjectDir C:/Users/elois/Documents/CC3501/Arm)
#cmake_path(GET CMAKE_CURRENT_SOURCE_DIR PARENT_PATH ProjectDir)
message("ProjectDir is ${ProjectDir}")

# Add MCP2515 Lib
include_directories(${ProjectDir}/drivers/mcp2515)


# Add executable. Default name is the project name, version 0.1
add_executable(CAN_TEST
    src/CAN_TEST.cpp
    ${ProjectDir}/drivers/mcp2515/mcp2515.cpp   
)

#pico_set_program_name(CAN_TEST "CAN_TEST")
#pico_set_program_version(CAN_TEST "0.1")

#pico_enable_stdio_uart(CAN_TEST 0)
#pico_enable_stdio_usb(CAN_TEST 1)

# Add any user requested libraries
target_link_libraries(CAN_TEST
    pico_stdlib
    hardware_spi
)

pico_add_extra_outputs(CAN_TEST)