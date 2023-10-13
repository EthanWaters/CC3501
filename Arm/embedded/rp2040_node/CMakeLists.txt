cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 20)
project(rp2040_node)
set(COMPILE_DEFINITIONS -O2 -Wall)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

set(ProjectDir /home/esl/Documents/CC3501/Arm)
#cmake_path(GET CMAKE_CURRENT_SOURCE_DIR PARENT_PATH ProjectDir)
message("ProjectDir is ${ProjectDir}")

include_directories(${ProjectDir}/drivers/mcp2515)


add_executable(vision 
    src/rp2040_node.cpp
    ${ProjectDir}/drivers/mcp2515/mcp2515.cpp
    
)

pico_set_program_name(rp2040_node "rp2040_node")
pico_set_program_version(rp2040_node "0.1")

pico_enable_stdio_uart(rp2040_node 0)
pico_enable_stdio_usb(rp2040_node 1)

# Add any user requested libraries
target_link_libraries(rp2040_node
    pico_stdlib
    hardware_spi
)

pico_add_extra_outputs(rp2040_node)