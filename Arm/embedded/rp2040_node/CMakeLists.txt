cmake_minimum_required(VERSION 3.13)


# Detect if the active kit is an ARM cross-compiler
if(CMAKE_CXX_COMPILER MATCHES "arm-none-eabi")
    message(STATUS "Detected that the current kit is a cross-compiler.")
    set(CrossCompiling 1)
else()
    message(STATUS "Detected that the current kit is a host compiler. Building the test harness.")
endif()

cmake_path(GET CMAKE_CURRENT_SOURCE_DIR PARENT_PATH ProjectDir)
get_filename_component(ProjectDir "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)
get_filename_component(ProjectDir "${ProjectDir}" DIRECTORY)
message("ProjectDir is ${ProjectDir}")
# Detect if the active kit is an ARM cross-compiler
if(CrossCompiling)
    # Yes, build for the RP2040

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions")
    # Specify an increased oscillator startup delay to work around a potential hardware issue where the 
    # clock is slow to boot.
    add_compile_definitions(PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)

    # Pull in Raspberry Pi Pico SDK (must be before project)
    include(pico_sdk_import.cmake)

    project(rp2040_node C CXX ASM)
    
    # Initialise the Raspberry Pi Pico SDK
    pico_sdk_init()
    
    include_directories(${ProjectDir}/drivers/mcp2515)
    include_directories(${ProjectDir}/drivers/LSM9DS1_driver)

    add_executable(rp2040_node
        src/rp2040_node.cpp
        ${ProjectDir}/drivers/mcp2515/mcp2515.cpp   
        ${ProjectDir}/drivers/LSM9DS1_driver/LSM9DS1_driver.cpp   
    )
    target_sources(rp2040_node 
        PUBLIC
        src/rp2040_node.cpp
        ${ProjectDir}/drivers/mcp2515/mcp2515.cpp   
        ${ProjectDir}/drivers/LSM9DS1_driver/LSM9DS1_driver.cpp  
    )
    target_include_directories(rp2040_node
        PUBLIC 
        src/rp2040_node.cpp
    )

    pico_set_program_name(rp2040_node "rp2040_node")
    pico_set_program_version(rp2040_node "0.2")

    # select UART for standard IO
    pico_enable_stdio_uart(rp2040_node 1)
    pico_enable_stdio_usb(rp2040_node 0)


    # Add the standard library to the build
    target_link_libraries(rp2040_node
        pico_stdlib
        hardware_spi
        hardware_i2c
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_timer
        hardware_watchdog
        hardware_clocks
        hardware_pwm
        hardware_adc

    )

    pico_add_extra_outputs(rp2040_node)

else()

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 20)
 

    # We are building natively, so create the test harness instead
    project(rp2040_node CXX)

   
    include_directories(${ProjectDir}/drivers/mcp2515/)
    
    add_executable(rp2040_node
        src/rp2040_node.cpp
        ${ProjectDir}/drivers/mcp2515/mcp2515.cpp   
    )
    target_sources(rp2040_node 
        PUBLIC
        src/rp2040_node.cpp
        ${ProjectDir}/drivers/mcp2515/mcp2515.cpp 
        tests/mocks/pico/stdlib.cpp
        tests/mocks/pico/time.cpp
        tests/mocks/hardware/gpio.cpp
        tests/mocks/hardware/pio.cpp
        tests/mocks/ws2812.cpp  
    )
    target_include_directories(rp2040_node
        PUBLIC 
        src/rp2040_node.cpp
        tests/
        tests/mocks/
    )
    target_compile_definitions(rp2040_node 
        PUBLIC
        TEST_HARNESS=1
    )

endif()



