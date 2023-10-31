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

    project(CAN_TEST C CXX ASM)
    
    # Initialise the Raspberry Pi Pico SDK
    pico_sdk_init()
    
    include_directories(${ProjectDir}/drivers/mcp2515)
    
    add_executable(CAN_TEST
        src/CAN_TEST.cpp
        ${ProjectDir}/drivers/mcp2515/mcp2515.cpp   
    )
    target_sources(CAN_TEST 
        PUBLIC
        src/CAN_TEST.cpp
        ${ProjectDir}/drivers/mcp2515/mcp2515.cpp   
    )
    target_include_directories(CAN_TEST
        PUBLIC 
        src/CAN_TEST.cpp
    )

    pico_set_program_name(CAN_TEST "CAN_TEST")
    pico_set_program_version(CAN_TEST "0.2")

    # select UART for standard IO
    pico_enable_stdio_uart(CAN_TEST 1)
    pico_enable_stdio_usb(CAN_TEST 0)


    # Add the standard library to the build
    target_link_libraries(CAN_TEST
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

    pico_add_extra_outputs(CAN_TEST)

else()

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 20)
 

    # We are building natively, so create the test harness instead
    project(CAN_TEST CXX)

   
    include_directories(${ProjectDir}/drivers/mcp2515/)
    
    add_executable(CAN_TEST
        src/CAN_TEST.cpp
        ${ProjectDir}/drivers/mcp2515/mcp2515.cpp   
    )
    target_sources(CAN_TEST 
        PUBLIC
        src/CAN_TEST.cpp
        ${ProjectDir}/drivers/mcp2515/mcp2515.cpp 
        tests/mocks/pico/stdlib.cpp
        tests/mocks/pico/time.cpp
        tests/mocks/hardware/gpio.cpp
        tests/mocks/hardware/pio.cpp
        tests/mocks/ws2812.cpp  
    )
    target_include_directories(CAN_TEST
        PUBLIC 
        src/CAN_TEST.cpp
        tests/
        tests/mocks/
    )
    target_compile_definitions(CAN_TEST 
        PUBLIC
        TEST_HARNESS=1
    )

endif()



