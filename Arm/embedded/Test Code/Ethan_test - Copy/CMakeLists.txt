cmake_minimum_required(VERSION 3.13)

# Detect if the active kit is an ARM cross-compiler
if(CMAKE_CXX_COMPILER MATCHES "arm-none-eabi")
    message(STATUS "Detected that the current kit is a cross-compiler.")
    set(CrossCompiling 1)
else()
    message(STATUS "Detected that the current kit is a host compiler. Building the test harness.")
endif()



# file(GLOB_RECURSE files "*.cpp")

# add_library(Fusion ${files})

# if(UNIX AND NOT APPLE)
#     target_link_libraries(Fusion m) # link math library for Linux
# endif()

# Detect if the active kit is an ARM cross-compiler
if(CrossCompiling)
    # Yes, build for the RP2040

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)

    # Specify an increased oscillator startup delay to work around a potential hardware issue where the 
    # clock is slow to boot.
    add_compile_definitions(PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)

    # Pull in Raspberry Pi Pico SDK (must be before project)
    include(pico_sdk_import.cmake)

    project(Ethan_test C CXX ASM)
    
    # Initialise the Raspberry Pi Pico SDK
    pico_sdk_init()

    add_executable(Ethan_test)
    target_sources(Ethan_test 
        PUBLIC
        src/main.cpp
        src/drivers/LSM9DS1.h
        src/drivers/LSM9DS1.cpp
        src/Sensor_Board.h
        src/drivers/MPU9250_driver.h
        src/drivers/MPU9250_driver.cpp
        src/drivers/MPU6050.h
        src/drivers/MPU6050.cpp
        src/SensorFusion/FusionArhs.h
        src/SensorFusion/FusionArhs.cpp
        src/SensorFusion/FusionAxes.h
        src/SensorFusion/FusionCalibration.h
        src/SensorFusion/FusionCompass.cpp
        src/SensorFusion/FusionCompass.h
        src/SensorFusion/FusionMath.h
        src/SensorFusion/FusionOffset.cpp
        src/SensorFusion/FusionOffset.h
        src/SensorFusion/Fusion.h
    )
    target_include_directories(Ethan_test
        PUBLIC 
        src/
        src/SensorFusion ##### ADDDEDDD #######
    )

    pico_set_program_name(Ethan_test "Ethan_test")
    pico_set_program_version(Ethan_test "1.0")

    # select UART for standard IO
    pico_enable_stdio_usb(Ethan_test 1)
    #pico_enable_stdio_uart(labs 1)
    #pico_enable_stdio_usb(labs 0)


    # Add the standard library to the build
    target_link_libraries(Ethan_test
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
        hardware_spi
    )

    pico_add_extra_outputs(Ethan_test)

else()

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 20)

    # We are building natively, so create the test harness instead
    project(Ethan_test CXX)

    add_executable(Ethan_test)
    target_sources(Ethan_test 
        PUBLIC
        src/main.cpp
        src/drivers/logging/logging.cpp
        tests/mocks/pico/stdlib.cpp
        tests/mocks/pico/time.cpp
        tests/mocks/hardware/gpio.cpp
        tests/mocks/hardware/pio.cpp
        tests/mocks/ws2812.cpp
    )
    target_include_directories(Ethan_test
        PUBLIC 
        src/
      
    )
    target_compile_definitions(Ethan_test
        PUBLIC
        TEST_HARNESS=1
    )

endif()

target_compile_definitions(Ethan_test
    PUBLIC
    LOG_DRIVER_STYLE=${LogDriverImplementation}
)
