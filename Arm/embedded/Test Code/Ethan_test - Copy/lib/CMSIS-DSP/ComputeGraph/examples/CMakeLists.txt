cmake_minimum_required (VERSION 3.14)
include(CMakePrintHelpers)

set(Python_FIND_REGISTRY "LAST")

find_package (Python COMPONENTS Interpreter)

function(sdf TARGET)
    if (DOT)
    add_custom_command(TARGET ${TARGET} PRE_BUILD 
        BYPRODUCTS ${CMAKE_CURRENT_SOURCE_DIR}/test.pdf
        COMMAND ${DOT} -Tpdf -o ${CMAKE_CURRENT_SOURCE_DIR}/test.pdf ${CMAKE_CURRENT_SOURCE_DIR}/test.dot
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test.dot
        VERBATIM
        )
    endif()

    add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/generated/scheduler.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test.dot
        COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/graph.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/graph.py
        VERBATIM
        )
    target_sources(${TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/generated/scheduler.cpp)
endfunction()

set(CGSTATICDIR ${CMAKE_CURRENT_SOURCE_DIR}/../cg/static)

set(DSP ${CMAKE_CURRENT_SOURCE_DIR}/../..)

function(add_sdf_dir TARGET)
    target_include_directories(${TARGET} PRIVATE ${CGSTATICDIR}/src)
    target_include_directories(${TARGET} PRIVATE ${CGSTATICDIR}/nodes/cpp)
    target_include_directories(${TARGET} PRIVATE ${CMSISCORE})
    target_include_directories(${TARGET} PRIVATE ${DSP}/Include)
endfunction()

project(Examples)


# Add DSP folder to module path
list(APPEND CMAKE_MODULE_PATH ${DSP})

add_subdirectory(${DSP}/Source bin_dsp)


add_subdirectory(example1 bin_example1)
add_subdirectory(example2 bin_example2)
add_subdirectory(example3 bin_example3)
add_subdirectory(example6 bin_example6)
add_subdirectory(example8 bin_example8)
