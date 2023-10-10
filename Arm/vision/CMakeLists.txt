cmake_minimum_required(VERSION 3.12)
project(vision)
set(CMAKE_CXX_STANDARD 20)
set(COMPILE_DEFINITIONS -O2 -Wall)
find_package(OpenCV REQUIRED)

set(ProjectDir /home/esl/Documents/CC3501/Arm)
#cmake_path(GET CMAKE_CURRENT_SOURCE_DIR PARENT_PATH ProjectDir)
message("ProjectDir is ${ProjectDir}")

#Add the include directories for PiCameraDetection and Client
include_directories(${ProjectDir}/drivers/PiCameraDetection)
include_directories(${ProjectDir}/drivers/Client)

add_executable(vision 
    src/main.cpp
    ${ProjectDir}/drivers/Client/Client.cpp
    ${ProjectDir}/drivers/PiCameraDetection/PiCameraDetection.cpp
)

target_link_libraries(vision ${OpenCV_LIBS})



