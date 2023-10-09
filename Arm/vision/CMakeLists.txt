cmake_minimum_required(VERSION 3.12)
project(vision)
set(CMAKE_CXX_STANDARD 20)
set(COMPILE_DEFINITIONS -O2 -Wall)
find_package(OpenCV REQUIRED)

set(Client /home/esl/Documents/CC3501/Arm/drivers/Client)
set(PiCameraDetection /home/esl/Documents/CC3501/Arm/drivers/PiCameraDetection)

# Add the include directories for PiCameraDetection and Client
include_directories(/home/esl/Documents/CC3501/Arm/drivers/PiCameraDetection)
include_directories(/home/esl/Documents/CC3501/Arm/drivers/Client)

add_subdirectory(${PiCameraDetection} PiCameraDetection)
add_subdirectory(${Client} Client)

add_executable(vision src/main.cpp)
target_include_directories(Client PUBLIC /home/esl/Documents/CC3501/Arm/drivers/Client)
target_include_directories(PiCameraDetection PUBLIC /home/esl/Documents/CC3501/Arm/drivers/PiCameraDetection)

target_link_libraries(vision ${OpenCV_LIBS})
target_link_libraries(vision Client)
target_link_libraries(vision PiCameraDetection)



