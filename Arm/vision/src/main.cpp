#include "PiCameraDetection.h"
#include "Client.h"
#include <iostream>
#include <thread>
#include <atomic>

#define NUM_OBJECTS 4
#define NUM_ANGLES NUM_OBJECTS-2

// Shared data structures
float angles[NUM_ANGLES];
float previous_angles[NUM_ANGLES];
float angle_differences[NUM_ANGLES];
cv::Point2f centroids[NUM_OBJECTS];
bool is_display;
std::string data_send;
std::atomic<bool> shouldExit(false);
std::string command;
std::string parameter;
std::mutex commandMutex;

void inputThread() {
    while (!shouldExit) {
        std::string inputCommand;
        std::cout << "Enter a command: ";
        std::getline(std::cin, inputCommand);

        // Lock the mutex before updating the command
        std::lock_guard<std::mutex> lock(commandMutex);
        std::istringstream iss(inputCommand);
        iss >> command >> parameter;

        if (command == "exit") {
            shouldExit = true;
            break;
        }
    }
}



template <typename T>
std::string array_to_string(const T& data) {
    std::ostringstream oss;
    size_t data_size = sizeof(data) / sizeof(data[0]);
    for (size_t i = 0; i < data_size; ++i) {
        oss << data[i];
        if (i < data_size - 1) {
            oss << ", ";
        }
    }
    return oss.str();
}


int main(int argc, char* argv[]) {
    
    const char* defaultIp = "192.168.0.185";
    const char* defaultPort = "54321";
    
    const char* ip = (argc > 1) ? argv[1] : defaultIp;
    const char* port = (argc > 2) ? argv[2] : defaultPort;

    std::cout << ip << std::endl;
    std::cout << port << std::endl;
    
    // Open the video camera.
    std::string pipeline = "libcamerasrc"
        " ! video/x-raw, width=800, height=600" // camera needs to capture at a higher resolution
        " ! videoconvert"
        " ! videoscale"
        " ! video/x-raw, width=400, height=300" // can downsample the image after capturing
        " ! videoflip method=rotate-180" // remove this line if the image is upside-down
        " ! appsink drop=true max_buffers=2";
    std::shared_ptr<cv::VideoCapture> sharedCapture = std::make_shared<cv::VideoCapture>(pipeline, cv::CAP_GSTREAMER);
    if(!sharedCapture->isOpened()) {
        printf("Could not open camera.\n");
        return 1;
    }
    
    
    // Create instances of PiCameraDetection to find arm segments
    PiCameraDetection detector_1(sharedCapture); //Centroid 0: Shoulder 
    PiCameraDetection detector_2(sharedCapture); //Centroid 1: Elbow 
    PiCameraDetection detector_3(sharedCapture); //Centroid 2: Wrist
    PiCameraDetection detector_4(sharedCapture); //Centroid 3: Knuckle 
    
    
    // intialise client class to send data over socket
    Client client(ip, port);
    client.init();
    
    std::thread input(inputThread);
    while (!shouldExit) {
        if (command == "init_window") {
            std::cout << "Initialising window...." << std::endl;
            detector_1.init_window();
            is_display = true;
            
         } else if (command == "load") {
            if(parameter.find('1') != std::string::npos){
                detector_1.load_calibration(parameter);
            } else if parameter.find('2') != std::string::npos){
                detector_2.load_calibration(parameter);
            } else if parameter.find('3') != std::string::npos){
                detector_3.load_calibration(parameter);
            } else if parameter.find('4') != std::string::npos){
                detector_4.load_calibration(parameter);
            } 
            
            std::cout << "Threshold loaded " << parameter << std::endl;
        } else if (command == "save") {
            if(parameter.find('1') != std::string::npos){
                detector_1.save_calibration(parameter);
            } else if parameter.find('2') != std::string::npos){
                detector_2.save_calibration(parameter);
            } else if parameter.find('3') != std::string::npos){
                detector_3.save_calibration(parameter);
            } else if parameter.find('4') != std::string::npos){
                detector_4.save_calibration(parameter);
            } 
            
            std::cout << "Threshold saved " << parameter << std::endl;
        } else if command == "start") {
          break   
        }
        command.clear();
        
        // Start object detection threads
        detector_1.detect_coordinates();
        detector_2.detect_coordinates();
        detector_3.detect_coordinates();
        detector_4.detect_coordinates();
        
        // Add centroids to the vector
        centroids[0] = detector_1.get_centroid();
        centroids[1] = detector_2.get_centroid();
        centroids[2] = detector_3.get_centroid();
        centroids[3] = detector_4.get_centroid();
        
        get_arm_angles(angles, centroids, NUM_ANGLES);
        data_send = array_to_string(angles);
        client.send(data_send);
           
        if(is_display == true){
            detector_1.populate_window();
            detector_2.populate_window();
            detector_3.populate_window();
            detector_4.populate_window();
        }
    }
    
    
    
    while (!shouldExit) {
        
        //set previous angle
        for (int i = 0; i < NUM_ANGLES; i++) {
            previous_angles[i] = angles[i];
        } 
        
        if (!command.empty()){
             (command == "close_window") {  
                is_display = false;
            
            } else if (command == "load") {
                if(parameter.find('1') != std::string::npos){
                    detector_1.load_calibration(parameter);
                } else if parameter.find('2') != std::string::npos){
                    detector_2.load_calibration(parameter);
                } else if parameter.find('3') != std::string::npos){
                    detector_3.load_calibration(parameter);
                } else if parameter.find('4') != std::string::npos){
                    detector_4.load_calibration(parameter);
                } 
                
                std::cout << "Threshold loaded " << parameter << std::endl;
             } else if (command == "save") {
                if(parameter.find('1') != std::string::npos){
                    detector_1.save_calibration(parameter);
                } else if parameter.find('2') != std::string::npos){
                    detector_2.save_calibration(parameter);
                } else if parameter.find('3') != std::string::npos){
                    detector_3.save_calibration(parameter);
                } else if parameter.find('4') != std::string::npos){
                    detector_4.save_calibration(parameter);
                } 
                
                std::cout << "Threshold saved " << parameter << std::endl;
                
            }   
        }
        command.clear();
           
       
         // Start object detection threads
        detector_1.detect_coordinates();
        detector_2.detect_coordinates();
        detector_3.detect_coordinates();
        detector_4.detect_coordinates();
        
        // Add centroids to the vector
        centroids[0] = detector_1.get_centroid();
        centroids[1] = detector_2.get_centroid();
        centroids[2] = detector_3.get_centroid();
        centroids[3] = detector_4.get_centroid();
        
        get_arm_angles(angles, centroids, NUM_ANGLES);
        get_angle_differences(angle_differences, angles, previous_angles, NUM_ANGLES);
        data_send = array_to_string(angle_differences);
        client.send(data_send);
        
        if(is_display == true){
            detector_1.populate_window();
            detector_2.populate_window();
            detector_3.populate_window();
            detector_4.populate_window();
        }
    }
    
    input.join(); // Wait for the input thread to finish
    return 0;
}





