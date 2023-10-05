#include "PiCameraDetection/PiCameraDetection.h"
#include "Client/Client.h"


#define NUM_OBJECTS 4;
bool is_display;

// Shared data structures
volatile float angles[NUM_OBJECTS - 2];
volatile float previous_angles[NUM_OBJECTS - 2];
float angle_differences[NUM_OBJECTS - 2];
<cv::Point2f> centroids[NUM_OBJECTS];


int main() {
    
    // Create instances of PiCameraDetection to find arm segments
    PiCameraDetection detector_1;
    PiCameraDetection detector_2;
    PiCameraDetection detector_3;
    PiCameraDetection detector_4;
    
    // intialise client class to send data over socket
    Client client("192.168.0.252", "54321");
    
       
     while (true) {
        std::string command;
        std::cin >> command;

        if (command == "init_window") {
            // Load thresholds for all detectors
            detector_1.init_window();
            detector_2.init_window();
            detector_3.init_window();
            detector_4.init_window();
            is_display = true;
            
        } else if (command == "close_window") {  
            is_display = false;
            
        } else if (command == "load_thresholds") {
            // Load thresholds for all detectors
            detector_1.load_calibration("detector_1_calibration");
            detector_2.load_calibration("detector_2_calibration");
            detector_3.load_calibration("detector_3_calibration");
            detector_4.load_calibration("detector_4_calibration");
            

            std::cout << "Thresholds loaded." << std::endl;
        } else if (command == "save_thresholds") {
            // Save thresholds for all detectors
            detector_1.save_calibration("detector_1_calibration");
            detector_2.save_calibration("detector_2_calibration");
            detector_3.save_calibration("detector_3_calibration");
            detector_4.save_calibration("detector_4_calibration");
            
            std::cout << "Thresholds saved." << std::endl;
       
            
        } else if (command == "exit") {
            
            // Stop object detection threads
            detector_1.stop_detection();
            detector_2.stop_detection();
            detector_3.stop_detection();
            detector_4.stop_detection();
            return 0
            
        } else if (command == "start") {
            
            previous_angle = angles
            while (true) {
                std::string command;
                std::cin >> command;
                
                previous_angles = angles;
                if (command == "exit") {            
                    // Stop object detection threads
                    detector_1.stop_detection();
                    detector_2.stop_detection();
                    detector_3.stop_detection();
                    detector_4.stop_detection();
                    return 0
            
                }
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
                
                get_arm_angles(angles, centroids);
                get_angle_differences(angle_differences, angles, previous_angles);
                client.send(angle_differences)
                
            }
        }
            
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
        
        get_arm_angles(angles, centroids);
        
           
        if(is_display == true){
            detector_1.populate_window();
            detector_2.populate_window();
            detector_3.populate_window();
            detector_4.populate_window();
        }
       
       
	}
    

    
    return 0;
}
