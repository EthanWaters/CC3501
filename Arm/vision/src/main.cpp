#include "PiCameraDetection/PiCameraDetection.h"

bool is_display;

int main() {
    
    // Create instances of PiCameraDetection
    PiCameraDetection detector_1();
    PiCameraDetection detector_2();
    PiCameraDetection detector_3();
    PiCameraDetection detector_4();

       
     while (true) {
        std::string command;
        std::cin >> command;

        if (command == "exit") {
            stop_threads = true;
            break;
        } else if (command == "init_window") {
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
        } else if (command == "start_detection") {
            
            // Start object detection threads
            detector_1.start_detection();
            detector_2.start_detection();
            detector_3.start_detection();
            detector_4.start_detection();
            
        } else if (command == "stop_detection") {
            
            // Stop object detection threads
            detector_1.stop_detection();
            detector_2.stop_detection();
            detector_3.stop_detection();
            detector_4.stop_detection();
            
        }
        
        if(is_display == true){
            detector_1.populate_window();
            detector_2.populate_window();
            detector_3.populate_window();
            detector_4.populate_window();
        }
        // Notify waiting threads (if any) that a command was processed
        cv.notify_all();
	}
    

    // Wait for the central thread to finish
    central_thread.join();
    
    return 0;
}
