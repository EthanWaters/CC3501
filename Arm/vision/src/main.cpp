#include "PiCameraDetection.h"
#include "Client.h"


#define NUM_OBJECTS 4
#define NUM_ANGLES NUM_OBJECTS-2

// Shared data structures
float angles[NUM_ANGLES];
float previous_angles[NUM_ANGLES];
float angle_differences[NUM_ANGLES];
cv::Point2f centroids[NUM_OBJECTS];
bool is_display;

int main(int argc, char* argv[]) {
    
    const char* defaultIp = "192.168.0.252";
    const char* defaultPort = "54321";

    const char* ip = (argc > 1) ? argv[1] : defaultIp;
    const char* port = (argc > 2) ? argv[2] : defaultPort;

    
    // Create instances of PiCameraDetection to find arm segments
    PiCameraDetection detector_1;
    PiCameraDetection detector_2;
    PiCameraDetection detector_3;
    PiCameraDetection detector_4;
    
    detector_1.init_capture();
    detector_2.init_capture();
    detector_3.init_capture();
    detector_4.init_capture();
    
    // intialise client class to send data over socket
    Client client(ip, port);
    client.init();
       
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
            detector_1.close();
            detector_2.close();
            detector_3.close();
            detector_4.close();
            return 0;
            
        } else if (command == "start") {
            
            while (true) {
                std::string command;
                std::cin >> command;
                
                for (int i = 0; i < NUM_ANGLES; i++) {
                    previous_angles[i] = angles[i];
                }
                if (command == "exit") {            
                    // Stop object detection threads
                    detector_1.close();
                    detector_2.close();
                    detector_3.close();
                    detector_4.close();
                    return 0;
            
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
                
                get_arm_angles(angles, centroids, NUM_ANGLES);
                get_angle_differences(angle_differences, angles, previous_angles, NUM_ANGLES);
                client.send(angle_differences);
                
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
        
        get_arm_angles(angles, centroids, NUM_ANGLES);
        
           
        if(is_display == true){
            detector_1.populate_window();
            detector_2.populate_window();
            detector_3.populate_window();
            detector_4.populate_window();
        }
       
       
	}
    

    
    return 0;
}



float get_joint_angle(cv::Point2f centroid1, cv::Point2f centroid2, cv::Point2f centroid_ref){
	
	// scale values to be with reference to centroid_ref
	
	cv::Point2f vector1 = centroid1 - centroid_ref;
    cv::Point2f vector2 = centroid2 - centroid_ref;

    // Calculate the angle between the vectors using atan2
    double angle1 = atan2(vector1.y, vector1.x);
    double angle2 = atan2(vector2.y, vector2.x);

    // Calculate the angle difference (angle2 - angle1)
    float angleDifference = angle2 - angle1;

    // Ensure the angle is within the range of -pi to pi
    if (angleDifference > CV_PI) {
        angleDifference -= 2 * CV_PI;
    } else if (angleDifference <= -CV_PI) {
        angleDifference += 2 * CV_PI;
    }

    return angleDifference;
	
}


void get_arm_angles(float angles[], cv::Point2f centroids[], int arrayLength ){
	//(EXAMPLE)
	//Centroid 0: Shoulder 
	//Centroid 1: Elbow 
	//Centroid 2: Wrist
	//Centroid 3: Knuckle 
	
	for(int i=0; i<arrayLength; i++){
		angles[i] = get_joint_angle(centroids[i], centroids[i+2], centroids[i+1]);
	}
	
}


void get_angle_differences(float angle_differences[], float angles[], float previous_angles[], int arrayLength){
	//(EXAMPLE)
	//Angle 0: Elbow 
	//Angle 1: Wrist
	
	for(int i=0; i<arrayLength; i++){
		angle_differences[i] = previous_angles[i] - angles[i];
	}
}




