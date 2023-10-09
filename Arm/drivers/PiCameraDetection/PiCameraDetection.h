#include "opencv2/opencv.hpp"
#include <sys/time.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <thread>
#include <cmath>
#include <condition_variable>

using namespace std;
using namespace cv;




class PiCameraDetection {
	public:
		PiCameraDetection();	
		void init_window();
		void populate_window();
		void detect_coordinates();
		cv::Point2f get_centroid();
		void load_calibration(const std::string& filename);
		void save_calibration(const std::string& filename);
		void close();
		
		
	
	private:
		static void on_low_H_thresh_trackbar(int, void *);
		static void on_high_H_thresh_trackbar(int, void *);
		static void on_low_S_thresh_trackbar(int, void *);
		static void on_high_S_thresh_trackbar(int, void *);
		static void on_low_V_thresh_trackbar(int, void *);
		static void on_high_V_thresh_trackbar(int, void *);
		static void on_low_R_thresh_trackbar(int, void *);
		static void on_high_R_thresh_trackbar(int, void *);
		static void on_low_G_thresh_trackbar(int, void *);
		static void on_high_G_thresh_trackbar(int, void *);
		static void on_low_B_thresh_trackbar(int, void *);
		static void on_high_B_thresh_trackbar(int, void *);
		static void on_morph_size(int, void*);
		static void on_morph_operator(int, void*);
		static void on_morph_elem(int, void*);
		static void on_thresh(int, void*);
		const String window_capture_name;
		const String window_detection_name;
		int morph_elem;
		int morph_size;
		int morph_operator;
		int thresh;
		const int max_operator;
		const int max_elem;
		const int max_kernel_size;
		const int max_thresh;
		const int max_value_H;
		const int max_value;
		int low_H, low_S, low_V;
		int high_R, high_G, high_B;
		int low_R, low_G, low_B;
		int high_H, high_S, high_V;
		Scalar color;
		Scalar color2;
		cv::Mat frame, frame_HSV, frame_threshold, canny_output;
		vector<vector<Point>> contours;
		cv::VideoCapture cap;
		cv::Point2f mc;
};


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




