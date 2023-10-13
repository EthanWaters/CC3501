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
		PiCameraDetection(std::shared_ptr<cv::VideoCapture> cap);	
		void init_window();
		void populate_window();
		int detect_coordinates();
		cv::Point2f get_centroid();
		std::string get_centroid_s();
		int load_calibration(const std::string& filename);
		int save_calibration(const std::string& filename);
		void close();
		int init_capture();
		vector<vector<Point>> get_contour();
		
	
	private:
		static void on_low_H_thresh_trackbar(int, void*);
		static void on_high_H_thresh_trackbar(int, void*);
		static void on_low_S_thresh_trackbar(int, void*);
		static void on_high_S_thresh_trackbar(int, void*);
		static void on_low_V_thresh_trackbar(int, void*);
		static void on_high_V_thresh_trackbar(int, void*);
		static void on_low_R_thresh_trackbar(int, void*);
		static void on_high_R_thresh_trackbar(int, void*);
		static void on_low_G_thresh_trackbar(int, void*);
		static void on_high_G_thresh_trackbar(int, void*);
		static void on_low_B_thresh_trackbar(int, void*);
		static void on_high_B_thresh_trackbar(int, void*);
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
		cv::Mat frame_HSV, frame_threshold, canny_output;
		vector<vector<Point>> contours;
		cv::Mat frame;
		std::shared_ptr<cv::VideoCapture> cap;
		cv::Point2f mc;
		
};

float get_joint_angle(cv::Point2f centroid1, cv::Point2f centroid2, cv::Point2f centroid_ref);
void get_arm_angles(float angles[], cv::Point2f centroids[], int arrayLength );
void get_angle_differences(float angle_differences[], float angles[], float previous_angles[], int arrayLength);
