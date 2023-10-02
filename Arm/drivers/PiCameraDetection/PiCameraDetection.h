#include "opencv2/opencv.hpp"
#include <sys/time.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <thread>
#include <condition_variable>

using namespace std;
using namespace cv;




class PiCameraDetection {
	public:
		PiCameraDetection();	
		void init_window();
		void populate_window();
		void threshold();
		void find_centroid_coords();
		void load_calibration(const std::string& filename);
		void central_thread_function(std::vector<PiCameraDetection>& detectors);
		void save_calibration(const std::string& filename);
		void close();
		std::mutex mtx;
		std::atomic<bool> stop_threads;
		std::vector<cv::Point2f> mc;
		
	
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
		vector<vector<Point> > contours;
		cv::VideoCapture cap;
		std::condition_variable cv;
		std::thread detection_thread;
        std::atomic<bool> stop_threads;
};
