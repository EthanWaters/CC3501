#include <PiCameraDetection.h>


PiCameraDetection::PiCameraDetection()
{   
	static const String window_capture_name = "Camera";
	static const String window_detection_name = "Camera_Threshold";
	cv::Mat frame, frame_threshold;
	
    // Open the video camera.
    std::string pipeline = "libcamerasrc"
        " ! video/x-raw, width=800, height=600" // camera needs to capture at a higher resolution
        " ! videoconvert"
        " ! videoscale"
        " ! video/x-raw, width=400, height=300" // can downsample the image after capturing
        " ! videoflip method=rotate-180" // remove this line if the image is upside-down
        " ! appsink drop=true max_buffers=2";
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if(!cap.isOpened()) {
        printf("Could not open camera.\n");
        return 1;
    }
	
	
}	


void PiCameraDetection::init_window(){
	// Create the OpenCV window
    cv::namedWindow(window_capture_name, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(window_detection_name, cv::WINDOW_AUTOSIZE);
    
     // Trackbars to set thresholds for HSV values
     createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
     createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
     createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
     createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
     createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
     createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
     reateTrackbar("Low R", window_detection_name, &low_R, max_value_R, on_low_R_thresh_trackbar);
     createTrackbar("High R", window_detection_name, &high_R, max_value_R, on_high_R_thresh_trackbar);
     createTrackbar("Low G", window_detection_name, &low_G, max_value, on_low_G_thresh_trackbar);
     createTrackbar("High G", window_detection_name, &high_G, max_value, on_high_G_thresh_trackbar);
     createTrackbar("Low B", window_detection_name, &low_B, max_Balue, on_low_B_thresh_trackbar);
     createTrackbar("High B", window_detection_name, &high_V, max_Balue, on_high_B_thresh_trackbar);
    
    //Trackbars for Morphology options 
     createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", window_capture_name, &morph_operator, max_operator, on_morph_operator);
     createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse", window_capture_name, &morph_elem, max_elem, on_morph_elem);
     createTrackbar( "Kernel size:\n 2n +1", window_capture_name, &morph_size, max_kernel_size, on_morph_size );
    
}


void PiCameraDetection::threshold(){	
	if (!cap.read(frame)) {
            printf("Could not read a frame.\n");
            break;
        }
	cv::Mat HSV_threshold, RGB_threshold; 
	
	// Convert from BGR to HSV colorspace
	cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
	inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), HSV_threshold);
	inRange(frame, Scalar(low_R, low_G, low_B), Scalar(high_R, high_G, high_B), RGB_threshold);
	cv::bitwise_and(HSV_threshold, RGB_threshold, frame_threshold);
	
	// Perform morphology
	int operation = morph_operator + 2;
	Mat element = getStructuringElement(morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
	morphologyEx(frame_threshold, frame_threshold, operation, element);
}	

void PiCameraDetection::find_centroid_coords(std::vector<cv::Point2f>& mc){	
	cv::Moments mu = moments(frame_threshold);
	if(mu.m00 > 0){
		mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 ); 
		
	}
	cv.notify_one();
}

void PiCameraDetection::populate_window(){	
	vector<vector<Point> > contours;
	findContours(frame_threshold, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
	for( size_t i = 0; i< contours.size(); i++ ){
		drawContours( frame, contours, (int)i, color, 2 );
	 }
	
	//show frame
	circle(frame, mc, 4, color2, -1, 8, 0 );
	putText(frame, "Centre", mc,FONT_HERSHEY_COMPLEX, 1,color2, 2);
	cv::imshow(window_capture_name, frame);
	cv::imshow(window_detection_name, frame_threshold);
	cv::waitKey(1);	
	
}


void PiCameraDetection::central_thread_function(std::vector<PiCameraDetection>& detectors) {
    while (true) {
        std::string command;
        std::cin >> command;

        if (command == "exit") {
            stop_threads = true;
            break;
        } else if (command == "load_thresholds") {
            // Load thresholds for all detectors
            for (auto& detector : detectors) {
                detector.load_thresholds();
            }
            std::cout << "Thresholds loaded." << std::endl;
        } else if (command == "save_thresholds") {
            // Save thresholds for all detectors
            for (auto& detector : detectors) {
                detector.save_thresholds();
            }
            std::cout << "Thresholds saved." << std::endl;
        } else if (command == "start_detection") {
            // Start object detection threads
            for (auto& detector : detectors) {
                detector.start_detection();
            }
        } else if (command == "stop_detection") {
            // Stop object detection threads
            for (auto& detector : detectors) {
                detector.stop_detection();
            }
        }

        // Notify waiting threads (if any) that a command was processed
        cv.notify_all();
	}
}


// Function to save all thresholding variables to a YAML file
void PiCameraDetection::save_calibration(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    fs << "low_H" << low_H;
    fs << "high_H" << high_H;
    fs << "low_S" << low_S;
    fs << "high_S" << high_S;
    fs << "low_V" << low_V;
    fs << "high_V" << high_V;

    // Add RGB thresholding variables
    fs << "low_R" << low_R;
    fs << "high_R" << high_R;
    fs << "low_G" << low_G;
    fs << "high_G" << high_G;
    fs << "low_B" << low_B;
    fs << "high_B" << high_B;

    fs.release();
    std::cout << "Threshold variables saved to " << filename << std::endl;
}

// Function to load all thresholding variables from a YAML file
void PiCameraDetection::load_calibration(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open file for reading: " << filename << std::endl;
        return;
    }

    fs["low_H"] >> low_H;
    fs["high_H"] >> high_H;
    fs["low_S"] >> low_S;
    fs["high_S"] >> high_S;
    fs["low_V"] >> low_V;
    fs["high_V"] >> high_V;

    // Load RGB thresholding variables
    fs["low_R"] >> low_R;
    fs["high_R"] >> high_R;
    fs["low_G"] >> low_G;
    fs["high_G"] >> high_G;
    fs["low_B"] >> low_B;
    fs["high_B"] >> high_B;

    fs.release();
    std::cout << "Threshold variables loaded from " << filename << std::endl;
}


void PiCameraDetection::close(){	
	cap.release();
}


static void on_low_H_thresh_trackbar(int, void *)
{
 low_H = min(high_H-1, low_H);
 setTrackbarPos("Low H", window_detection_name, low_H);
}


static void on_high_H_thresh_trackbar(int, void *)
{
 high_H = max(high_H, low_H+1);
 setTrackbarPos("High H", window_detection_name, high_H);
}


static void on_low_S_thresh_trackbar(int, void *)
{
 low_S = min(high_S-1, low_S);
 setTrackbarPos("Low S", window_detection_name, low_S);
}


static void on_high_S_thresh_trackbar(int, void *)
{
 high_S = max(high_S, low_S+1);
 setTrackbarPos("High S", window_detection_name, high_S);
}


static void on_low_V_thresh_trackbar(int, void *)
{
 low_V = min(high_V-1, low_V);
 setTrackbarPos("Low V", window_detection_name, low_V);
}


static void on_high_V_thresh_trackbar(int, void *)
{
 high_V = max(high_V, low_V+1);
 setTrackbarPos("High V", window_detection_name, high_V);
}


static void on_low_R_thresh_trackbar(int, void *)
{
 low_R = min(high_R-1, low_R);
 setTrackbarPos("Low R", window_detection_name, low_R);
}


static void on_high_R_thresh_trackbar(int, void *)
{
 high_R = max(high_R, low_R+1);
 setTrackbarPos("High R", window_detection_name, high_R);
}


static void on_low_G_thresh_trackbar(int, void *)
{
 low_G = min(high_G-1, low_G);
 setTrackbarPos("Low S", window_detection_name, low_G);
}


static void on_high_G_thresh_trackbar(int, void *)
{
 high_G = max(high_G, low_G+1);
 setTrackbarPos("High S", window_detection_name, high_G);
}


static void on_low_B_thresh_trackbar(int, void *)
{
 low_B = min(high_B-1, low_B);
 setTrackbarPos("Low V", window_detection_name, low_B);
}


static void on_high_B_thresh_trackbar(int, void *)
{
 high_B = max(high_B, low_B+1);
 setTrackbarPos("High V", window_detection_name, high_B);


static void on_morph_size(int, void*)
{
  setTrackbarPos("Kernel size:\n 2n +1", window_capture_name, morph_size);
}


static void on_morph_operator(int, void*)
{
  setTrackbarPos("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", window_capture_name, morph_operator);
}


static void on_morph_elem(int, void*)
{
  setTrackbarPos("Element:\n 0: Rect - 1: Cross - 2: Ellipse", window_capture_name, morph_elem);
}


static void on_thresh(int, void*)
{
  setTrackbarPos("Thresh", window_capture_name, thresh);
}

