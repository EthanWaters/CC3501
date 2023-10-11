#include <PiCameraDetection.h>


PiCameraDetection::PiCameraDetection() : window_capture_name("Camera"),
      window_detection_name("Camera_Threshold"),
      max_operator(4),
      max_elem(2),
      max_kernel_size(21),
      max_thresh(255),
      max_value_H(360 / 2),
      max_value(255)

{   
	int morph_elem;
	int morph_size;
	int morph_operator;
	int low_H, low_S, low_V;
	int high_R, high_G, high_B;
	int low_R, low_G, low_B;
	int high_H, high_S, high_V;
	Scalar color;
	Scalar color2;
	cv::Mat frame, frame_HSV, frame_threshold, canny_output;
	vector<vector<Point>> contours;
	cv::VideoCapture cap;
}	



int PiCameraDetection::init_capture(){
    
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
    
    return 0;
    
}


void PiCameraDetection::init_window(){
	// Create the OpenCV window
    cv::namedWindow(window_capture_name, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(window_detection_name, cv::WINDOW_AUTOSIZE);
    
    //auto on_low_H_thresh_trackbar_adapter = cv::adapt(
    //[](int pos, void* data) {
      //PiCameraDetection* current_instance = (PiCameraDetection*)data;

      //current_instance->low_H = min(current_instance->high_H-1, current_instance->low_H);
      //setTrackbarPos("Low H", window_detection_name, current_instance->low_H);
    //},
    //this);
    
     // Trackbars to set thresholds for HSV values
     createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar, this);
     createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar, this);
     createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar, this);
     createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar, this);
     createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar, this);
     createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar, this);
     createTrackbar("Low R", window_detection_name, &low_R, max_value, on_low_R_thresh_trackbar, this);
     createTrackbar("High R", window_detection_name, &high_R, max_value, on_high_R_thresh_trackbar, this);
     createTrackbar("Low G", window_detection_name, &low_G, max_value, on_low_G_thresh_trackbar, this);
     createTrackbar("High G", window_detection_name, &high_G, max_value, on_high_G_thresh_trackbar, this);
     createTrackbar("Low B", window_detection_name, &low_B, max_value, on_low_B_thresh_trackbar, this);
     createTrackbar("High B", window_detection_name, &high_V, max_value, on_high_B_thresh_trackbar, this);
    
    //Trackbars for Morphology options 
     createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", window_capture_name, &morph_operator, max_operator, on_morph_operator, this);
     createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse", window_capture_name, &morph_elem, max_elem, on_morph_elem, this);
     createTrackbar( "Kernel size:\n 2n +1", window_capture_name, &morph_size, max_kernel_size, on_morph_size, this);
    
}


int PiCameraDetection::detect_coordinates(){	
	if (!cap.read(frame)) {
            printf("Could not read a frame.\n");
            return 1;
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
	
	cv::Moments mu = moments(frame_threshold);
	if(mu.m00 > 0){
	    mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 ); 
		
	}
	return 0;
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


// Function to save all thresholding variables to a YAML file
int PiCameraDetection::save_calibration(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return 1;
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
    return 0;
}

// Function to load all thresholding variables from a YAML file
int PiCameraDetection::load_calibration(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open file for reading: " << filename << std::endl;
        return 1;
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
    return 0;
}


cv::Point2f PiCameraDetection::get_centroid() {
	return mc;
}
    
    
void PiCameraDetection::close(){	
	cap.release();
}

void PiCameraDetection::on_low_H_thresh_trackbar(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
    current_instance->low_H = min(current_instance->high_H-1, current_instance->low_H);
    setTrackbarPos("Low H", current_instance->window_detection_name, current_instance->low_H);
}


void PiCameraDetection::on_high_H_thresh_trackbar(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
    current_instance->high_H = max(current_instance->high_H, current_instance->low_H+1);
    setTrackbarPos("High H", current_instance->window_detection_name, current_instance->high_H);
}


void PiCameraDetection::on_low_S_thresh_trackbar(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
 current_instance->low_S = min(current_instance->high_S-1, current_instance->low_S);
 setTrackbarPos("Low S", current_instance->window_detection_name, current_instance->low_S);
}


void PiCameraDetection::on_high_S_thresh_trackbar(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
 current_instance->high_S = max(current_instance->high_S, current_instance->low_S+1);
 setTrackbarPos("High S", current_instance->window_detection_name, current_instance->high_S);
}


void PiCameraDetection::on_low_V_thresh_trackbar(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
 current_instance->low_V = min(current_instance->high_V-1, current_instance->low_V);
 setTrackbarPos("Low V", current_instance->window_detection_name, current_instance->low_V);
}


void PiCameraDetection::on_high_V_thresh_trackbar(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
 current_instance->high_V = max(current_instance->high_V, current_instance->low_V+1);
 setTrackbarPos("High V", current_instance->window_detection_name, current_instance->high_V);
}


void PiCameraDetection::on_low_R_thresh_trackbar(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
 current_instance->low_R = min(current_instance->high_R-1, current_instance->low_R);
 setTrackbarPos("Low R", current_instance->window_detection_name, current_instance->low_R);
}


void PiCameraDetection::on_high_R_thresh_trackbar(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
 current_instance->high_R = max(current_instance->high_R, current_instance->low_R+1);
 setTrackbarPos("High R", current_instance->window_detection_name, current_instance->high_R);
}


void PiCameraDetection::on_low_G_thresh_trackbar(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
 current_instance->low_G = min(current_instance->high_G-1, current_instance->low_G);
 setTrackbarPos("Low S", current_instance->window_detection_name, current_instance->low_G);
}


void PiCameraDetection::on_high_G_thresh_trackbar(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
 current_instance->high_G = max(current_instance->high_G, current_instance->low_G+1);
 setTrackbarPos("High S", current_instance->window_detection_name, current_instance->high_G);
}


void PiCameraDetection::on_low_B_thresh_trackbar(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
 current_instance->low_B = min(current_instance->high_B-1, current_instance->low_B);
 setTrackbarPos("Low V", current_instance->window_detection_name, current_instance->low_B);
}


void PiCameraDetection::on_high_B_thresh_trackbar(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
 current_instance->high_B = max(current_instance->high_B, current_instance->low_B+1);
 setTrackbarPos("High V", current_instance->window_detection_name, current_instance->high_B);
}

void PiCameraDetection::on_morph_size(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
  setTrackbarPos("Kernel size:\n 2n +1", current_instance->window_capture_name, current_instance->morph_size);
}


void PiCameraDetection::on_morph_operator(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
  setTrackbarPos("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", current_instance->window_capture_name, current_instance->morph_operator);
}


void PiCameraDetection::on_morph_elem(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
  setTrackbarPos("Element:\n 0: Rect - 1: Cross - 2: Ellipse", current_instance->window_capture_name, current_instance->morph_elem);
}


void PiCameraDetection::on_thresh(int, void* data)
{
    PiCameraDetection* current_instance = static_cast<PiCameraDetection*>(data);
  setTrackbarPos("Thresh", current_instance->window_capture_name, current_instance->thresh);
}

