#include "opencv2/opencv.hpp"
#include <sys/time.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include <iomanip>

using namespace std;
using namespace cv;
const String window_capture_name = "Camera";
const String window_detection_name = "Camera_Threshold";

cv::Point2f mc;
int morph_elem = 0;
int morph_size = 0;
int morph_operator = 0;
int thresh = 100;
const int max_operator = 4;
const int max_elem = 2;
const int max_kernel_size = 21;
const int max_thresh = 255;
const int max_value_H = 360/2;
const int max_value = 255;
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
Scalar color = Scalar( 256, 256, 256 );
Scalar color2 = Scalar( 0, 256, 0 );
            

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

int main(int argc, char* argv[])
{
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
    
    //Trackbars for Morphology options 
     createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", window_capture_name, &morph_operator, max_operator, on_morph_operator);
     createTrackbar( "Element:\n 0: Rect - 1: Cross - 2: Ellipse", window_capture_name, &morph_elem, max_elem, on_morph_elem);
     createTrackbar( "Kernel size:\n 2n +1", window_capture_name,
     &morph_size, max_kernel_size, on_morph_size );
     
    
    cv::Mat frame, frame_HSV, frame_threshold, canny_output;
    vector<vector<Point> > contours;
    // Measure the frame rate - initialise variables
    int frame_id = 0;
    timeval start, end;
    gettimeofday(&start, NULL);

    for(;;) {
        if (!cap.read(frame)) {
            printf("Could not read a frame.\n");
            break;
        }

        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        
        // Perform morphology
        int operation = morph_operator + 2;
        Mat element = getStructuringElement(morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
        morphologyEx(frame_threshold, frame_threshold, operation, element);
        
        //Canny(frame_threshold, canny_output, thresh, thresh*2, 3 );
        findContours(frame_threshold, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
        for( size_t i = 0; i< contours.size(); i++ ){
            drawContours( frame, contours, (int)i, color, 2 );
         }
         
        cv::Moments mu = moments(frame_threshold);
        if(mu.m00 > 0){
            mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 ); 
            std::cout << mc << std::endl;
            circle(frame, mc, 4, color2, -1, 8, 0 );
            putText(frame, "Centre", mc,FONT_HERSHEY_COMPLEX, 1,color2, 2);
        }
        
        
        //show frame
        cv::imshow(window_capture_name, frame);
        cv::imshow(window_detection_name, frame_threshold);
        cv::waitKey(1);
        
        
    }

    // Free the camera 
    cap.release();
    return 0;
}

