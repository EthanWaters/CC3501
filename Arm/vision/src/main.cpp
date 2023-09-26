#include <PiCameraDetection.h>


int main() {
    // Create instances of PiCameraDetection for each thread
    PiCameraDetection detector1;
    PiCameraDetection detector2; // Create more instances as needed

    // Initialize settings for each detector (e.g., set different thresholds)

    // Start threads to process frames
    std::thread thread1(process_frames, std::ref(detector1));
    std::thread thread2(process_frames, std::ref(detector2)); // Create more threads as needed

    // Join threads (wait for them to finish)
    thread1.join();
    thread2.join(); // Join more threads as needed

    return 0;
}
