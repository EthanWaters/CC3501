#include <PiCameraDetection.h>

int main() {
    // Number of object detection threads
    const int num_threads = 3;

    // Create instances of PiCameraDetection
    std::vector<PiCameraDetection> detectors(num_threads);

    // Create and start the central thread
    std::thread central_thread(central_thread_function, std::ref(detectors));

    // Wait for all detectors to finish
    for (auto& detector : detectors) {
        detector.join_detection_thread();
    }

    // Wait for the central thread to finish
    central_thread.join();

    return 0;
}
