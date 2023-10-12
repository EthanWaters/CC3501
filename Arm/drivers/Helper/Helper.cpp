// this contains common helpful functions that are not specific to any 
// driver
#include <Helper.h>


template <typename T>
std::string array_to_string(const T& data) {
    std::ostringstream oss;
    size_t data_size = sizeof(data) / sizeof(data[0]);
    for (size_t i = 0; i < data_size; ++i) {
        oss << data[i];
        if (i < data_size - 1) {
            oss << ", ";
        }
    }
    return oss.str();
}

