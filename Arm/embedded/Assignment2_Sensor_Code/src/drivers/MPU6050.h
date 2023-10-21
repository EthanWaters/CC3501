// MPU6050Driver.h

#pragma once

#include <stdint.h>

class MPU6050Driver {
public:
    MPU6050Driver();
    void initialize();
    void reset();
    void readRaw(int16_t accel[3], int16_t gyro[3], int16_t* temp);
    void read_magnetometer(int16_t mag_data);
};
