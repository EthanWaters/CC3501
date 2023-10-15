#pragma once

#include "hardware/spi.h"
#include "Sensor_Board.h"
#include <cstdint>

class LSM9DS1 {
public:
    LSM9DS1();
    void readAccel(int16_t& x_accel, int16_t& y_accel, int16_t& z_accel);
    void readGyro(int16_t& x_gyro, int16_t& y_gyro, int16_t& z_gyro);
    void readMag(int16_t& x_mag, int16_t& y_mag, int16_t& z_mag);

private:
    void readBytes(uint8_t regAddr, uint8_t* data, uint8_t numBytes);
    void writeByte(uint8_t regAddr, uint8_t data);
};
