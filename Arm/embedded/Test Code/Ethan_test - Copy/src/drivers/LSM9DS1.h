#pragma once

#include "hardware/spi.h"
#include "Sensor_Board.h"
#include <cstdint>

class LSM9DS1 {
public:
    LSM9DS1();
    void initialise();
    void readAccel(int16_t& x, int16_t& y, int16_t& z);
    void readGyro(int16_t& x, int16_t& y, int16_t& z);
    void readMag(int16_t& x, int16_t& y, int16_t& z);

private:
    void readBytes(uint8_t regAddr, uint8_t* data, uint8_t numBytes);
    void writeByte(uint8_t regAddr, uint8_t data);
};
