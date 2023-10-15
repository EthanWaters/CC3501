#pragma once

#include <stdint.h>

class MPU9250Driver {
public:
    MPU9250Driver(uint8_t miso, uint8_t cs, uint8_t sck, uint8_t mosi);
    ~MPU9250Driver();

    void initialize();
    void readRawData(int16_t accel[3], int16_t gyro[3], int16_t mag[3]);

    uint8_t misoPin;
    uint8_t csPin;
    uint8_t sckPin;
    uint8_t mosiPin;

    void selectChip();
    void deselectChip();
    void resetMPU9250();
    void configureMPU9250();
    void initializeAK8963();
    void readRegisters(uint8_t reg, uint8_t* data, uint16_t len);

private:

};
