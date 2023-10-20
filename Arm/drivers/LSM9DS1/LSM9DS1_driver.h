#pragma once

#include <stdint.h>

class LSM9DS1Driver {
public:
    LSM9DS1Driver(uint8_t miso, uint8_t cs, uint8_t sck, uint8_t mosi);
    ~LSM9DS1Driver();

    void initialize();
    void readRawData(int16_t accel[3], int16_t gyro[3], int16_t mag[3]);
    void readAccelData(int16_t& accel_x, int16_t& accel_y, int16_t& accel_z);
    void readGyroData(int16_t& gyro_x, int16_t& gyro_y, int16_t& gyro_z);
    void readMagData(int16_t& mag_x, int16_t& mag_y, int16_t& mag_z);
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
    void writeRegister(uint8_t reg, uint8_t data);
    void WHO_AM_I_CHECK();

private:

};
