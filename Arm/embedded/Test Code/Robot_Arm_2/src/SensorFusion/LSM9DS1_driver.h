#pragma once

#include <stdint.h>

 
struct OrientationData {
float pitch;
float yaw;
float roll;
float ax_G; 
float ay_G; 
float az_G; 
float gx_dps; 
float gy_dps; 
float gz_dps; 
float mx_g; 
float my_g; 
float mz_g; 
};

class LSM9DS1Driver {
public:
    LSM9DS1Driver(uint8_t miso, uint8_t cs, uint8_t sck, uint8_t mosi);
    ~LSM9DS1Driver();
   

    void initialise_I2C();
    void OnStartUp();
    void readAccelData(int16_t * destination);
    void readGyroData(int16_t * destination);
    void readMagData(int16_t * destination);
    OrientationData getdata();
    int16_t readTempData();
    void getMres();
    void getGres();
    void getAres();
    void readRegisters_AG(uint8_t reg, uint8_t* data, uint16_t len);
    void writeRegisters_AG(uint8_t reg, uint8_t data);
    void readRegisters_M(uint8_t reg, uint8_t* data, uint16_t len);
    void writeRegisters_M(uint8_t reg, uint8_t data);
    uint8_t readByte_AG(uint8_t reg, uint8_t data_byte);
    uint8_t sa0Pin;
    uint8_t csPin;
    uint8_t sckPin;
    uint8_t sdaPin;

   
    void resetlsm9ds1();
    void CalibrateAG();
    void CalibrateMag();
    void configurelsm9ds1();
    void accelgyrocalibrateLSM9DS1(float * dest1, float * dest2);
    void selftestLSM9DS1();
    void magcalLSM9DS1(float * dest1);
    void WHO_AM_I_CHECK_AG();
    void WHO_AM_I_CHECK_M();

private:

};
