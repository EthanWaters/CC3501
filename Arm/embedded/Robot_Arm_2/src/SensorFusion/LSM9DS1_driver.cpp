#include "LSM9DS1_driver.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "LSM9DS1Headers.h"


#define PI 3.14159
#define SDA_PIN 6
#define SCL_PIN 23
#define I2C_PORT i2c1


uint8_t LSM9DS1_ADDR_AG = 0x6B;
uint8_t LSM9DS1_ADDR_M = 0x1E;
#define READ_BIT 0x7F

#define SerialDebug true  // set to true to get Serial output for debugging

enum Ascale {  // set of allowable accel full scale settings
  AFS_2G = 0,
  AFS_16G,
  AFS_4G,
  AFS_8G
};

enum Aodr {  // set of allowable gyro sample rates
  AODR_PowerDown = 0,
  AODR_10Hz,
  AODR_50Hz,
  AODR_119Hz,
  AODR_238Hz,
  AODR_476Hz,
  AODR_952Hz
};

enum Abw {  // set of allowable accewl bandwidths
   ABW_408Hz = 0,
   ABW_211Hz,
   ABW_105Hz,
   ABW_50Hz
};

enum Gscale {  // set of allowable gyro full scale settings
  GFS_245DPS = 0,
  GFS_500DPS,
  GFS_NoOp,
  GFS_2000DPS
};

enum Godr {  // set of allowable gyro sample rates
  GODR_PowerDown = 0,
  GODR_14_9Hz,
  GODR_59_5Hz,
  GODR_119Hz,
  GODR_238Hz,
  GODR_476Hz,
  GODR_952Hz
};

enum Gbw {   // set of allowable gyro data bandwidths
  GBW_low = 0,  // 14 Hz at Godr = 238 Hz,  33 Hz at Godr = 952 Hz
  GBW_med,      // 29 Hz at Godr = 238 Hz,  40 Hz at Godr = 952 Hz
  GBW_high,     // 63 Hz at Godr = 238 Hz,  58 Hz at Godr = 952 Hz
  GBW_highest   // 78 Hz at Godr = 238 Hz, 100 Hz at Godr = 952 Hz
};

enum Mscale {  // set of allowable mag full scale settings
  MFS_4G = 0,
  MFS_8G,
  MFS_12G,
  MFS_16G
};

enum Mmode {
  MMode_LowPower = 0, 
  MMode_MedPerformance,
  MMode_HighPerformance,
  MMode_UltraHighPerformance
};

enum Modr {  // set of allowable mag sample rates
  MODR_0_625Hz = 0,
  MODR_1_25Hz,
  MODR_2_5Hz,
  MODR_5Hz,
  MODR_10Hz,
  MODR_20Hz,
  MODR_80Hz
};


// Specify sensor full scale
uint8_t Gscale = GFS_245DPS; // gyro full scale
uint8_t Godr = GODR_238Hz;   // gyro data sample rate
uint8_t Gbw = GBW_med;       // gyro data bandwidth
uint8_t Ascale = AFS_2G;     // accel full scale
uint8_t Aodr = AODR_238Hz;   // accel data sample rate
uint8_t Abw = ABW_50Hz;      // accel data bandwidth
uint8_t Mscale = MFS_4G;     // mag full scale
uint8_t Modr = MODR_10Hz;    // mag data sample rate
uint8_t Mmode = MMode_HighPerformance;  // magnetometer operation mode
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
  


int16_t accelCount[3], gyroCount[3], magCount[3];  // Stores the 16-bit signed accelerometer, gyro, and mag sensor output
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0},  magBias[3] = {0, 0, 0}; // Bias corrections for gyro, accelerometer, and magnetometer
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the LSM9DS1gyro internal chip temperature in degrees Celsius


// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint64_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint64_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint64_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


LSM9DS1Driver::LSM9DS1Driver(uint8_t miso, uint8_t cs, uint8_t sck, uint8_t mosi)
    : sa0Pin(miso), csPin(cs), sckPin(sck), sdaPin(mosi) {
}

LSM9DS1Driver::~LSM9DS1Driver() {
    // Clean up if needed
}

void LSM9DS1Driver::initialise_I2C() {

    gpio_init(4);
    gpio_init(5);
    gpio_set_dir(4, GPIO_OUT);
    gpio_set_dir(5, GPIO_OUT);
    gpio_put(4, 1);
    gpio_put(5, 1); 

    // Initialize SPI and GPIO pins
    gpio_set_function(sdaPin, GPIO_FUNC_I2C);
    gpio_set_function(sckPin, GPIO_FUNC_I2C);
    gpio_pull_up(sdaPin);
    gpio_pull_up(sckPin);

    i2c_init(I2C_PORT, 400*1000); // Initialize I2C at 400 kHz
}


void LSM9DS1Driver::resetlsm9ds1() {
  writeRegisters_AG(LSM9DS1XG_CTRL_REG8, 0x05);
  sleep_ms(20);
  writeRegisters_M(LSM9DS1M_CTRL_REG2_M, 0x0c);
  sleep_ms(20);
}


void LSM9DS1Driver::readRegisters_AG(uint8_t reg, uint8_t* data, uint16_t len) {
    reg &= READ_BIT;
    i2c_write_blocking(I2C_PORT, LSM9DS1_ADDR_AG, &reg, 1, true);
    sleep_ms(10);
    i2c_read_blocking(I2C_PORT, LSM9DS1_ADDR_AG, data, len, false);
}

void LSM9DS1Driver::writeRegisters_AG(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, LSM9DS1_ADDR_AG, data, 2, false);
}


void LSM9DS1Driver::readRegisters_M(uint8_t reg, uint8_t* data, uint16_t len) {
    reg &= READ_BIT;
    i2c_write_blocking(I2C_PORT, LSM9DS1_ADDR_M, &reg, 1, true);
    sleep_ms(10);
    i2c_read_blocking(I2C_PORT, LSM9DS1_ADDR_M, data, len, false);
}

void LSM9DS1Driver::writeRegisters_M(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, LSM9DS1_ADDR_M, data, 2, false);
}


uint8_t LSM9DS1Driver::readByte_AG(uint8_t reg, uint8_t data_byte) {
    reg &= READ_BIT;
    i2c_write_blocking(I2C_PORT, LSM9DS1_ADDR_AG, &reg, 1, true);
    sleep_ms(10);
    i2c_read_blocking(I2C_PORT, LSM9DS1_ADDR_AG, &data_byte, 1, false);
    return data_byte;
}


void LSM9DS1Driver::WHO_AM_I_CHECK_AG() {
  uint8_t who_am_i;
  readRegisters_AG(LSM9DS1_AG_WHO_AM_I, &who_am_i, 1);
  printf("Who Am I XL: 0x%02X\n", who_am_i);
}

void LSM9DS1Driver::WHO_AM_I_CHECK_M() {
  uint8_t who_am_i;
  readRegisters_M(LSM9DS1_AG_WHO_AM_I, &who_am_i, 1);
  printf("Who Am I MAG: 0x%02X\n", who_am_i);
}

void LSM9DS1Driver::OnStartUp() {
    // Set initial input parameters


}


void LSM9DS1Driver::configurelsm9ds1() {
   // enable the 3-axes of the gyroscope
   writeRegisters_AG(LSM9DS1XG_CTRL_REG4, 0x38);
   // configure the gyroscope
   writeRegisters_AG(LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
   sleep_ms(200);
   // enable the three axes of the accelerometer 
   writeRegisters_AG(LSM9DS1XG_CTRL_REG5_XL, 0x38);
   // configure the accelerometer-specify bandwidth selection with Abw
   writeRegisters_AG(LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 |Abw);
   sleep_ms(200);
   // enable block data update, allow auto-increment during multiple byte read
   writeRegisters_AG(LSM9DS1XG_CTRL_REG8, 0x44);
   // configure the magnetometer-enable temperature compensation of mag data
   writeRegisters_M(LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
   writeRegisters_M(LSM9DS1M_CTRL_REG2_M, Mscale << 5 ); // select mag full scale
   writeRegisters_M(LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
   writeRegisters_M(LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
   writeRegisters_M(LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode
   printf("LSM9DS1 Configured! \n");
}


void LSM9DS1Driver::accelgyrocalibrateLSM9DS1(float * dest1, float * dest2)
{  
  uint8_t addr1;
  uint8_t addr2;  
  uint8_t addr3;
  uint8_t addr4;
  uint8_t data_byte;    
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  uint16_t samples, ii;

   // enable the 3-axes of the gyroscope
   writeRegisters_AG(LSM9DS1XG_CTRL_REG4, 0x38);
   // configure the gyroscope
   writeRegisters_AG(LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
   sleep_ms(200);
   // enable the three axes of the accelerometer 
   writeRegisters_AG(LSM9DS1XG_CTRL_REG5_XL, 0x38);
   // configure the accelerometer-specify bandwidth selection with Abw
   writeRegisters_AG(LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 |Abw);
   sleep_ms(200);
   // enable block data update, allow auto-increment during multiple byte read
   writeRegisters_AG(LSM9DS1XG_CTRL_REG8, 0x44);
  
  // First get gyro bias
  readRegisters_AG(LSM9DS1XG_CTRL_REG9, &addr1, 1);
  writeRegisters_AG(LSM9DS1XG_CTRL_REG9, addr1 | 0x02);     // Enable gyro FIFO  
  sleep_ms(50);                                                       // Wait for change to take effect
  writeRegisters_AG(LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  sleep_ms(1000);  // sleep_ms 1000 milliseconds to collect FIFO samples
  
  samples = (readByte_AG((LSM9DS1XG_FIFO_SRC & 0x2F), data_byte)); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
    int16_t gyro_temp[3] = {0, 0, 0};
    readRegisters_AG(LSM9DS1XG_OUT_X_L_G, &data[0], 6);
    gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    gyro_bias[0] += (int32_t) gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    gyro_bias[1] += (int32_t) gyro_temp[1]; 
    gyro_bias[2] += (int32_t) gyro_temp[2]; 
  }  

  gyro_bias[0] /= samples; // average the data
  gyro_bias[1] /= samples; 
  gyro_bias[2] /= samples; 
  
  dest1[0] = (float)gyro_bias[0]*gRes;  // Properly scale the data to get deg/s
  dest1[1] = (float)gyro_bias[1]*gRes;
  dest1[2] = (float)gyro_bias[2]*gRes;
  
  readRegisters_AG(LSM9DS1XG_CTRL_REG9, &addr2, 1);
  writeRegisters_AG(LSM9DS1XG_CTRL_REG9, addr2 & ~0x02);   //Disable gyro FIFO  
  sleep_ms(50);
  writeRegisters_AG(LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable gyro bypass mode
 
  // now get the accelerometer bias
  readRegisters_AG(LSM9DS1XG_CTRL_REG9, &addr3, 1);
  writeRegisters_AG(LSM9DS1XG_CTRL_REG9, addr3 | 0x02);     // Enable accel FIFO  
  sleep_ms(50);                                                       // Wait for change to take effect
  writeRegisters_AG(LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable accel FIFO stream mode and set watermark at 32 samples
  sleep_ms(1000);  // sleep_ms 1000 milliseconds to collect FIFO samples
  
  samples = (readByte_AG((LSM9DS1XG_FIFO_SRC & 0x2F), data_byte)); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the accel data stored in the FIFO
    int16_t accel_temp[3] = {0, 0, 0};
    readRegisters_AG(LSM9DS1XG_OUT_X_L_XL, &data[0], 6);
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1]; 
    accel_bias[2] += (int32_t) accel_temp[2]; 
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 
  
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) (1.0/aRes);}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) (1.0/aRes);}
  
  dest2[0] = (float)accel_bias[0]*aRes;  // Properly scale the data to get g
  dest2[1] = (float)accel_bias[1]*aRes;
  dest2[2] = (float)accel_bias[2]*aRes;
  
  readRegisters_AG(LSM9DS1XG_CTRL_REG9, &addr4, 1);
  writeRegisters_AG(LSM9DS1XG_CTRL_REG9, addr4 & ~0x02);   //Disable accel FIFO  
  sleep_ms(50);
  writeRegisters_AG(LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable accel bypass mode
}

void LSM9DS1Driver::selftestLSM9DS1()
{
  float accel_noST[3] = {0., 0., 0.}, accel_ST[3] = {0., 0., 0.};
  float gyro_noST[3] = {0., 0., 0.}, gyro_ST[3] = {0., 0., 0.};

  writeRegisters_AG(LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
  accelgyrocalibrateLSM9DS1(gyro_noST, accel_noST);
  writeRegisters_AG(LSM9DS1XG_CTRL_REG10,   0x05); // enable gyro/accel self test
  accelgyrocalibrateLSM9DS1(gyro_ST, accel_ST);

  float gyrodx = (gyro_ST[0] - gyro_noST[0]);
  float gyrody = (gyro_ST[1] - gyro_noST[1]);
  float gyrodz = (gyro_ST[2] - gyro_noST[2]);

printf("Gyro self-test results: \n");
printf("x-axis = "); 
printf("%f", gyrodx); 
printf(" dps"); 
printf(" should be between 20 and 250 dps \n");
printf("y-axis = ");
printf("%f", gyrody); 
printf(" dps"); 
printf(" should be between 20 and 250 dps \n");
printf("z-axis = ");
printf("%f", gyrodz); 
printf(" dps");
printf(" should be between 20 and 250 dps \n");
printf("\n"); printf("\n");

  float accdx = 1000.*(accel_ST[0] - accel_noST[0]);
  float accdy = 1000.*(accel_ST[1] - accel_noST[1]);
  float accdz = 1000.*(accel_ST[2] - accel_noST[2]);

  printf("Accelerometer self-test results: ");
  printf("x-axis = "); 
  printf("%f", accdx); 
  printf(" mg"); 
  printf(" should be between 60 and 1700 mg \n");
  printf("y-axis = "); 
  printf("%f", accdy); 
  printf(" mg"); 
  printf(" should be between 60 and 1700 mg \n");
  printf("z-axis = "); 
  printf("%f", accdz); printf(" mg"); 
  printf(" should be between 60 and 1700 mg \n");
  printf("\n"); printf("\n");

  writeRegisters_AG(LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
  sleep_ms(200);
}


void LSM9DS1Driver::magcalLSM9DS1(float * dest1) 
{
  uint8_t data[6]; // data array to hold mag x, y, z, data
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};
 
   // configure the magnetometer-enable temperature compensation of mag data
   writeRegisters_M(LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
   writeRegisters_M(LSM9DS1M_CTRL_REG2_M, Mscale << 5 ); // select mag full scale
   writeRegisters_M(LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
   writeRegisters_M(LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
   writeRegisters_M(LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode
   
  printf("Mag Calibration: Wave device in a figure eight until done! \n");
  sleep_ms(4000);
  
   sample_count = 128; //update this for more accuracy?
   for(ii = 0; ii < sample_count; ii++) {
    int16_t mag_temp[3] = {0, 0, 0};
    readRegisters_M(LSM9DS1M_OUT_X_L_M, &data[0], 6);  // Read the six raw data registers into data array
    mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
    mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    sleep_ms(105);  // at 10 Hz ODR, new mag data is available every 100 ms
   }

    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*mRes;  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes;   
    dest1[2] = (float) mag_bias[2]*mRes;          

  //write biases to accelerometermagnetometer offset registers as counts);
  writeRegisters_M(LSM9DS1M_OFFSET_X_REG_L_M, (int16_t) mag_bias[0]  & 0xFF);
  writeRegisters_M(LSM9DS1M_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
  writeRegisters_M(LSM9DS1M_OFFSET_Y_REG_L_M, (int16_t) mag_bias[1] & 0xFF);
  writeRegisters_M(LSM9DS1M_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
  writeRegisters_M(LSM9DS1M_OFFSET_Z_REG_L_M, (int16_t) mag_bias[2] & 0xFF);
  writeRegisters_M(LSM9DS1M_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);
 
   printf("Mag Calibration done! \n");
}


void LSM9DS1Driver::readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readRegisters_AG(LSM9DS1XG_OUT_X_L_XL, &rawData[0], 6);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}


void LSM9DS1Driver::readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readRegisters_AG(LSM9DS1XG_OUT_X_L_G, &rawData[0], 6);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

void LSM9DS1Driver::readMagData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readRegisters_M(LSM9DS1M_OUT_X_L_M, &rawData[0], 6);  // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

int16_t LSM9DS1Driver::readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readRegisters_AG(LSM9DS1XG_OUT_TEMP_L, &rawData[0], 2);  // Read the two raw data registers sequentially into data array 
  return (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a 16-bit signed value
}


void LSM9DS1Driver::getMres() {
  switch (Mscale)
  {
 	// Possible magnetometer scales (and their register bit settings) are:
	// 4 Gauss (00), 8 Gauss (01), 12 Gauss (10) and 16 Gauss (11)
    case MFS_4G:
          mRes = 4.0/32768.0;
          break;
    case MFS_8G:
          mRes = 8.0/32768.0;
          break;
    case MFS_12G:
          mRes = 12.0/32768.0;
          break;
    case MFS_16G:
          mRes = 16.0/32768.0;
          break;
  }
}

void LSM9DS1Driver::getGres() {
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 245 DPS (00), 500 DPS (01), and 2000 DPS  (11). 
    case GFS_245DPS:
          gRes = 245.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void LSM9DS1Driver::getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 16 Gs (01), 4 Gs (10), and 8 Gs  (11). 
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
  }
}

void LSM9DS1Driver::CalibrateAG(){
  //get accel and gyro resolutions
  getAres();
  getGres();
  // tell us the sensitivities 
  printf("accel sensitivity is: "); printf("%f",1./(1000.*aRes)); printf(" LSB/mg \n");
  printf("gyro sensitivity is: "); printf("%f",1./(1000.*gRes)); printf(" LSB/mdps \n");
  sleep_ms(1500);
  printf("\033[H\033[J");

  printf("Performing gyro and accel self test...");
  selftestLSM9DS1(); // check function of gyro and accelerometer via self test
  printf("Self test complete! \n");
  sleep_ms(1500);
  printf("\033[H\033[J");

  printf("Calibrating gyro and accel... place sensor on flat surface with zero movement \n");
  sleep_ms(1500);
  accelgyrocalibrateLSM9DS1(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
  printf("\n"); printf("\n");
  printf("Calibration Complete! \n");
  printf("accel biases (mg): \n"); printf("ax %f \n",1000.*accelBias[0]); printf("ay %f \n",1000.*accelBias[1]); printf("az %f \n",1000.*accelBias[2]);
  printf("\n"); printf("\n");
  printf("gyro biases (dps): \n"); printf("gx %f \n", gyroBias[0]); printf("ay %f \n", gyroBias[1]); printf("gz %f \n", gyroBias[2]);
  sleep_ms(1500);
  printf("\033[H\033[J");
}

void LSM9DS1Driver::CalibrateMag(){
  // get mag resolution
   getMres();
   printf("mag sensitivity is: "); printf("%f ",1./(1000.*mRes)); printf(" LSB/mGauss \n");
   sleep_ms(1500);
   printf("\033[H\033[J");

   // calibrate mag
   magcalLSM9DS1(magBias);
   printf("mag biases (mG): "); printf("mx %f \n",1000.*magBias[0]); printf("my %f \n",1000.*magBias[1]); printf("mz %f \n",1000.*magBias[2]); 
   sleep_ms(2000); // add sleep_ms to see results before serial spew of data
   printf("\033[H\033[J");
   
   printf("Applying calibration settings and configuring LSM9DS1 \n");
   configurelsm9ds1();
   printf("\033[H\033[J");
   sleep_ms(500); 
   printf("LSM9DS1 initialized for active data mode....\n"); 
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;

        }
  
  
  
 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones. 
            void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, bx, bz;
            float vx, vy, vz, wx, wy, wz;
            float ex, ey, ez;
            float pa, pb, pc;

            // Auxiliary variables to avoid repeated arithmetic
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;   

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
            bx = sqrt((hx * hx) + (hy * hy));
            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

            // Estimated direction of gravity and magnetic field
            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (Ki > 0.0f)
            {
                eInt[0] += ex;      // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f;     // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }

            // Apply feedback terms
            gx = gx + Kp * ex + Ki * eInt[0];
            gy = gy + Kp * ey + Ki * eInt[1];
            gz = gz + Kp * ez + Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

            // Normalise quaternion
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
 
        }

OrientationData LSM9DS1Driver::getdata() {
uint8_t data;   

    OrientationData orientation;

    readRegisters_AG((LSM9DS1XG_STATUS_REG & 0x01), &data, 1); // check if new accel data is ready  
    readAccelData(accelCount);  // Read the x/y/z adc values
 
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2]; 
   
  readRegisters_AG((LSM9DS1XG_STATUS_REG & 0x02), &data, 1); // check if new gyro data is ready  
    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gyroBias[1];  
    gz = (float)gyroCount[2]*gRes - gyroBias[2];   

  
  readRegisters_M((LSM9DS1M_STATUS_REG_M & 0x08), &data, 1); // check if new mag data is ready  
    readMagData(magCount);  // Read the x/y/z adc values
    
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*mRes; // - magBias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes; // - magBias[1];  
    mz = (float)magCount[2]*mRes; // - magBias[2];   
  
  
    uint64_t start_time;
    uint64_t current_time = time_us_64();  // Get the current time in microseconds

    // Calculate the time elapsed since the program started
    uint64_t Now = current_time - start_time;

  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  // Sensors x, y, and z axes of the accelerometer and gyro are aligned. The magnetometer  
  // the magnetometer z-axis (+ up) is aligned with the z-axis (+ up) of accelerometer and gyro, but the magnetometer
  // x-axis is aligned with the -x axis of the gyro and the magnetometer y axis is aligned with the y axis of the gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // For the LSM9DS1, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  -mx,  my, mz);
//  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -mx, my, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = (current_time - start_time) - count;
    if (delt_t > 0) { 

    // if(SerialDebug) {
    // printf("Accel: "); 
    // printf("%f, %f, %f \n", ax, ay, az);
    // printf("Gyro: ");
    // printf("%f, %f, %f ", gx, gy, gz);
    // printf(" deg/s \n");   
    // printf("Mag: ");
    // printf("%f, %f, %f", mx, my, mz ); 
    // printf(" mG \n");
    // printf("mx = "); 

    float ax_G = ax*1000;
    float ay_G = ay*1000;
    float az_G = az*1000;
    float gx_dps = gx;
    float gy_dps = gy;
    float gz_dps = gz;
    float mx_g = mx;
    float my_g = my;
    float mz_g = mz;

  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 11.6f; // Declination at Townsville, Aus
    roll  *= 180.0f / PI;
     
    int yaw_d = round(yaw);
    int pitch_d = round(pitch);
    int roll_d = round(roll);

    orientation.pitch = pitch;
    orientation.yaw = yaw;
    orientation.roll = roll;
    orientation.ax_G = ax_G;
    orientation.ay_G = ay_G;
    orientation.az_G = az_G;
    orientation.gx_dps = gx_dps;
    orientation.gy_dps = gy_dps;
    orientation.gz_dps = gz_dps;
    orientation.mx_g = mx_g;
    orientation.my_g = my_g;
    orientation.mz_g = mz_g;


    // printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n",
    //            (roll), (pitch), (yaw));

    count = current_time - start_time; 
    sumCount = 0;
    sum = 0;    
    

  }
  return orientation;
  
}







