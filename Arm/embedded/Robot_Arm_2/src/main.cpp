#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "SensorFusion/LSM9DS1_driver.h"
#include "math.h"


#define PIN_SA0 4
#define PIN_CS   5
#define PIN_SCK  23
#define PIN_SDA 6

float PI = 3.14159;
float rad_conv = (PI/180);


int main()
{
    stdio_init_all();
    uint64_t start_time = time_us_64();

    //************************************************* SET UP LSM9DS1*************************************************************//
    LSM9DS1Driver lsm9ds1(PIN_SA0, PIN_CS, PIN_SCK, PIN_SDA);
    lsm9ds1.initialise_I2C(); // Set up I2C
    lsm9ds1.WHO_AM_I_CHECK_AG(); // Check that we're talkin' to accel/gyro//
    lsm9ds1.WHO_AM_I_CHECK_M();   // Check that we're talkin' to mag //
    lsm9ds1.CalibrateIMU(); // Run sensor through calibration process //
    sleep_ms(2000); // you've done well, have a rest //

    //*********************************************** GET SENSOR DATA AND PERFORM FUSION *******************************************//
while (true) {
   
    OrientationData orientation = lsm9ds1.getdata();
    
    float pitch_rad = orientation.pitch*rad_conv;
    float yaw_rad = orientation.yaw*rad_conv;
    float roll_rad = orientation.roll*rad_conv;
    float ax = orientation.ax_G;
    float ay = orientation.ay_G;
    float az = orientation.az_G;
    float gx = orientation.gx_dps;
    float gy = orientation.gy_dps;
    float gz = orientation.gz_dps;
    float mx = orientation.mx_g;
    float my = orientation.my_g;
    float mz = orientation.mz_g;
      
    printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n",
               (roll_rad), (pitch_rad), (yaw_rad));


}
}