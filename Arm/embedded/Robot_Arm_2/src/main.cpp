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
    lsm9ds1.CalibrateAG(); // Run sensor through calibration process //
    lsm9ds1.CalibrateMag();
    sleep_ms(2000); // you've done well, have a rest //

    //*********************************************** GET SENSOR DATA AND PERFORM FUSION *******************************************//
while (true) {
   
    OrientationData orientation = lsm9ds1.getdata();

    float pitch_rad = orientation.pitch*rad_conv;
    float yaw_rad = orientation.yaw*rad_conv;
    float roll_rad = orientation.roll*rad_conv;

      
    printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n",
               (roll_rad), (pitch_rad), (yaw_rad));


}
}