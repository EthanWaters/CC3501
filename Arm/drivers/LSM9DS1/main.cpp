#include "pico/stdlib.h"
#include <iostream>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <chrono>
#include "FusionArhs.h"
#include "FusionAxes.h"
#include "FusionCalibration.h"
#include "FusionCompass.h"
#include "FusionConvention.h"
#include "FusionOffset.h"
#include "FusionMath.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "SensorTesting/LSM9DS1_driver.h"

#define SAMPLE_RATE (0.1f) // replace this with actual sample rate

#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  2
#define PIN_MOSI 3
#define SPI_PORT spi0

int main() {

    stdio_init_all();

    printf("Hello, LSM9DS1! Reading raw data from registers via SPI...\n");

    LSM9DS1Driver LSM9DS1(PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI);
    LSM9DS1.initialize();
    LSM9DS1.resetLSM9DS1();
    LSM9DS1.configureLSM9DS1();
    //LSM9DS1.initializeAK8963();
    LSM9DS1.WHO_AM_I_CHECK();
    sleep_ms(5000);





    float gyro_total_x = 0;
    float gyro_total_y = 0;
    float gyro_total_z = 0;
    float accel_total_x = 0;
    float accel_total_y = 0;
    float accel_total_z = 0;
    float average_gyro_x = 0;
    float average_gyro_y = 0;
    float average_gyro_z = 0;
    float average_accel_x = 0;
    float average_accel_y = 0;
    float average_accel_z = 0;
    float divisor = 50;
    float count = 0;

    float gyroscope_sensitivity = 0.00875;
    float accel_sensitivity = 0.061; 
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t accel_x, accel_y, accel_z;
    int16_t mag_x, mag_y, mag_z;

    //printf("Calibrating...place on level surface...\n");
    sleep_ms(5000);
    // Calculate Gyro Offsets  //
    for(int i = 0; i <= 49; i ++ ) {
        LSM9DS1.readGyroData(gyro_x, gyro_y, gyro_z);
        float gyrox_cal = ((gyro_x*gyroscope_sensitivity));
        float gyroy_cal = ((gyro_y*gyroscope_sensitivity));
        float gyroz_cal = ((gyro_z*gyroscope_sensitivity));
        float accelx_cal = ((accel_x*accel_sensitivity)/1000.0);
        float accely_cal = ((accel_y*accel_sensitivity)/1000);
        float accelz_cal = ((accel_z*accel_sensitivity)/1000);
        gyro_total_x = ((gyrox_cal + gyro_total_x));
        gyro_total_y = ((gyroy_cal + gyro_total_y));
        gyro_total_z = ((gyroz_cal + gyro_total_z));
        accel_total_x = (accelx_cal + accel_total_x);
        accel_total_y = (accely_cal + accel_total_y);
        accel_total_z = (accelz_cal + accel_total_z);
        count = count + 1;
    }

    average_gyro_x = gyro_total_x / divisor;
    average_gyro_y = gyro_total_y / divisor;
    average_gyro_z = gyro_total_z / divisor;
    average_accel_x = accel_total_x / divisor;
    average_accel_y = accel_total_y / divisor;
    average_accel_z = accel_total_z / divisor;

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);




    // This loop should repeat each time new gyroscope data is available
    while (true) {
        

        LSM9DS1.readGyroData(gyro_x, gyro_y, gyro_z);
        float gyrox = ((gyro_x*gyroscope_sensitivity)) - average_gyro_x;
        float gyroy = ((gyro_y*gyroscope_sensitivity)) - average_accel_y;
        float gyroz = ((gyro_z*gyroscope_sensitivity)) - average_gyro_z;
        LSM9DS1.readAccelData(accel_x, accel_y, accel_z);
        float accelx = ((accel_x*accel_sensitivity) / 1000.0);
        float accely = ((accel_y*accel_sensitivity) / 1000.0);
        float accelz = ((accel_z*accel_sensitivity) / 1000.0);

      
        const FusionVector gyroscope = {gyrox, gyroy, gyroz}; // replace this with actual gyroscope data in degrees/s
        const FusionVector accelerometer = {accelx, accely, accelz}; // replace this with actual accelerometer data in g

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_RATE);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        //printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

         /* The WebSerial 3D Model Viewer expects data as heading, pitch, roll */
        printf("Orientation: ");
        // printf("%f", euler.angle.yaw);
        // printf(", ");
        // printf("%f", euler.angle.roll);
        // printf(", ");
        // printf("%f", euler.angle.pitch);
        // printf(", ");
        // printf("\n");
        
        printf("%f", euler.angle.yaw);
        printf(", ");
        printf("%f", euler.angle.pitch);
        printf(", ");
        printf("%f", euler.angle.roll);
        printf(", ");
        printf("\n");
      
        
       // printf("%0.1f, %0.1f, %0.1f", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
             // earth.axis.x, earth.axis.y, earth.axis.z); // "Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",

    
     //printf("Acc. X = %.1f g, Y = %.1f g, Z = %.1f g\n", mag_x, mag_y, mag_z);
    //   printf("Gyro. X = %.1f degrees/s, Y = %.1f degrees/s, Z = %.1f degrees/s\n", gyrox, gyroy, gyroz);   
    }
}


