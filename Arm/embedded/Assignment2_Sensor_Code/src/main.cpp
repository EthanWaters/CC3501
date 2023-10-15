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
#include "drivers/MPU9250_driver.h"

#define SAMPLE_PERIOD (0.055f) // 0.05f replace this with actual sample period
#define PIN_MISO 4 //// Configured for PICO PI !!!! ////
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7


int main() {

    stdio_init_all();

    MPU9250Driver mpu9250(PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI);
    mpu9250.initialize();
    mpu9250.resetMPU9250();
    mpu9250.configureMPU9250();
    //mpu9250.initializeAK8963();

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    int16_t acceleration[3], gyro[3], mag[3];

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

    //printf("Calibrating...place on level surface...\n");
    sleep_ms(5000);
    // Calculate Gyro and Accel Offsets  //
    for(int i = 0; i <= 49; i ++ ) {
        mpu9250.readRawData(acceleration, gyro, mag);
        float gyrox_cal = gyro[0]/ 131.1;
        float gyroy_cal = gyro[1]/ 131.1;
        float gyroz_cal = gyro[2]/ 131.1;
        float accelx_cal = acceleration[0] / 16384.0;
        float accely_cal = acceleration[1] / 16384.0;
        float accelz_cal = acceleration[2] / 16384.0;
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

    while (true) { // this loop should repeat each time new gyroscope data is available

        mpu9250.readRawData(acceleration, gyro, mag);
        float gyrox = (gyro[0]/ 131.1) - average_gyro_x;
        float gyroy = (gyro[1]/ 131.1) - average_gyro_y;
        float gyroz = (gyro[2]/ 131.1) - average_gyro_z;
        float accelx = (acceleration[0]/ 16384.0) - average_accel_x;
        float accely = (acceleration[1]/ 16384.0) - average_accel_y;
        float accelz = (acceleration[2]/ 16384.0);
        float magx = mag[0] * 0.15;
        float magy = mag[1] * 0.15;
        float magz = mag[2] * 0.15;



        const FusionVector gyroscope = {gyrox, gyroy, gyroz}; // replace this with actual gyroscope data in degrees/s
        const FusionVector accelerometer = {accelx, accely, accelz}; // replace this with actual accelerometer data in g

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        printf("Orientation: ");
        printf("%f", euler.angle.yaw);
        printf(", ");
        printf("%f", euler.angle.roll);
        printf(", ");
        printf("%f", euler.angle.pitch);
        printf(", ");
        printf("\n");

        //  printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    }
}