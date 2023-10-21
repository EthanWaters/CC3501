#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "SensorFusion/LSM9DS1_driver.h"

#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  2
#define PIN_MOSI 3
#define SPI_PORT spi0

float gyroscope_sensitivity = 0.00875;
float accel_sensitivity = 0.061; 

int main()
{
    stdio_init_all();

    LSM9DS1Driver lsm9ds1(PIN_MISO, PIN_CS, PIN_SCK, PIN_MOSI);
    lsm9ds1.initialize();
    lsm9ds1.resetlsm9ds1();
    lsm9ds1.configurelsm9ds1();
    lsm9ds1.WHO_AM_I_CHECK();
    sleep_ms(2000);

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
    float divisor = 100;
    float count = 0;

    int16_t gyro_x, gyro_y, gyro_z;
    int16_t accel_x, accel_y, accel_z;
    int16_t mag_x, mag_y, mag_z;

    //printf("Calibrating...place on level surface...\n");
    sleep_ms(5000);
    // Calculate Gyro Offsets  //
    for(int i = 0; i <= 99; i ++ ) {
        lsm9ds1.readGyroData(gyro_x, gyro_y, gyro_z);
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

     while (true) {
        

        lsm9ds1.readGyroData(gyro_x, gyro_y, gyro_z);
        float gyrox = ((gyro_x*gyroscope_sensitivity)) - average_gyro_x;
        float gyroy = ((gyro_y*gyroscope_sensitivity)) - average_accel_y;
        float gyroz = ((gyro_z*gyroscope_sensitivity)) - average_gyro_z;
        lsm9ds1.readAccelData(accel_x, accel_y, accel_z);
        float accelx = ((accel_x*accel_sensitivity) / 1000.0);
        float accely = ((accel_y*accel_sensitivity) / 1000.0);
        float accelz = ((accel_z*accel_sensitivity) / 1000.0);

    
    printf("Acc. X = %.1f g, Y = %.1f g, Z = %.1f g\n", accelx, accely, accelz);
    printf("Gyro. X = %.1f degrees/s, Y = %.1f degrees/s, Z = %.1f degrees/s\n", gyrox, gyroy, gyroz);   
    }

    return 0;
}
