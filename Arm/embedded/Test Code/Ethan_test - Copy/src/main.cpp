#include "pico/stdlib.h"
#include <iostream>
#include <stdbool.h>
#include <stdio.h>
// #include <time.h>
// #include <chrono>
// #include "FusionArhs.h"
// #include "FusionAxes.h"
// #include "FusionCalibration.h"
// #include "FusionCompass.h"
// #include "FusionConvention.h"
// #include "FusionOffset.h"
// #include "FusionMath.h"
// #include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
// #include "drivers/MPU9250_driver.h"

#define SAMPLE_PERIOD (0.055f) // 0.05f replace this with actual sample period
#define I2C_SDA_PIN 7
#define I2C_SCL_PIN 6
#define SA01 4
#define CS 5
#define AG_A 0xD4
#define M_A 0x38

int16_t accel[3];
int16_t gyro[3];
int16_t mag[3];

void readRaw(int16_t accel[3], int16_t gyro[3], int16_t mag[3]) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x28;
    i2c_write_blocking(i2c_default, AG_A, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, AG_A, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }


    val = 0x18;
    i2c_write_blocking(i2c_default, AG_A, &val, 1, true);
    i2c_read_blocking(i2c_default, AG_A, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }



    val = 0x28;
    i2c_write_blocking(i2c_default, M_A, &val, 1, true);
    i2c_read_blocking(i2c_default, M_A, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        mag[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

}




int main() {

    stdio_init_all();
    gpio_init(SA01);
    gpio_put(SA01, 0);
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(CS);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);


    //printf("Calibrating...place on level surface...\n");
    sleep_ms(500);
    // Calculate Gyro and Accel Offsets  //
    

    while (true) { // this loop should repeat each time new gyroscope data is available
        readRaw(accel, gyro, mag);
        std::cout << "MAG: ";
        for (int i = 0; i < 3; i++) {
            std::cout << mag[i] << " ";
        }
        std::cout << std::endl;

        std::cout << "ACCEL: ";
        for (int i = 0; i < 3; i++) {
            std::cout << accel[i] << " ";
        }
        std::cout << std::endl;

         std::cout << "GYRO: ";
        for (int i = 0; i < 3; i++) {
            std::cout << gyro[i] << " ";
        }
        std::cout << std::endl;
        
    }
}
