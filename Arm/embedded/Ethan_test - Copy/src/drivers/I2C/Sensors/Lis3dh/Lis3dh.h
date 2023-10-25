#include <stdio.h>
#include <array>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "WS2812.pio.h" // This header file gets produced during compilation from the WS2812.pio file

#define ADDR_ACCEL_X 0x28
#define ADDR_ACCEL_Y 0x2A
#define ADDR_ACCEL_Z 0x2C
#define CTRL_REG_1 0x20

typedef struct {
    float x;
    float y;
    float z;
} accel;


class Lis3dh {
    public:
        // Lis3dh(i2c_inst, uint8_t, int, int, int);
        Lis3dh(uint8_t address, int frequency, int I2C_SDA_PIN, int I2C_SCL_PIN);
        void scale_value(uint16_t raw_value, float *final_value);
        accel read_accel_data();
        void read_data(uint8_t, float*);
        bool write_register(uint8_t, uint8_t);

    private:
        uint8_t _address;
        int _frequency;
        int _I2C_SDA_PIN;
        int _I2C_SCL_PIN;
        // i2c_inst _i2c_instance;
};
