#include "drivers/logging/logging.h"
// Include any additional libraries needed for your LED strip.
#include "Lis3dh.h"
#include "hardware/i2c.h"
#include <stdexcept>
#include <iostream>

// read all accel data in a single transaction to ensure that all measurements were taken at the same time

Lis3dh::Lis3dh(uint8_t address, int frequency, int I2C_SDA_PIN, int I2C_SCL_PIN){
    this-> _address = address;
    this-> _frequency = frequency;
    this-> _I2C_SDA_PIN = I2C_SDA_PIN;
    this-> _I2C_SCL_PIN = I2C_SCL_PIN;
    // this-> _i2c_instance = i2c0;

    const uint8_t TEMP_CFG_REG = 0xC0;
    uint8_t buf[2];
    buf[0] = CTRL_REG_1;
    buf[1] = 0x97;

    i2c_init(i2c0, _frequency*1000);
    gpio_set_function(_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(_I2C_SCL_PIN, GPIO_FUNC_I2C);
    i2c_write_blocking(i2c0, _address, buf, 2, false);
}

void Lis3dh::scale_value(uint16_t raw_value, float *final_value) {
    // Convert with respect to the value being temperature or acceleration reading 
    float senstivity = 0.004;
    *final_value = (float) ((int16_t) raw_value) * senstivity;
}

accel Lis3dh::read_accel_data() {
    accel accel_data;
    read_data(ADDR_ACCEL_X, &accel_data.x);
    read_data(ADDR_ACCEL_Y, &accel_data.y);
    read_data(ADDR_ACCEL_Z, &accel_data.z);
    return(accel_data);
}

void Lis3dh::read_data(uint8_t reg, float *final_value) {
    // Read two bytes of data and store in a 16 bit data structure

    int length = 2;
    reg |= (1 << 7);
    uint8_t raw_data [2];
    int M = 6;
    
    // Tell the device which address we want to read
    if (1 != i2c_write_blocking(i2c0 , _address, &reg, 1, true)) {
        log(LogLevel::ERROR, "lis3dh::read_registers: Failed to select register address.");
    }
    // Now read the data
    int bytes_read = i2c_read_blocking(i2c0 , _address, raw_data, length, false);
    if (bytes_read != length) {
        log(LogLevel::ERROR, "lis3dh::read_registers: Failed to read data.");
    }
    
    int16_t data = (int16_t)(raw_data[0] | (raw_data[1] << 8)) >> M;
    scale_value(data, final_value);
}

bool Lis3dh::write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    int bytes_written = i2c_write_blocking(i2c0 , _address, buf, 2, false);
    if (bytes_written != 2) {
        log(LogLevel::ERROR, "Failed to write to accelerometer register.");
        return false;
    }
    return true;
}