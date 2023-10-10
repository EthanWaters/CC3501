#include "LSM9DS1.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "Sensor_Board.h"

LSM9DS1::LSM9DS1() {}

void LSM9DS1::LSM9DS1() {
    // Initialize SPI
    spi_init(spiPort, 1000000); // Initialize SPI at 1 MHz
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    spi_set_format(spiPort, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    spi_set_slave(spiPort, PIN_CS);

    // set chip select pin output 
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    // Configure accelerometer
    uint8_t ctrl_reg6_xl_data = 0x40; // 2g range, continuous mode
    writeByte(0x20, ctrl_reg6_xl_data);

    // Configure gyroscope
    uint8_t ctrl_reg1_g_data = 0x0F; // 245 dps, continuous mode
    writeByte(0x10, ctrl_reg1_g_data);

    // Configure magnetometer
    uint8_t ctrl_reg1_m_data = 0x9C; // Temperature compensation, high-performance mode
    writeByte(0x20, ctrl_reg1_m_data);
}

void LSM9DS1::readBytes(uint8_t regAddr, uint8_t* data, uint8_t numBytes) {
    uint8_t tx_data[1] = { regAddr | 0x80 };
    spi_write_blocking(spiPort, tx_data, 1);
    spi_read_blocking(spiPort, 0, data, numBytes);
}

void LSM9DS1::writeByte(uint8_t regAddr, uint8_t data) {
    uint8_t tx_data[2] = { regAddr, data };
    spi_write_blocking(spiPort, tx_data, 2);
}

void LSM9DS1::readAccel(int16_t& x_accel, int16_t& y_accel, int16_t& z_accel) {
    uint8_t accel_data[6];
    readBytes(0x28 | 0x80, accel_data, 6); // Read high and low bytes for X, Y, Z axes

    x_accel = (int16_t)((accel_data[1] << 8) | accel_data[0]);
    y_accel = (int16_t)((accel_data[3] << 8) | accel_data[2]);
    z_accel = (int16_t)((accel_data[5] << 8) | accel_data[4]);
}

void LSM9DS1::readGyro(int16_t& x_gyro, int16_t& y_gyro, int16_t& z_gyro) {
    uint8_t gyro_data[6];
    readBytes(0x18 | 0x80, gyro_data, 6); // Read high and low bytes for X, Y, Z axes

    x_gyro = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);
    y_gyro = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);
    z_gyro = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);
}

void LSM9DS1::readMag(int16_t& x_mag, int16_t& y_mag, int16_t& z_mag) {
    uint8_t mag_data[6];
    readBytes(0x28 | 0x80, mag_data, 6); // Read high and low bytes for X, Y, Z axes

    x_mag = (int16_t)((mag_data[1] << 8) | mag_data[0]);
    y_mag = (int16_t)((mag_data[3] << 8) | mag_data[2]);
    z_mag = (int16_t)((mag_data[5] << 8) | mag_data[4]);
}
