#include "LSM9DS1_driver.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/binary_info.h"
#include <stdio.h>
#include <string.h>




#define SPI_PORT spi0
#define READ_BIT 0x80
uint8_t LSM9DS1_AG_WHO_AM_I = 0xF;
uint8_t LSM9DS1_CTRL_REG8 = 0x22;
uint8_t LSM9DS1_RESET1 = 0x05;
uint8_t LSM9DS1_CTRL_REG2_M  = 0x21;
uint8_t LSM9DS1_RESET2 = 0x0c;
uint8_t LSM9DS1_CTRL_REG1_G = 0x10;
uint8_t LSM9DS1_Gyro = 0x78;
uint8_t LSM9DS1_CTRL_REG6_XL = 0x20;
uint8_t LSM9DS1_Accel = 0x7; //0x70
uint8_t LSM9DS1AG_CTRL_REG4 =  0x1E;
uint8_t LSM9DS1AG_CTRL_REG5_XL = 0x1F;
uint8_t LSM9DS1AG_CTRL_REG8   = 0x22;
uint8_t LSM9DS1M_CTRL_REG5_M  = 0x24;


LSM9DS1Driver::LSM9DS1Driver(uint8_t miso, uint8_t cs, uint8_t sck, uint8_t mosi)
    : misoPin(miso), csPin(cs), sckPin(sck), mosiPin(mosi) {
}

LSM9DS1Driver::~LSM9DS1Driver() {
    // Clean up if needed
}

void LSM9DS1Driver::initialize() {
    // Initialize SPI and GPIO pins
    spi_init(SPI_PORT, 800 * 1000);
    gpio_set_function(misoPin, GPIO_FUNC_SPI);
    gpio_set_function(sckPin, GPIO_FUNC_SPI);
    gpio_set_function(mosiPin, GPIO_FUNC_SPI);

    gpio_init(csPin);
    gpio_set_dir(csPin, GPIO_OUT);
    deselectChip();

}

void LSM9DS1Driver::selectChip() {
    asm volatile("nop \n nop \n nop");
    gpio_put(csPin, 0);
    asm volatile("nop \n nop \n nop");
}

void LSM9DS1Driver::deselectChip() {
    asm volatile("nop \n nop \n nop");
    gpio_put(csPin, 1);
    asm volatile("nop \n nop \n nop");
}

void LSM9DS1Driver::resetlsm9ds1() {
  writeRegister(LSM9DS1_CTRL_REG8, 0x05);
  sleep_ms(20);
  writeRegister(LSM9DS1_CTRL_REG2_M, 0x0c);
  sleep_ms(20);
}


void LSM9DS1Driver::readRegisters(uint8_t reg, uint8_t* data, uint16_t len) {
    reg |= READ_BIT;
    selectChip();
    spi_write_blocking(SPI_PORT, &reg, 1);
    sleep_ms(10);
    spi_read_blocking(SPI_PORT, 0, data, len);
    deselectChip();
}

void LSM9DS1Driver::writeRegister(uint8_t reg, uint8_t data) {
    reg & 0x7F;
    selectChip();
    uint8_t tx_data[2] = {reg, data};
    spi_write_blocking(SPI_PORT, tx_data, 2);
    deselectChip();
}

void LSM9DS1Driver::WHO_AM_I_CHECK() {
  uint8_t who_am_i;
  readRegisters(LSM9DS1_AG_WHO_AM_I, &who_am_i, 1);
  printf("Who Am I: 0x%02X\n", who_am_i);
}

void LSM9DS1Driver::configurelsm9ds1() {
    writeRegister(LSM9DS1AG_CTRL_REG4, 0x38); //0x38
    sleep_ms(10);
    writeRegister(LSM9DS1AG_CTRL_REG5_XL, 0x38); //0x38
    sleep_ms(10);
    writeRegister(LSM9DS1AG_CTRL_REG8, 0x44); //0x44
    sleep_ms(10);
    writeRegister(LSM9DS1_CTRL_REG1_G, 0x30); //0x30
    sleep_ms(10);
    writeRegister(LSM9DS1_CTRL_REG6_XL, 0x0); //0x18 //0x0
    sleep_ms(10);
    writeRegister(LSM9DS1M_CTRL_REG5_M, 0x40);


}

void LSM9DS1Driver::readAccelData(int16_t& accel_x, int16_t& accel_y, int16_t& accel_z) {
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    readRegisters(0x28, buffer, 6);

    accel_x = (buffer[1] << 8 | buffer[0]);
    accel_y = (buffer[3] << 8 | buffer[2]);
    accel_z = (buffer[5] << 8 | buffer[4]);
}

void LSM9DS1Driver::readGyroData(int16_t& gyro_x, int16_t& gyro_y, int16_t& gyro_z) {
    uint8_t buffer[6];

    // Now gyro data from reg 0x43 for 6 bytes
    readRegisters(0x18, buffer, 6);

    gyro_x = (buffer[1] << 8 | buffer[0]);
    gyro_y = (buffer[3] << 8 | buffer[2]);
    gyro_z = (buffer[5] << 8 | buffer[4]);
}

void LSM9DS1Driver::readMagData(int16_t& mag_x, int16_t& mag_y, int16_t& mag_z) {
    uint8_t buffer[6];

    // Now gyro data from reg 0x43 for 6 bytes
    readRegisters(0x28, buffer, 6);

    mag_x = (buffer[1] << 8 | buffer[0]);
    mag_y = (buffer[3] << 8 | buffer[2]);
    mag_z = (buffer[5] << 8 | buffer[4]);
}



//void LSM9DS1Driver::calibrate(int16_t& gyro_x,int16_t& gyro_y,int16_t& gyro_z, int16_t& accel_x,int16_t& accel_y,int16_t& accel_z,     float gyroscope_sensitivity = 0.00875) {


//}

