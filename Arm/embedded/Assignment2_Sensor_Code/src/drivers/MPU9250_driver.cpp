#include "mpu9250_driver.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/binary_info.h"


#define SPI_PORT spi0
#define READ_BIT 0x80



MPU9250Driver::MPU9250Driver(uint8_t miso, uint8_t cs, uint8_t sck, uint8_t mosi)
    : misoPin(miso), csPin(cs), sckPin(sck), mosiPin(mosi) {
}

MPU9250Driver::~MPU9250Driver() {
    // Clean up if needed
}

void MPU9250Driver::initialize() {
    // Initialize SPI and GPIO pins
    spi_init(SPI_PORT, 500 * 1000);
    gpio_set_function(misoPin, GPIO_FUNC_SPI);
    gpio_set_function(sckPin, GPIO_FUNC_SPI);
    gpio_set_function(mosiPin, GPIO_FUNC_SPI);

    gpio_init(csPin);
    gpio_set_dir(csPin, GPIO_OUT);
    deselectChip();
}

void MPU9250Driver::selectChip() {
    asm volatile("nop \n nop \n nop");
    gpio_put(csPin, 0);
    asm volatile("nop \n nop \n nop");
}

void MPU9250Driver::deselectChip() {
    asm volatile("nop \n nop \n nop");
    gpio_put(csPin, 1);
    asm volatile("nop \n nop \n nop");
}

void MPU9250Driver::resetMPU9250() {
    uint8_t buf[] = {0x6B, 0x80};
    selectChip();
    spi_write_blocking(SPI_PORT, buf, 2);
    deselectChip();
    sleep_ms(100);
}

void MPU9250Driver::readRegisters(uint8_t reg, uint8_t* data, uint16_t len) {
    reg |= READ_BIT;
    selectChip();
    spi_write_blocking(SPI_PORT, &reg, 1);
    sleep_ms(10);
    spi_read_blocking(SPI_PORT, 0, data, len);
    deselectChip();
    sleep_ms(10);
}

void MPU9250Driver::configureMPU9250() {
    // Configure accelerometer range (+/- 8g)
    uint8_t accel_config = 0x1C;
    uint8_t gyro_config = 0x1B;
    uint8_t mag_config = 0x0A;
    uint8_t accel_setting = 0x00;
    uint8_t gyro_setting = 0x00;
    uint8_t mag_setting = 0x16;
    uint8_t mag_setting_power_down = 0x37;
    uint8_t mag_setting_init = 0x02;

    selectChip();
    spi_write_blocking(SPI_PORT, &accel_config, 1);
    spi_write_blocking(SPI_PORT, &accel_setting, 1);
    deselectChip();
    sleep_ms(10);
    selectChip();
    spi_write_blocking(SPI_PORT, &gyro_config, 1);
    spi_write_blocking(SPI_PORT, &gyro_setting, 1);
    deselectChip();
    sleep_ms(10);
    selectChip();
    spi_write_blocking(SPI_PORT, &mag_setting_power_down, 1);
    spi_write_blocking(SPI_PORT, &mag_setting_init, 1);
    spi_write_blocking(SPI_PORT, &mag_config, 1);
    spi_write_blocking(SPI_PORT, &mag_setting, 1);
    deselectChip();
    sleep_ms(10);
}



void MPU9250Driver::initializeAK8963() {
   // Power on the magnetometer
    uint8_t mode = 0x01;  // Enter continuous mode (0x01)
    selectChip();
    spi_write_blocking(SPI_PORT, &mode, 1);
    deselectChip();
    sleep_ms(10);

    // Read the magnetometer calibration data from Fuse ROM
    uint8_t calibration_data[3 * 3];
    readRegisters(0x10, calibration_data, sizeof(calibration_data));

    // Enable continuous measurement mode
    mode = 0x02;  // Enter continuous measurement mode (0x02)
    selectChip();
    spi_write_blocking(SPI_PORT, &mode, 1);
    deselectChip();
    sleep_ms(10);
}


void MPU9250Driver::readRawData(int16_t accel[3], int16_t gyro[3], int16_t mag[3]) {
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    readRegisters(0x3B, buffer, 6);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    readRegisters(0x43, buffer, 6);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now mag data from reg 0x03 for 6 bytes
    readRegisters(0x03, buffer, 6);

    for (int i = 0; i < 3; i++) {
        mag[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

}