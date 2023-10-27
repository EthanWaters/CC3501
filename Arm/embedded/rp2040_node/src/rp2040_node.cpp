#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515.h"
#include <time.h>
#include <chrono>
#include "pico/binary_info.h"
#include "LSM9DS1_driver.h"
#include "math.h"

// SPI 
#define PIN_SA0 4
#define PIN_CS   5
#define PIN_SCK  23
#define PIN_SDA 6

//CAN bus
#define SLAVE_ID 0x010
#define AG_CAL_REQUEST 0x500
#define MAG_CAL_REQUEST 0x600
#define START_REQUEST 0x700
#define YAW_ID 0x01
#define PITCH_ID 0x02
#define ROLL_ID 0x03
#define PAYLOAD_SIZE 4
#define CAN_RESET 15

// IMU 
float PI = 3.14159;
float rad_conv = (PI/180);
float pitch_rad, yaw_rad, roll_rad;
OrientationData imu_data;

// CAN bus
struct can_frame rx;
struct can_frame tx;
tx.can_dlc = PAYLOAD_SIZE;


int main() {
    stdio_init_all();

    //Initialise IMU 
    LSM9DS1Driver lsm9ds1(PIN_SA0, PIN_CS, PIN_SCK, PIN_SDA);
    lsm9ds1.initialise_I2C(); // Set up I2C
    lsm9ds1.WHO_AM_I_CHECK_AG(); 
    lsm9ds1.WHO_AM_I_CHECK_M();   
    lsm9ds1.CalibrateAG(); // Run sensor through calibration process //
    sleep_ms(2000); // you've done well, have a rest //
    lsm9ds1.CalibrateMag(); // Run sensor through calibration process //
    sleep_ms(2000); // you've done well, have a rest //

    // Set reset pin of CAN controller high to enable IC
    gpio_init(CAN_RESET);
    gpio_set_dir(CAN_RESET, 1);
    gpio_put(CAN_RESET, 1);

    MCP2515 mcp2515(spi1, CS_PIN, TX_PIN, RX_PIN, SCK_PIN, SPI_CLOCK);
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();    

    // // Gyro and accel calibration request
    // while(true) {
    //     if(mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
    //         if(rx.can_id == AG_CAL_REQUEST) {
    //             lsm9ds1.CalibrateAG(); // Run sensor through calibration process //
    //             sleep_ms(2000); // you've done well, have a rest //
    //             break;
    //         }
    //     }
    // }

    // // Mag calibration request
    // while(true) {
    //     if(mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
    //         if(rx.can_id == (SLAVE_ID + MAG_CAL_REQUEST)) {
    //             lsm9ds1.CalibrateMag(); // Run sensor through calibration process //
    //             sleep_ms(2000); // you've done well, have a rest //
    //             break;
    //         }
    //     }
    // }

    // // wait until arm ready to start
    // while(true) {
    //     if(mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
    //         if(rx.can_id == START_REQUEST) {
    //             break;
    //         }
    //     }
    // }

    
    //Arm active data collection loop
    while(true) {
        
        if(mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
            printf("New frame from ID: %10x\n", rx.can_id);
            if(rx.can_id == SLAVE_ID) {
                                        
                imu_data = lsm9ds1.getdata();
                
                pitch_rad = imu_data.pitch*rad_conv;
                yaw_rad = imu_data.yaw*rad_conv;
                roll_rad = imu_data.roll*rad_conv;

                tx.can_id = SLAVE_ID + YAW_ID;  
                memcpy(tx.data, &yaw_rad, sizeof(float));
                mcp2515.sendMessage(tx);  
                
                tx.can_id = SLAVE_ID + PITCH_ID;  
                memcpy(tx.data, &pitch_rad, sizeof(float));
                mcp2515.sendMessage(tx);  

                tx.can_id = SLAVE_ID + ROLL_ID;  
                memcpy(tx.data, &roll_rad, sizeof(float));
                mcp2515.sendMessage(tx);  
            }
        }
    }

    return 0;
}

