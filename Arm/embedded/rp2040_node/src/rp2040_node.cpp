#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515/mcp2515.h"
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

// SPI 
#define CS_PIN 29
#define TX_PIN 27
#define RX_PIN 28
#define SPI_CLOCK 1000000 //16MHz
#define SCK_PIN 26

//CAN bus
#define SLAVE_ID 0x010
#define YAW_ID 0x01
#define PITCH_ID 0x02
#define ROLL_ID 0x03
#define PAYLOAD_SIZE 4
#define CAN_RESET 15

// IMU 
#define SAMPLE_PERIOD (0.055f) // 0.05f replace this with actual sample period

struct can_frame rx;
struct can_frame tx;

tx.can_dlc = PAYLOAD_SIZE;


int main() {
    stdio_init_all();
    LSM9DS1 lsm9ds1();

    gpio_init(CAN_RESET);
    gpio_set_dir(CAN_RESET, 1);
    gpio_put(CAN_RESET, 1);

    cal_offset gyro_offset = lsm9ds1.cal_gyro(gyro);
    cal_offset accel_offset = lsm9ds1.cal_accel(accel);

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    MCP2515 mcp2515(spi1, CS_PIN, TX_PIN, RX_PIN, SCK_PIN, SPI_CLOCK);
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();    

    // Gyro and accel calibration request
     while(true) {
        if(mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
            if(rx.can_id == SLAVE_ID) {
            
            }
        }
     }
    //Arm active data collection loop
    while(true) {

        
        if(mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
       
        }
        if(mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
            printf("New frame from ID: %10x\n", rx.can_id);
            if(rx.can_id == SLAVE_ID) {
                
                imu_data = lsm9ds1.getData();
            

                tx.can_id = SLAVE_ID + YAW_ID;  
                memcpy(tx.data, &yaw, sizeof(float));
                mcp2515.sendMessage(tx);  
                
                tx.can_id = SLAVE_ID + PITCH_ID;  
                memcpy(tx.data, &pitch, sizeof(float));
                mcp2515.sendMessage(tx);  

                tx.can_id = SLAVE_ID + ROLL_ID;  
                memcpy(tx.data, &roll, sizeof(float));
                mcp2515.sendMessage(tx);  
            }
               


        }
    }

    return 0;
}

struct cal_offset {
    x_offset,
    y_offset,
    z_offset
}

cal_offset calibrate_accel(float cal_const){

    int16_t imu_sample[3];
    lsm9ds1.readAccel(imu_sample);
    cal_offset imu_offset;
    float total_x = 0;
    float total_y = 0;
    float total_z = 0;
    float average_x = 0;
    float average_y = 0;
    float average_z = 0;
    float divisor = 50;
    float count = 0;

    //printf("Calibrating...place on level surface...\n");
    sleep_ms(5000);
    for(int i = 0; i <= 50; i ++ ) {
        
        int16_t imu_sample[3];
        lsm9ds1.readAccel(imu_sample);
        float x_cal = imu_sample[0]/ cal_const;
        float y_cal = imu_sample[1]/ cal_const;
        float z_cal = imu_sample[2]/ cal_const;
        total_x = ((x_cal + total_x));
        total_y = ((y_cal + total_y));
        total_z = ((z_cal + total_z));
        count = count + 1;
    }

    imu_offset.average_x = total_x / divisor;
    imu_offset.average_y = total_y / divisor;
    imu_offset.average_z = total_z / divisor;
    
    return imu_offset;
}


cal_offset calibrate_gyro(double cal_const){

    int16_t imu_sample[3];
    lsm9ds1.readGyro(imu_sample);
    cal_offset imu_offset;
    float total_x = 0;
    float total_y = 0;
    float total_z = 0;
    float average_x = 0;
    float average_y = 0;
    float average_z = 0;
    float divisor = 50;
    float count = 0;

    //printf("Calibrating...place on level surface...\n");
    sleep_ms(5000);
    for(int i = 0; i <= 50; i ++ ) {
        ;
        float x_cal = imu_sample[0]/ cal_const;
        float y_cal = imu_sample[1]/ cal_const;
        float z_cal = imu_sample[2]/ cal_const;
        total_x = ((x_cal + total_x));
        total_y = ((y_cal + total_y));
        total_z = ((z_cal + total_z));
        count = count + 1;
    }

    imu_offset.average_x = total_x / divisor;
    imu_offset.average_y = total_y / divisor;
    imu_offset.average_z = total_z / divisor;
    
    return imu_offset;
}