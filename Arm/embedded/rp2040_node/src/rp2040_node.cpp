#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515/mcp2515.h"


#define CS_PIN 29
#define TX_PIN 27
#define RX_PIN 28
#define SPI_CLOCK 16000000 //16MHz
#define SCK_PIN 26
#define SLAVE_ID 0x010
#define YAW_ID 0x01
#define PITCH_ID 0x02
#define ROLL_ID 0x03
#define PAYLOAD_SIZE 8

struct can_frame rx;
struct can_frame tx;


tx.can_dlc = PAYLOAD_SIZE;

int16_t yaw, roll, pitch, x_gyro, y_gyro, z_gyro, x_accel, y_accel, z_accel, x_mag, y_mag, z_mag;

int main() {
    stdio_init_all();
    LSM9DS1 lsm9ds1();
    MCP2515 mcp2515(spi1, CS_PIN, TX_PIN, RX_PIN, SCK_PIN, SPI_CLOCK);
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
    mcp2515.setNormalMode();    

    //Listen loop
    while(true) {

       
        if(mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
            printf("New frame from ID: %10x\n", rx.can_id);
            if(rx.can_id == MASTER_REQUEST) {
                
                lsm9ds1.readAccel(int16_t x_accel, int16_t y_accel, int16_t z_accel);
                lsm9ds1.readGyro(int16_t x_gyro, int16_t y_gyro, int16_t z_gyro);
                lsm9ds1.readMag(int16_t x_mag, int16_t y_mag, int16_t z_mag);

                /*
                
                LOGIC TO READ ACCEL, GYRO and MAG -> Kalman Filter -> OUTPUT: Yaw, Pitch and Roll.
                
                */

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

