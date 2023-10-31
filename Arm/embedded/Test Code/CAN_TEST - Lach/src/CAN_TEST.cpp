#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515.cpp"
#include <time.h>
#include <iostream>

// SPI 
#define CS_PIN 29
#define TX_PIN 27
#define RX_PIN 28
#define SPI_CLOCK 16000000 //16MHz
#define SCK_PIN 26

//CAN bus
#define SLAVE_ID 0x010
#define YAW_ID 0x01
#define PITCH_ID 0x02
#define ROLL_ID 0x03
#define PAYLOAD_SIZE 8

// IMU 
#define SAMPLE_PERIOD (0.055f) // 0.05f replace this with actual sample period

__u8 value = 8;
can_frame rx;
can_frame tx;

uint64_t check;




int main() {
    stdio_init_all();
    
    tx.can_dlc = value;
    MCP2515 mcp2515(spi1, CS_PIN, TX_PIN, RX_PIN, SCK_PIN, SPI_CLOCK);
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
    mcp2515.setNormalMode();    

    //Listen loop
    while(true) {

       
        if(mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
            printf("New frame from ID: %10x\n", rx.can_id);
            
            memcpy(&check, rx.data, sizeof(uint64_t));
            std::cout << check << std::endl;

            printf("TEST _______________");
            for (int i = 0; i < PAYLOAD_SIZE; i++) {
                printf("%02X ", rx.data[i]);
            }
            printf("\n");

        }
    }

    return 0;
}
