#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515.h"
#include <time.h>
#include <iostream>
#include <cstring>

// SPI 
#define CS_PIN 29
#define TX_PIN 27
#define RX_PIN 28
#define SPI_CLOCK 10000000 //16MHz
#define SCK_PIN 26

//CAN bus
#define SLAVE_ID 0x010
#define YAW_ID 0x01
#define PITCH_ID 0x02
#define ROLL_ID 0x03
#define PAYLOAD_SIZE 8
#define CAN_RESET 15 
// IMU 
#define SAMPLE_PERIOD (0.055f) // 0.05f replace this with actual sample period


uint64_t check;
uint8_t flags;
MCP2515::ERROR result;
bool is_ready;

int main() {
    stdio_init_all();
    gpio_init(CAN_RESET);
    gpio_set_dir(CAN_RESET, 1);
    gpio_put(CAN_RESET, 1);
    can_frame rx;
    rx.can_id = SLAVE_ID; 
    rx.can_dlc = 8;
    rx.data[0] = 0x0;
    rx.data[1] = 0x0;
    rx.data[2] = 0x0;
    rx.data[3] = 0x0;
    rx.data[4] = 0x0;       
    rx.data[5] = 0x0;
    rx.data[6] = 0x0;
    rx.data[7] = 0x0;
    MCP2515 mcp2515(spi1, CS_PIN, TX_PIN, RX_PIN, SCK_PIN, SPI_CLOCK);
    bool flag_error;
    mcp2515.reset();
    mcp2515.setBitrate(CAN_5KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();    

    //Listen loop
    while(true) {
        is_ready = mcp2515.checkReceive();
        result = mcp2515.readMessage(&rx);
        if(is_ready == true){
            printf("IS READY\n");
        }
        if(result == MCP2515::ERROR_OK) {
            printf("New frame from ID: %10x\n", rx.can_id);
            printf("Bytes: ");
            for (int i = 0; i < PAYLOAD_SIZE; i++) {
                printf("%02X ", rx.data[i]);
            }
            printf("\n");
            printf("Char: ");
            for (int i = 0; i < PAYLOAD_SIZE; i++) {
                printf("%c", rx.data[i]);
            }
            printf("\n");

        }
        flags = mcp2515.getErrorFlags();
        flag_error = mcp2515.checkError();
        if(flag_error == true) {
            printf("ERROR...");
        }
    }

    return 0;
}
