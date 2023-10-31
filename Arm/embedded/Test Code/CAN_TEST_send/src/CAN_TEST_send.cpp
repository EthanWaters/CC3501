#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515.h"
#include <time.h>
#include <iostream>

// SPI 
#define CS_PIN 29
#define TX_PIN 27
#define RX_PIN 28
#define SPI_CLOCK 1000000 //16MHz
#define SCK_PIN 26
#define CAN_RESET 15

//CAN bus+
#define SLAVE_ID 0x010
#define YAW_ID 0x01
#define PITCH_ID 0x02
#define ROLL_ID 0x03
#define PAYLOAD_SIZE 8
// IMU 
#define SAMPLE_PERIOD (0.055f) // 0.05f replace this with actual sample period

__u8 value = 8;

uint64_t check;

MCP2515::ERROR result;
uint8_t flags;
bool flag_error;
uint8_t check_reg, check_reg1, check_reg2, check_reg3;

    int main() {
    
    stdio_init_all();
    gpio_init(CAN_RESET);
    gpio_set_dir(CAN_RESET, 1);
    gpio_put(CAN_RESET, 1);
    MCP2515 mcp2515(spi1, CS_PIN, TX_PIN, RX_PIN, SCK_PIN, SPI_CLOCK);
    
    mcp2515.reset();
    
    mcp2515.setBitrate(CAN_5KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();    
    can_frame tx;
    //Listen loop
    while(true) {
       
 
        tx.can_id = SLAVE_ID;  // Set the CAN ID of the specific slave
        tx.can_dlc = 8;   
        tx.data[0] = 0x48;
        tx.data[1] = 0x45;
        tx.data[2] = 0x4C;
        tx.data[3] = 0x4C;
        tx.data[4] = 0x48;       
        tx.data[5] = 0x45;
        tx.data[6] = 0x4C;
        tx.data[7] = 0x4C;
    // No data payload for the request
        result = mcp2515.sendMessage(&tx);
        if(result != MCP2515::ERROR_OK) {
            printf("ERROR...");
        }
        flags = mcp2515.getErrorFlags();
        flag_error = mcp2515.checkError();
        if(flag_error == true) {
            printf("ERROR...");
        }
       
    }

    return 0;
}
