#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515/mcp2515.h"


#define CS_PIN 29
#define TX_PIN 27
#define RX_PIN 28
#define SPI_CLOCK 16000000 //16MHz
#define SCK_PIN 26
#define REQUEST_SLAVE1 0x010
#define REQUEST_SLAVE2 0x020
#define REQUEST_SLAVE3 0x030
#define REQUEST_SLAVE4 0x040
#define MASTER_SEND 0x0
#define YAW_ID 0x01
#define PITCH_ID 0x02
#define ROLL_ID 0x03
#define PAYLOAD_SIZE 8

struct can_frame rx;
struct can_frame tx;
segment chest, forearm, bicep, hand;
tx.can_dlc = PAYLOAD_SIZE;
tx.can_id = MASTER_SEND;  // Set the CAN ID of the specific slave
int16_t yaw, roll, pitch; 
std::string arm_angles_s;

int main() {
    stdio_init_all();

    MCP2515 mcp2515(spi1, CS_PIN, TX_PIN, RX_PIN, SCK_PIN, SPI_CLOCK);
    //Initialize interface
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
    mcp2515.setNormalMode();    

    //Listen loop
    while(true) {

        sendRequest(REQUEST_SLAVE1);
        wait_for_data();
        mcp2515.readMessage(rx);
        processSensorData(rx, hand);

        sendRequest(REQUEST_SLAVE2);
        wait_for_data();
        mcp2515.readMessage(rx);
        processSensorData(rx, forearm);

        sendRequest(REQUEST_SLAVE3);
        wait_for_data();
        mcp2515.readMessage(rx);
        processSensorData(rx, bicep);

        sendRequest(REQUEST_SLAVE4);
        wait_for_data();
        mcp2515.readMessage(rx);
        processSensorData(rx, chest)


        get_joint_angles(hand, forearm, bicep, chest, arm_angles)
        arm_angles_s = array_to_string(arm_angles)
        

    }

    return 0;
}


void wait_for_data(){
    bool recieved_data = false;
    while(!recieved_data) {
        recieved_data = checkReceive();
    }
    return 0;
}

void sendRequest(uint32_t slaveID) {
    struct can_frame request_frame;
    request_frame.can_id = slaveID;  // Set the CAN ID of the specific slave
    request_frame.can_dlc = 8;       // No data payload for the request
    mcp2515.sendMessage(request_frame);
}

// Function to receive and process sensor data from a slave
void processSensorData(const struct can_frame* response_frame, segment &segment) {
    
    float yaw, pitch, roll;
    memcpy(&yaw, response_frame->data, sizeof(float));
    memcpy(&pitch, response_frame->data + sizeof(float), sizeof(float));
    memcpy(&roll, response_frame->data + 2 * sizeof(float), sizeof(float));

    // Store the data in the respective arrays
    segment.yaw = yaw;
    segment.pitch = pitch;
    segment.roll = roll;
}

struct segment {
    float yaw;
    float pitch;
    float roll;
}


template <typename T>
std::string array_to_string(const T& data) {
    std::ostringstream oss;
    size_t data_size = sizeof(data) / sizeof(data[0]);
    for (size_t i = 0; i < data_size; ++i) {
        oss << data[i];
        if (i < data_size - 1) {
            oss << ", ";
        }
    }
    return oss.str();
}