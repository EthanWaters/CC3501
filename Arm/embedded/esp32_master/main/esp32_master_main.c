
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "driver/twai.h"

// CAN bus definitions
#define REQUEST_SLAVE1 0x010
#define REQUEST_SLAVE2 0x020
#define REQUEST_SLAVE3 0x030
#define REQUEST_SLAVE4 0x040
#define MASTER_SEND 0x0
#define YAW_ID 0x01
#define PITCH_ID 0x02
#define ROLL_ID 0x03
#define PAYLOAD_SIZE 4
#define CANTX GPIO_NUM_38
#define CANRX GPIO_NUM_39
// #define portMAX_DELAY 10000
#define NO_OF_DATA_MSGS 3

// Wifi definitions
#define HOST_IP_ADDR "192.168.0.128"
#define PORT 54321
#define WIFI_SSID "elequent"
#define WIFI_PASS "elequent137"
#define MAX_RETRY 10
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// wifi variables
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static const char *payload = "Message from ESP32";

// CAN bus variables
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CANTX, CANRX, TWAI_MODE_NORMAL);


static void udp_client_task(void *pvParameters)
{
    
    int addr_family = 0;
    int ip_protocol = 0;
    printf("Free heap size: %lu", esp_get_free_heap_size());
    

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;


    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    printf("Socket:%i\n", sock);
    if (sock < 0) {
        printf("Error");
    }

    // Set timeout
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

    printf("Socket created, sending to %s:%d\n", HOST_IP_ADDR, PORT);
    twai_message_t rx_msg;

    while (1) {
        
        // uint32_t data_msgs_rec = 0;
        // while (data_msgs_rec < NO_OF_DATA_MSGS) {
            printf("BEFORE TWAI");
            if (twai_receive(&rx_msg, pdMS_TO_TICKS(10000)) == ESP_OK) {
                printf("TWAI IS OKAY");
                if (rx_msg.identifier == REQUEST_SLAVE1) {

                    uint8_t* rec_data[3];
                    for (int i = 0; i < rx_msg.data_length_code; i++) {
                        rec_data[i] = rx_msg.data[i];
                        printf("%02X", rx_msg.data[i]);
                    }
                    printf("\n");
                    for (int i = 0; i < rx_msg.data_length_code; i++) {
                        printf("%c", rx_msg.data[i]);
                    }
                    printf("\n");
                }
            }
        // }
        uint32_t alerts_triggered;
        twai_read_alerts(&alerts_triggered, portMAX_DELAY);

        int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            printf("Error occurred during sending err:%i\n", err);
            break;
        }
        printf("Message sent\n");

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    if (sock != -1) {
        printf("Shutting down socket and restarting...\n");
        shutdown(sock, 0);
        close(sock);
    }

}


static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            printf("retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        printf("connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());

    printf("wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        printf("connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        printf("Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        printf("UNEXPECTED EVENT");
    }
}

void app_main(void)
{

    //setup wifi connection 
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    printf("Connecting wifi...");
    wifi_init_sta();
    
    //Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }

    //Start TWAI driver
    if (twai_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return;
    }


    // begin socket communication    
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

    
}