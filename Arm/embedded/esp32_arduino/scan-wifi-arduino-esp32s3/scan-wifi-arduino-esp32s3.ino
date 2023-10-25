#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "Your_SSID";       // Replace with your Wi-Fi network SSID
const char* password = "Your_PASSWORD"; // Replace with your Wi-Fi network password

const char* udpServerIP = "UDP_SERVER_IP"; // Replace with the IP address of the UDP server
const int udpServerPort = 12345;           // Replace with the port number of the UDP server

WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  connectToWiFi();
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    // Connected to Wi-Fi, now send data over UDP
    sendUDPData();
  } else {
    // Not connected to Wi-Fi, try to reconnect
    connectToWiFi();
  }
}

void connectToWiFi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }
  Serial.println("Connected to Wi-Fi");
}

void sendUDPData() {
  udp.beginPacket(udpServerIP, udpServerPort);
  udp.print("Hello, UDP Server!"); // Replace with your data to send
  udp.endPacket();
}
