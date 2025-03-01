#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
// spiffs makes the file system
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <stdio.h>
#include <Wire.h>
#include <CAN.h>

String getSensorReadings();
// WIFI INSTANTIATIONS
const char* ssid = "CEV_GOOBER"; //change to new ssid for the router
const char* password = "G0Ob3rCEV!"; //change to new password for the router
// creating an AsyncWebServer object on port 80
AsyncWebServer server(80);
//WiFiServer server(80);
//create a websocket object
AsyncWebSocket ws("/ws");
//Json variable to hold sensor readings
JSONVar readings;
// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 200;
// Set your Static IP address
IPAddress local_IP(192, 168, 1, 242);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  //hotspot ssid and password
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting...");
    Serial.print("Status: ");
    Serial.println(WiFi.status());
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  Serial.println("Connected to WiFi");
}
void notifyClients(String sensorReadings) {
  ws.textAll(sensorReadings);
}
void get_network_info(){
    if(WiFi.status() == WL_CONNECTED) {
        Serial.print("[*] Network information for ");
        Serial.println(ssid);
        Serial.println("[+] BSSID : " + WiFi.BSSIDstr());
        Serial.print("[+] Gateway IP : ");
        Serial.println(WiFi.gatewayIP());
        Serial.print("[+] Subnet Mask : ");
        Serial.println(WiFi.subnetMask());
        Serial.println((String)"[+] RSSI : " + WiFi.RSSI() + " dB");
        Serial.print("[+] ESP32 IP : ");
        Serial.println(WiFi.localIP());
    }
}

String getSensorReadings() {
  long id = CAN.packetId();
  Serial.println(id);
  int packetSize = CAN.parsePacket();
    // To hold the received data as string
  Serial.println(packetSize);
  String receivedData = " ";
  if (packetSize) {
    Serial.print("Received packet of size: ");
    Serial.println(packetSize);
    while (CAN.available()) {
      receivedData += (char)CAN.read();  // Append each byte to the string
      }
  } else {
      Serial.println("No data received.");
  }

    Serial.print("Received Data: ");
    Serial.println(receivedData);  // Print the received string for debugging

  // Add other sensor data (e.g., temperature) to the JSON
  readings["temperature"] = String(receivedData); // switch temp and steering for data a and a testing 
  readings["Steering Angle"] = String("19.4");  // Example, replace with actual sensor data

  // Convert the JSON object to a string and return it
  String jsonString = JSON.stringify(readings);
  
  // Print the JSON string for debugging
  Serial.println("JSON STRING:");
  Serial.println(jsonString);
  
  return jsonString;
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    //data[len] = 0;
    //String message = (char*)data;
    // Check if the message is "getReadings"
    //if (strcmp((char*)data, "getReadings") == 0) {
      //if it is, send current sensor readings
      String sensorReadings = getSensorReadings();
      Serial.print(sensorReadings);
      // send to data A&A function
      notifyClients(sensorReadings);
    //}
  }
}
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}
void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}


void setup() {  
  Serial.begin(115200);
  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }
  initWiFi();
  initWebSocket();
  server.begin(); 

  Serial.println("Initializing CAN Bus...");
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {

  // Now process sensor readings after CAN data is received
  if ((millis() - lastTime) > timerDelay) {
    String sensorReadings = getSensorReadings();
    Serial.print(sensorReadings);
    notifyClients(sensorReadings);
  }

  // Delay between readings, to limit frequency
  delay(timerDelay);

  // Cleanup WebSocket clients
  ws.cleanupClients();
}

