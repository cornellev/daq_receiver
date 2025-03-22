#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <Wire.h>
#include <CAN.h>

// WiFi Credentials
const char* ssid = "CEV_GOOBER"; 
const char* password = "G0Ob3rCEV!";

// Async Web Server
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// JSON for sensor readings
JSONVar readings;

// Timer Variables
unsigned long lastTime = 0;
const unsigned long timerDelay = 1000;

// Static IP Configuration
IPAddress local_IP(192, 168, 1, 242);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nConnected to WiFi. IP: " + WiFi.localIP().toString());
}

// Send data to all WebSocket clients
void notifyClients(String sensorReadings) {
  ws.textAll(sensorReadings);
}

// Process CAN Messages
void processCANMessages() {
  while (CAN.parsePacket()) { // Check if a new CAN message is available
    long id = CAN.packetId();
    Serial.print("Received packet with ID: 0x");
    Serial.print(id, HEX);
    Serial.print(", Size: ");
    Serial.println(CAN.packetDlc());

    switch (id) {
      case 0x12: {  // Steering Data
        String receivedData = "";
        while (CAN.available()) {
          receivedData += (char)CAN.read();
        }
        readings["Steering Angle"] = receivedData;
        Serial.println("Steering Angle: " + receivedData);
        break;
      }

      case 0x15: {  // RPM Data
        if (CAN.available() >= 4) {  // Ensure full data is available
          uint8_t leftHigh = CAN.read();
          uint8_t leftLow = CAN.read();
          uint8_t rightHigh = CAN.read();
          uint8_t rightLow = CAN.read();
          int leftRPM = (leftHigh << 8) | leftLow;
          int rightRPM = (rightHigh << 8) | rightLow;
          readings["Left_RPM"] = leftRPM;
          readings["Right_RPM"] = rightRPM;
          Serial.printf("Left RPM: %d | Right RPM: %d\n", leftRPM, rightRPM);
        }
        break;
      }

      case 0x18: {  // Accelerometer Data
        if (CAN.available() >= 6) {  // Ensure full data is available
          uint8_t xHigh = CAN.read();
          uint8_t xLow = CAN.read();
          uint8_t yHigh = CAN.read();
          uint8_t yLow = CAN.read();
          uint8_t zHigh = CAN.read();
          uint8_t zLow = CAN.read();
          int x = (xHigh << 8) | xLow;
          int y = (yHigh << 8) | yLow;
          int z = (zHigh << 8) | zLow;
          readings["X"] = x;
          readings["Y"] = y;
          readings["Z"] = z;
          Serial.printf("Accel X: %d | Accel Y: %d | Accel Z: %d\n", x, y, z);
        }
        break;
      }

      default:
        Serial.println("Unknown CAN packet ID.");
        break;
    }
  }
}

// Get Sensor Readings and return as JSON String
String getSensorReadings() {
  processCANMessages();  // Fetch new CAN data
  return JSON.stringify(readings);
}

// WebSocket Event Handler
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->opcode == WS_TEXT) {
    String sensorReadings = getSensorReadings();
    //Serial.println(sensorReadings);
    notifyClients(sensorReadings);
  }
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
      case WS_EVT_CONNECT:
        Serial.printf("Client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        break;
      case WS_EVT_DISCONNECT:
        Serial.printf("Client #%u disconnected\n", client->id());
        break;
      case WS_EVT_DATA:
        handleWebSocketMessage(arg, data, len);
        break;
      default:
        break;
    }
  }
  
  
  // Initialize WebSocket
  void initWebSocket() {
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
  }
  
  // Setup Function
  void setup() {
    Serial.begin(115200);
  
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
    Serial.println("CAN initialized.");
  }
  
  // Main Loop
  void loop() {
    unsigned long currentMillis = millis();
  
    // Only process CAN data at defined intervals
    if (currentMillis - lastTime >= timerDelay) {
      String sensorReadings = getSensorReadings();
      Serial.println(sensorReadings);
      notifyClients(sensorReadings);
      lastTime = currentMillis;
    }
  
    // Cleanup WebSocket Clients
    ws.cleanupClients();
  }
  