#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <CAN.h>

// WiFi Credentials
// const char* ssid = "CEV_GOOBER"; 
// const char* password = "G0Ob3rCEV!";

const char* ssid = "cev-router";
const char* password = "cev@2024";

// Async Web Server
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// JSON for sensor readings
JSONVar readings;

// UART Configuration
#define UART_RX_PIN 16  // RX pin (ESP32 receives from Pico)
#define UART_TX_PIN 17  // TX pin (not used, but defined)
#define UART_BAUD 115200

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

// Read and parse RPM from UART
void readUARTData() {
  static String uartBuffer = "";
  
  while (Serial2.available()) {
      char c = Serial2.read();

      if (c == '\n') {  // End of message
          uartBuffer.trim();
          Serial.println("UART Received: " + uartBuffer);

          int leftRPM, rightRPM;
          // Serial.println(uartBuffer);
          if (sscanf(uartBuffer.c_str(), "%04d%04d", &leftRPM, &rightRPM) == 2) {
              readings["Left_RPM"] = leftRPM;
              readings["Right_RPM"] = rightRPM;
              Serial.printf("Parsed -> Left RPM: %d | Right RPM: %d\n", leftRPM, rightRPM);
          } else {
              Serial.println("UART: Invalid data format!");
          }

          uartBuffer = "";  // Clear buffer after processing
      } else {
          uartBuffer += c;  // Build up message
      }
  }
}


// Process CAN Messages
void processCANMessages() {
  while (CAN.parsePacket()) {
    long id = CAN.packetId();
    Serial.print("Received CAN ID: 0x");
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

      case 0x18: {  // Accelerometer Data
        if (CAN.available() >= 6) {
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
  readUARTData(); // Fetch new RPM data
  processCANMessages(); // Fetch new CAN data
  return JSON.stringify(readings);
}

// WebSocket Event Handler
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->opcode == WS_TEXT) {
    String sensorReadings = getSensorReadings();
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
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);  // UART2 for RPM
  
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
  
  Serial.println("Ready to receive RPM data via UART.");
}

// Main Loop
void loop() {
  unsigned long currentMillis = millis();

  readUARTData(); // Read new RPM data from UART

  if (currentMillis - lastTime >= timerDelay) {
    String sensorReadings = getSensorReadings();
    Serial.println(sensorReadings);
    notifyClients(sensorReadings);
    lastTime = currentMillis;
  }

  ws.cleanupClients();
}
