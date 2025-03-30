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

const char *ssid = "cev-router";
const char *password = "cev@2024";

// Async Web Server
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// JSON for sensor readings
JSONVar readings;

// UART Configuration
#define UART_RX_PIN 16 // RX pin (ESP32 receives from Pico)
#define UART_TX_PIN 17 // TX pin (not used, but defined)
#define UART_BAUD 115200

// Timer Variables
unsigned long lastTime = 0;
const unsigned long timerDelay = 1000;

// Static IP Configuration
IPAddress local_IP(192, 168, 1, 242);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

// Initialize WiFi
void initWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nConnected to WiFi. IP: " + WiFi.localIP().toString());
}

// Send data to all WebSocket clients
void notifyClients(String sensorReadings)
{
  ws.textAll(sensorReadings);
}

// Read and parse RPM from UART
void readUARTData()
{
  static String uartBuffer = "";

  while (Serial2.available())
  {
    char c = Serial2.read();

    if (c == '\n')
    { // End of message
      uartBuffer.trim();
      Serial.println("UART Received: " + uartBuffer);

      float velocity, throttle;
      // Serial.println(uartBuffer);
      if (sscanf(uartBuffer.c_str(), "%4f%4f", &velocity, &throttle) == 2)
      {
        readings["Velocity"] = velocity;
        readings["Throttle"] = throttle;
        Serial.printf("Parsed -> Velocity: %f | Throttle: %f\n", velocity, throttle);
      }
      else
      {
        Serial.println("UART: Invalid data format!");
      }

      uartBuffer = ""; // Clear buffer after processing
    }
    else
    {
      uartBuffer += c; // Build up message
    }
  }
}

// Get Sensor Readings and return as JSON String
String getSensorReadings()
{
  readUARTData(); // Fetch new RPM data
  return JSON.stringify(readings);
}

// WebSocket Event Handler
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->opcode == WS_TEXT)
  {
    String sensorReadings = getSensorReadings();
    notifyClients(sensorReadings);
  }
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
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
void initWebSocket()
{
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
}

// Setup Function
void setup()
{
  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN); // UART2 for RPM

  if (!WiFi.config(local_IP, gateway, subnet))
  {
    Serial.println("STA Failed to configure");
  }

  initWiFi();
  initWebSocket();
  server.begin();

  Serial.println("Ready to receive sensor data via UART.");
}

// Main Loop
void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - lastTime >= timerDelay)
  {
    String sensorReadings = getSensorReadings();
    Serial.println(sensorReadings);
    notifyClients(sensorReadings);
    lastTime = currentMillis;
  }

  ws.cleanupClients();
}
