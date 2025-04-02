#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Arduino_JSON.h>
#include <CAN.h>

// WiFi Credentials
const char *ssid = "CEV_GOOBER";
const char *password = "G0Ob3rCEV!";

// const char *ssid = "cev-router";
// const char *password = "cev@2024";

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

// Make data buffer
float timestamp_buffer[100];
float left_rpm_buffer[100];
float right_rpm_buffer[100];
float throttle_buffer[100];
float steering_buffer[100];

int buffer_start = 0;
int buffer_end = 0;

void buffer_add(float timestamp, float left_rpm, float right_rpm, float throttle, float steering)
{
  timestamp_buffer[buffer_end] = timestamp;
  left_rpm_buffer[buffer_end] = left_rpm;
  right_rpm_buffer[buffer_end] = right_rpm;
  throttle_buffer[buffer_end] = throttle;
  steering_buffer[buffer_end] = steering;

  buffer_end = (buffer_end + 1) % 100;
  if (buffer_end == buffer_start)
  {
    buffer_start = (buffer_start + 1) % 100;
  }
}

void buffer_get(int index, float *timestamp, float *left_rpm, float *right_rpm, float *throttle, float *steering)
{
  *timestamp = timestamp_buffer[(buffer_start + index) % 100];
  *left_rpm = left_rpm_buffer[(buffer_start + index) % 100];
  *right_rpm = right_rpm_buffer[(buffer_start + index) % 100];
  *throttle = throttle_buffer[(buffer_start + index) % 100];
  *steering = steering_buffer[(buffer_start + index) % 100];
}

// Parse buffer into a json string starting at buffer_start and ending at buffer_end
String buffer_to_json()
{
  // Json with timestamp array, left_rpm array, right_rpm array, throttle array, steering array
  JSONVar json;
  JSONVar timestamp_array;
  JSONVar left_rpm_array;
  JSONVar right_rpm_array;
  JSONVar throttle_array;
  JSONVar steering_array;

  for (int i = 0; i < 100; i++)
  {
    float timestamp, left_rpm, right_rpm, throttle, steering;
    buffer_get(i, &timestamp, &left_rpm, &right_rpm, &throttle, &steering);

    timestamp_array[i] = timestamp;
    left_rpm_array[i] = left_rpm;
    right_rpm_array[i] = right_rpm;
    throttle_array[i] = throttle;
    steering_array[i] = steering;
  }

  json["timestamp"] = timestamp_array;
  json["left_rpm"] = left_rpm_array;
  json["right_rpm"] = right_rpm_array;
  json["throttle"] = throttle_array;
  json["steering"] = steering_array;

  String json_str = JSON.stringify(json);

  Serial.println("JSON: ");
  Serial.println(json_str);

  return json_str;
}

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
  // Clear buffer
  buffer_start = 0;
  buffer_end = 0;

  static String uartBuffer = "";

  while (Serial2.available() && buffer_end < 100)
  {
    char c = Serial2.read();

    if (c == '\n')
    { // End of message
      uartBuffer.trim();
      // Serial.println("UART Received: " + uartBuffer);

      float timestamp, left_rpm, throttle, steering;

      if (sscanf(uartBuffer.c_str(), "%f %f %f %f", &timestamp, &left_rpm, &throttle, &steering) == 4)
      {
        // Update buffer with Timestamp in s, RPM, Throttle, Steering Angle
        buffer_add(timestamp, left_rpm, 0, throttle, steering);
        // Serial.printf("Parsed -> Timestamp: %f | Left RPM: %f | Throttle: %f | Steering: %f\n", timestamp, left_rpm, throttle, steering);
      }
      else
      {
        // Serial.println("UART: Invalid data format!");
      }

      uartBuffer = ""; // Clear buffer after processing
    }
    else
    {
      uartBuffer += c; // Build up message
    }
  }
  // Flush Serial2 buffer
  while (Serial2.available())
  {
    Serial2.read();
  }
}

// Get Sensor Readings and return as JSON String
String getSensorReadings()
{
  readUARTData(); // Fetch new RPM data
  return buffer_to_json();
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
    notifyClients(sensorReadings);
    lastTime = currentMillis;
  }

  ws.cleanupClients();
}
