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
unsigned long timerDelay = 1000;
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


//ITER 1: without multiple sensors
// String getSensorReadings() {
//   long id = CAN.packetId();
//   Serial.println(id);
//   int packetSize = CAN.parsePacket();
//     // To hold the received data as string
//   Serial.println(packetSize);
//   String receivedData = " ";
//   if (packetSize) {
//     Serial.print("Received packet of size: ");
//     Serial.println(packetSize);
//     while (CAN.available()) {
//       receivedData += (char)CAN.read();  // Append each byte to the string
//       }
//   } else {
//       Serial.println("No data received.");
//   }

//     Serial.print("Received Data: ");
//     Serial.println(receivedData);  // Print the received string for debugging

//   // Add other sensor data (e.g., temperature) to the JSON
//   readings["temperature"] = String(receivedData); // switch temp and steering for data a and a testing 
//   readings["Steering Angle"] = String("19.4");  // Example, replace with actual sensor data
//   readings["RPM"] = String();
//   // Convert the JSON object to a string and return it
//   String jsonString = JSON.stringify(readings);
  
//   // Print the JSON string for debugging
//   Serial.println("JSON STRING:");
//   Serial.println(jsonString);
  
//   return jsonString;
// }

//ITER 2: only checks  first case
// String getSensorReadings() {
//   String receivedData = " ";
//   int packetSize = CAN.parsePacket();

//   if (packetSize) {
//     long id = CAN.packetId(); // Get the CAN packet ID
//     Serial.print("Received packet with ID: 0x");
//     Serial.print(id, HEX);
//     Serial.print(" of size: ");
//     Serial.println(packetSize);

//     // Handle packets based on their ID
//     switch (id) {
//       case 0x12: { // Joystick/Steering data
//         uint8_t steeringHigh = CAN.read();
//         uint8_t steeringLow = CAN.read();
//         int steeringValue = (steeringHigh << 8) | steeringLow;
//         readings["Steering_Angle"] = String(steeringValue);
//         Serial.print("Steering Angle: ");
//         Serial.println(steeringValue);
//         //break;
//       }
//       case 0x15: { // RPM data
//         uint8_t leftHigh = CAN.read();
//         uint8_t leftLow = CAN.read();
//         uint8_t rightHigh = CAN.read();
//         uint8_t rightLow = CAN.read();
//         int leftRPM = (leftHigh << 8) | leftLow;
//         int rightRPM = (rightHigh << 8) | rightLow;
//         readings["Left_RPM"] = String(leftRPM);
//         readings["Right_RPM"] = String(rightRPM);
//         Serial.print("Left RPM: ");
//         Serial.print(leftRPM);
//         Serial.print(" | Right RPM: ");
//         Serial.println(rightRPM);
//         //break;
//       }
//       case 0x18: { // Accelerometer data
//         uint8_t xHigh = CAN.read();
//         uint8_t xLow = CAN.read();
//         uint8_t yHigh = CAN.read();
//         uint8_t yLow = CAN.read();
//         uint8_t zHigh = CAN.read();
//         uint8_t zLow = CAN.read();
//         int x = (xHigh << 8) | xLow;
//         int y = (yHigh << 8) | yLow;
//         int z = (zHigh << 8) | zLow;
//         readings["Accel_X"] = String(x);
//         readings["Accel_Y"] = String(y);
//         readings["Accel_Z"] = String(z);
//         Serial.print("Accel X: ");
//         Serial.print(x);
//         Serial.print(" | Accel Y: ");
//         Serial.print(y);
//         Serial.print(" | Accel Z: ");
//         Serial.println(z);
//         //break;
//       }
//       default: {
//         // Handle unknown packet IDs
//         while (CAN.available()) {
//           receivedData += (char)CAN.read(); // Append each byte to the string
//         }
//         Serial.print("Received Data: ");
//         Serial.println(receivedData);
//         readings["Unknown_Data"] = receivedData;
//         break;
//       }
//     }
//   } else {
//     Serial.println("No data received.");
//   }


//ITER 3: updated code, doesnt work. 
String getSensorReadings() {
  // Clear previous readings
  readings = JSONVar();
  
  // Process all available CAN packets
  while (CAN.available()) {
    long id = CAN.packetId(); // Get the CAN packet ID
    int packetSize = CAN.parsePacket();
    Serial.print("Received packet with ID: 0x");
    Serial.print(id, HEX);
    Serial.print(" of size: ");
    Serial.println(packetSize);

    // Handle packets based on their ID
    switch (id) {
      case 0x12: { // Joystick/Steering data
        // uint8_t steeringHigh = CAN.read();
        // uint8_t steeringLow = CAN.read();
        // int steeringValue = (steeringHigh << 8) | steeringLow;
        String receivedData = " ";
        while (CAN.available()) {
          receivedData += (char)CAN.read(); // Append each byte to the string
        }
        readings["Steering_Angle"] = String(receivedData);
        // Serial.print("Steering Angle: ");
        // Serial.println(receivedData);
        break;
      }
      case 0x15: { // RPM data
        uint8_t leftHigh = CAN.read();
        uint8_t leftLow = CAN.read();
        uint8_t rightHigh = CAN.read();
        uint8_t rightLow = CAN.read();
        int leftRPM = (leftHigh << 8) | leftLow;
        int rightRPM = (rightHigh << 8) | rightLow;
        readings["Left_RPM"] = String(leftRPM);
        readings["Right_RPM"] = String(rightRPM);
        Serial.print("Left RPM: ");
        Serial.print(leftRPM);
        Serial.print(" | Right RPM: ");
        Serial.println(rightRPM);
        break;
      }
      case 0x18: { // Accelerometer data
        uint8_t xHigh = CAN.read();
        uint8_t xLow = CAN.read();
        uint8_t yHigh = CAN.read();
        uint8_t yLow = CAN.read();
        uint8_t zHigh = CAN.read();
        uint8_t zLow = CAN.read();
        int x = (xHigh << 8) | xLow;
        int y = (yHigh << 8) | yLow;
        int z = (zHigh << 8) | zLow;
        readings["Accel_X"] = String(x);
        readings["Accel_Y"] = String(y);
        readings["Accel_Z"] = String(z);
        Serial.print("Accel X: ");
        Serial.print(x);
        Serial.print(" | Accel Y: ");
        Serial.print(y);
        Serial.print(" | Accel Z: ");
        Serial.println(z);
        break;
      }
      default: {
        // Handle unknown packet IDs
        // String receivedData = " ";
        // while (CAN.available()) {
        //   receivedData += (char)CAN.read(); // Append each byte to the string
        // }
        Serial.print("Received Data: Unknown Packet ID ");
        // Serial.println(receivedData);
        // readings["Unknown_Data"] = receivedData;
        break;
      }
    } //CAN.read();
  }

  // Convert the JSON object to a string and return it
  String jsonString = JSON.stringify(readings);

  // Print the JSON string for debugging
  // Serial.println("JSON STRING:");
  // Serial.println(jsonString);

  return jsonString;
}

//only for RPM 
// String getSensorReadings() {
//   int packetSize = CAN.parsePacket(); // Parse the CAN packet first
//   if (packetSize == 0) {
//       Serial.println("No CAN data received.");
//       return "";
//   }

//   long id = CAN.packetId(); // Get the CAN packet ID after parsing

//   Serial.printf("Received CAN Frame - ID: 0x%X, Size: %d\n", id, packetSize);

//   if (id == 0x15 && packetSize == 4) { // Ensure correct ID and data length
//       uint8_t leftHigh = CAN.read();
//       uint8_t leftLow = CAN.read();
//       uint8_t rightHigh = CAN.read();
//       uint8_t rightLow = CAN.read();

//       int leftRPM = (leftHigh << 8) | leftLow;
//       int rightRPM = (rightHigh << 8) | rightLow;

//       Serial.printf("Left RPM: %d | Right RPM: %d\n", leftRPM, rightRPM);

//       // Store RPM values in JSON
//       readings["Left_RPM"] = leftRPM;
//       readings["Right_RPM"] = rightRPM;

//       // Convert JSON to string and send to WebSocket clients
//       String jsonString = JSON.stringify(readings);
//       notifyClients(jsonString);

//       return jsonString;
//   } else {
//       Serial.printf("Unknown CAN ID: 0x%X | Size: %d\n", id, packetSize);
//   }
  
//   return "";
// }



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

