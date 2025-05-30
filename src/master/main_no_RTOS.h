#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include <LittleFS.h>

// Define SPI pins for ESP32-S3
#define HSPI_MOSI 11
#define HSPI_MISO 12
#define HSPI_SCLK 13
#define CS_SLAVE1 10  // CS for Slave 1
#define CS_SLAVE2 9   // CS for Slave 2

// Initialize SPI
SPIClass hspi(HSPI);

// WiFi credentials
const char* ssid = "Nha 34";
const char* password = "Tranvandu";

// Initialize web server and WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Buffer size for SPI communication
static const uint32_t BUFFER_SIZE = 8;
uint8_t tx_buf[BUFFER_SIZE] = {0};
uint8_t rx_buf[BUFFER_SIZE] = {0};

// Variables to store SPI data
String slave1ToMaster = "0";
String slave2ToMaster = "0";

// Variables for continuous SPI sending
bool isSendingByte = false;
uint8_t byteToSend = 0;
unsigned long lastWebSocketUpdate = 0;
const unsigned long WEBSOCKET_UPDATE_INTERVAL = 500; // Gửi phản hồi WebSocket mỗi 500ms

// Function to communicate with SPI slave
uint8_t spiTransfer(uint8_t csPin, uint8_t data) {
  digitalWrite(csPin, LOW);
  delayMicroseconds(50);
  uint8_t received = hspi.transfer(data);
  delayMicroseconds(50);
  digitalWrite(csPin, HIGH);
  return received;
}

// Function to send data to WebSocket and print to Serial
void notifyClients(String message) {
  if (ws.count() > 0) { // Chỉ gửi nếu có client kết nối
    ws.textAll(message);
    Serial.println("Sent to Web: " + message);
  }
}

// Initialize LittleFS
void initLittleFS() {
  if (!LittleFS.begin(true)) {
    Serial.println("An error has occurred while mounting LittleFS");
    return;
  }
  Serial.println("LittleFS mounted successfully");
}

// WebSocket event handler
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected");
    notifyClients("WebSocket client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("WebSocket client disconnected");
    notifyClients("WebSocket client disconnected");
    // Đặt lại trạng thái khi WebSocket đóng
    isSendingByte = false;
    Serial.println("Stopped sending byte due to WebSocket disconnect");
  } else if (type == WS_EVT_DATA) {
    data[len] = 0; // Ensure null termination
    String message = (char*)data;
    Serial.println("WebSocket data received: " + message);

    // Parse the message
    JSONVar jsonData = JSON.parse(message);
    if (JSON.typeof(jsonData) == "undefined") {
      Serial.println("Parsing JSON failed!");
      return;
    }

    // Option 1: Handle byte sending
    if (jsonData.hasOwnProperty("action")) {
      String action = (const char*)jsonData["action"];
      if (action == "start" && jsonData.hasOwnProperty("byte")) {
        byteToSend = (int)jsonData["byte"];
        isSendingByte = true;
        Serial.println("Started sending byte: 0x" + String(byteToSend, HEX));
      } else if (action == "stop") {
        isSendingByte = false;
        Serial.println("Stopped sending byte");
      }
    }

    // Option 2: Control RGB LED on slaves
    if (jsonData.hasOwnProperty("slave") && jsonData.hasOwnProperty("color")) {
      String slave = (const char*)jsonData["slave"];
      String color = (const char*)jsonData["color"];
      uint8_t command;

      // Map color to command
      if (color == "red") command = 0x01;
      else if (color == "green") command = 0x02;
      else if (color == "blue") command = 0x03;
      else if (color == "off") command = 0x00;
      else return;

      // Send command to the specified slave
      if (slave == "slave1") {
        spiTransfer(CS_SLAVE1, command);
        Serial.println("Sent LED command to Slave 1: " + color);
      } else if (slave == "slave2") {
        spiTransfer(CS_SLAVE2, command);
        Serial.println("Sent LED command to Slave 2: " + color);
      }

      // Notify WebSocket
      JSONVar response;
      response["type"] = "led_response";
      response["slave"] = slave;
      response["color"] = color;
      String jsonString = JSON.stringify(response);
      notifyClients(jsonString);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Configure CS pins
  pinMode(CS_SLAVE1, OUTPUT);
  pinMode(CS_SLAVE2, OUTPUT);
  digitalWrite(CS_SLAVE1, HIGH);
  digitalWrite(CS_SLAVE2, HIGH);

  // Initialize SPI
  hspi.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI);
  hspi.setFrequency(250000);  // 250 kHz for stability

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize LittleFS
  initLittleFS();

  // Configure WebSocket
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Serve static files from LittleFS
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/");

  // Start server
  server.begin();
}

void loop() {
  ws.cleanupClients();

  // Nếu đang ở trạng thái gửi byte, gửi liên tục qua SPI
  if (isSendingByte) {
    // Send to Slave 1
    uint8_t slave1Response = spiTransfer(CS_SLAVE1, byteToSend);
    slave1ToMaster = String(slave1Response, HEX);
    Serial.println("Slave 1 to Master: 0x" + slave1ToMaster);

    // Send to Slave 2
    uint8_t slave2Response = spiTransfer(CS_SLAVE2, byteToSend);
    slave2ToMaster = String(slave2Response, HEX);
    Serial.println("Slave 2 to Master: 0x" + slave2ToMaster);

    // Chỉ gửi phản hồi WebSocket mỗi 500ms
    unsigned long currentMillis = millis();
    if (currentMillis - lastWebSocketUpdate >= WEBSOCKET_UPDATE_INTERVAL) {
      JSONVar response;
      response["type"] = "byte_response";
      response["s1m"] = "0x" + slave1ToMaster;
      response["s2m"] = "0x" + slave2ToMaster;
      String jsonString = JSON.stringify(response);
      notifyClients(jsonString);
      lastWebSocketUpdate = currentMillis;
    }
  }

  delay(10); // Tốc độ gửi SPI 10ms
}