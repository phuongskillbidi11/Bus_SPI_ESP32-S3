#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>

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

// Variables to store SPI data
String masterToSlave1 = "0xAA";
String masterToSlave2 = "0xBB";
String slave1ToMaster = "0";
String slave2ToMaster = "0";

// Function to communicate with SPI slave
uint8_t spiTransfer(uint8_t csPin, uint8_t data) {
  digitalWrite(csPin, LOW);  // Activate slave
  uint8_t received = hspi.transfer(data);  // Send and receive data
  digitalWrite(csPin, HIGH);  // Deactivate slave
  return received;
}

// WebSocket event handler
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("WebSocket client disconnected");
  } else if (type == WS_EVT_DATA) {
    // Process data from client (if needed)
    data[len] = 0; // Ensure null termination
    Serial.printf("WebSocket data received: %s\n", (char*)data);
  }
}

// HTML for web interface
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32-S3 SPI WebSocket Monitor</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; }
    h2 { color: #333; }
    p { font-size: 1.2em; }
  </style>
</head>
<body>
  <h2>ESP32-S3 SPI WebSocket Monitor</h2>
  <p>Master to Slave 1: <span id="m2s1">0xAA</span></p>
  <p>Master to Slave 2: <span id="m2s2">0xBB</span></p>
  <p>Slave 1 to Master: <span id="s1m">0</span></p>
  <p>Slave 2 to Master: <span id="s2m">0</span></p>
  <script>
    var ws = new WebSocket("ws://" + window.location.hostname + "/ws");
    ws.onmessage = function(event) {
      var data = JSON.parse(event.data);
      document.getElementById("m2s1").innerText = data.m2s1;
      document.getElementById("m2s2").innerText = data.m2s2;
      document.getElementById("s1m").innerText = data.s1m;
      document.getElementById("s2m").innerText = data.s2m;
    };
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);

  // Configure CS pins
  pinMode(CS_SLAVE1, OUTPUT);
  pinMode(CS_SLAVE2, OUTPUT);
  digitalWrite(CS_SLAVE1, HIGH);
  digitalWrite(CS_SLAVE2, HIGH);

  // Initialize SPI
  hspi.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI);  // No CS needed in begin, we'll manage it manually

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Configure WebSocket
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Configure web server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Use send with FlashStringHelper to send PROGMEM data
    request->send(200, "text/html", FPSTR(index_html));
  });

  server.begin();
}

void loop() {
  // Communicate with Slave 1
  uint8_t dataToSlave1 = 0xAA;
  uint8_t dataFromSlave1 = spiTransfer(CS_SLAVE1, dataToSlave1);
  masterToSlave1 = String(dataToSlave1, HEX);
  slave1ToMaster = String(dataFromSlave1, HEX);
  Serial.print("Master to Slave 1: 0x");
  Serial.print(masterToSlave1);
  Serial.print(", Slave 1 to Master: 0x");
  Serial.println(slave1ToMaster);

  // Communicate with Slave 2
  uint8_t dataToSlave2 = 0xBB;
  uint8_t dataFromSlave2 = spiTransfer(CS_SLAVE2, dataToSlave2);
  masterToSlave2 = String(dataToSlave2, HEX);
  slave2ToMaster = String(dataFromSlave2, HEX);
  Serial.print("Master to Slave 2: 0x");
  Serial.print(masterToSlave2);
  Serial.print(", Slave 2 to Master: 0x");
  Serial.println(slave2ToMaster);

  // Create JSON using Arduino_JSON
  JSONVar jsonData;
  jsonData["m2s1"] = "0x" + masterToSlave1;
  jsonData["m2s2"] = "0x" + masterToSlave2;
  jsonData["s1m"] = "0x" + slave1ToMaster;
  jsonData["s2m"] = "0x" + slave2ToMaster;

  // Convert JSON to string and send via WebSocket
  String jsonString = JSON.stringify(jsonData);
  ws.textAll(jsonString);

  delay(1000);  // Update every second
}