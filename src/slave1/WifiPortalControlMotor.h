#include <Arduino.h>  //not needed in the arduino ide
#include <WiFi.h>
// Captive Portal
#include <AsyncTCP.h>  //https://github.com/me-no-dev/AsyncTCP using the latest dev version from @me-no-dev
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>	//https://github.com/me-no-dev/ESPAsyncWebServer using the latest dev version from @me-no-dev
#include <esp_wifi.h>			//Used for mpdu_rx_disable android workaround
#include <LittleFS.h>
#include "Control_servo.h"

// Pre reading on the fundamentals of captive portals https://textslashplain.com/2022/06/24/captive-portals/

const char *ssid = "[Slave1] - Control Motor";  // FYI The SSID can't have a space in it.
// const char * password = "12345678"; //Atleast 8 chars
const char *password = NULL;  // no password

#define MAX_CLIENTS 4	// ESP32 supports up to 10 but I have not tested it yet
#define WIFI_CHANNEL 6	// 2.4ghz channel 6 https://en.wikipedia.org/wiki/List_of_WLAN_channels#2.4_GHz_(802.11b/g/n/ax)

const IPAddress localIP(4, 3, 2, 1);		   // the IP address the web server, Samsung requires the IP to be in public space
const IPAddress gatewayIP(4, 3, 2, 1);		   // IP address of the network should be the same as the local IP for captive portals
const IPAddress subnetMask(255, 255, 255, 0);  // no need to change: https://avinetworks.com/glossary/subnet-mask/

const String localIPURL = "http://4.3.2.1";	 // a string version of the local IP with http, used for redirecting clients to your webpage

// const char index_html[] PROGMEM = R"=====(
//   <!DOCTYPE html> <html>
//     <head>
//       <title>ESP32 Captive Portal</title>
//       <style>
//         body {background-color:#06cc13;}
//         h1 {color: white;}
//         h2 {color: white;}
//       </style>
//       <meta name="viewport" content="width=device-width, initial-scale=1.0">
//     </head>
//     <body>
//       <h1>Hello World!</h1>
//       <h2>This is a captive portal example. All requests will be redirected here </h2>
//     </body>
//   </html>
// )=====";

DNSServer dnsServer;
AsyncWebServer server(80);

void setUpDNSServer(DNSServer &dnsServer, const IPAddress &localIP) {
// Define the DNS interval in milliseconds between processing DNS requests
#define DNS_INTERVAL 30

	// Set the TTL for DNS response and start the DNS server
	// dnsServer.setTTL(3600);
	// dnsServer.start(53, "*", localIP);
	  Serial.println("[DNS] Starting DNS Server...");
  
  dnsServer.setTTL(3600);
  dnsServer.start(53, "*", localIP);
  
  Serial.printf("[DNS] ✓ DNS Server started on %s:53\n", localIP.toString().c_str());
  Serial.println("[DNS] Redirecting all domains to captive portal");
}
void startSoftAccessPoint(const char *ssid, const char *password, 
                          const IPAddress &localIP, const IPAddress &gatewayIP) {
  Serial.println("[WiFi] Initializing Access Point...");
  
  // 1. Set mode
  WiFi.mode(WIFI_MODE_AP);
  Serial.println("[WiFi] Mode set to AP");
  
  // 2. Configure IP
  const IPAddress subnetMask(255, 255, 255, 0);
  if (WiFi.softAPConfig(localIP, gatewayIP, subnetMask)) {
    Serial.println("[WiFi] IP config successful");
    Serial.printf("[WiFi] IP: %s\n", localIP.toString().c_str());
  } else {
    Serial.println("[WiFi] ERROR: IP config failed!");
    return;
  }
  
  // 3. Start AP
  Serial.printf("[WiFi] Starting AP: SSID='%s'\n", ssid);
  bool apStarted = WiFi.softAP(ssid, password, WIFI_CHANNEL, 0, MAX_CLIENTS);
  
  if (apStarted) {
    Serial.println("[WiFi] ✓ AP Started Successfully!");
    
    // 4. Wait for AP to be ready
    delay(100);
    
    // 5. Verify
    IPAddress myIP = WiFi.softAPIP();
    Serial.printf("[WiFi] AP IP Address: %s\n", myIP.toString().c_str());
    Serial.printf("[WiFi] SSID: %s\n", ssid);
    Serial.printf("[WiFi] Channel: %d\n", WIFI_CHANNEL);
    Serial.printf("[WiFi] Max Clients: %d\n", MAX_CLIENTS);
    
    if (myIP == IPAddress(0, 0, 0, 0)) {
      Serial.println("[WiFi] ⚠ WARNING: IP is 0.0.0.0 - AP may not be working!");
    }
  } else {
    Serial.println("[WiFi] ✗ ERROR: AP Start Failed!");
    Serial.println("[WiFi] Possible reasons:");
    Serial.println("[WiFi]   - Invalid SSID");
    Serial.println("[WiFi]   - Channel conflict");
    Serial.println("[WiFi]   - Hardware issue");
  }
}


void setUpWebserver(AsyncWebServer &server, const IPAddress &localIP) {
	//======================== Webserver ========================
	// WARNING IOS (and maybe macos) WILL NOT POP UP IF IT CONTAINS THE WORD "Success" https://www.esp8266.com/viewtopic.php?f=34&t=4398
	// SAFARI (IOS) IS STUPID, G-ZIPPED FILES CAN'T END IN .GZ https://github.com/homieiot/homie-esp8266/issues/476 this is fixed by the webserver serve static function.
	// SAFARI (IOS) there is a 128KB limit to the size of the HTML. The HTML can reference external resources/images that bring the total over 128KB
	// SAFARI (IOS) popup browser has some severe limitations (javascript disabled, cookies disabled)

	// Required
	server.on("/connecttest.txt", [](AsyncWebServerRequest *request) { request->redirect("http://logout.net"); });	// windows 11 captive portal workaround
	server.on("/wpad.dat", [](AsyncWebServerRequest *request) { request->send(404); });								// Honestly don't understand what this is but a 404 stops win 10 keep calling this repeatedly and panicking the esp32 :)

	// Background responses: Probably not all are Required, but some are. Others might speed things up?
	// A Tier (commonly used by modern systems)
	server.on("/generate_204", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });		   // android captive portal redirect
	server.on("/redirect", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });			   // microsoft redirect
	server.on("/hotspot-detect.html", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });  // apple call home
	server.on("/canonical.html", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });	   // firefox captive portal call home
	server.on("/success.txt", [](AsyncWebServerRequest *request) { request->send(200); });					   // firefox captive portal call home
	server.on("/ncsi.txt", [](AsyncWebServerRequest *request) { request->redirect(localIPURL); });			   // windows call home
  
	// B Tier (uncommon)
	//  server.on("/chrome-variations/seed",[](AsyncWebServerRequest *request){request->send(200);}); //chrome captive portal call home
	//  server.on("/service/update2/json",[](AsyncWebServerRequest *request){request->send(200);}); //firefox?
	//  server.on("/chat",[](AsyncWebServerRequest *request){request->send(404);}); //No stop asking Whatsapp, there is no internet connection
	//  server.on("/startpage",[](AsyncWebServerRequest *request){request->redirect(localIPURL);});

	// return 404 to webpage icon
	server.on("/favicon.ico", [](AsyncWebServerRequest *request) { request->send(404); });	// webpage icon
    server.on("/servo", HTTP_GET, [](AsyncWebServerRequest *request) {
      if (request->hasParam("num") && request->hasParam("angle")) {
          int servoNum = request->getParam("num")->value().toInt();
          int angle = request->getParam("angle")->value().toInt();
          
          // Validate parameters
          if (servoNum >= 1 && servoNum <= 3 && angle >= 0 && angle <= 180) {
              ServoSetAngle(servoNum, angle, false);
              
              Serial.printf("[API] Servo %d → %d°\n", servoNum, angle);
              request->send(200, "text/plain", "OK");
          } else {
              request->send(400, "text/plain", "Invalid parameters");
          }
      } else {
          request->send(400, "text/plain", "Missing parameters");
      }
  });

	// // Serve Basic HTML Page
	// server.on("/", HTTP_ANY, [](AsyncWebServerRequest *request) {
	// 	AsyncWebServerResponse *response = request->beginResponse(200, "text/html", index_html);
	// 	response->addHeader("Cache-Control", "public,max-age=31536000");  // save this file to cache for 1 year (unless you refresh)
	// 	request->send(response);
	// 	Serial.println("Served Basic HTML Page");
	// });

	// // the catch all
	// server.onNotFound([](AsyncWebServerRequest *request) {
	// 	request->redirect(localIPURL);
	// 	Serial.print("onnotfound ");
	// 	Serial.print(request->host());	// This gives some insight into whatever was being requested on the serial monitor
	// 	Serial.print(" ");
	// 	Serial.print(request->url());
	// 	Serial.print(" sent redirect to " + localIPURL + "\n");
	// });

 

    // ========== SERVE STATIC FILES FROM LITTLEFS ==========
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    
    // ========== CATCH ALL ==========
    server.onNotFound([](AsyncWebServerRequest *request) {
        String path = request->url();
        if (path == "/") path = "/index.html";
        
        // Try to serve from LittleFS
        if (LittleFS.exists(path)) {
            request->send(LittleFS, path);
        } else {
            request->redirect(localIPURL);
        }
        
        Serial.printf("[Web] %s %s\n", request->host().c_str(), request->url().c_str());
    });
}


void WifiPortalSetup() {
    Serial.setTxBufferSize(1024);
    Serial.begin(115200);
    while (!Serial);
    
    Serial.println("\n========================================");
    Serial.println("  CAPTIVE PORTAL INITIALIZATION");
    Serial.println("========================================");
    Serial.printf("Compiled: %s %s\n", __DATE__, __TIME__);
    Serial.printf("Chip: %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.println("========================================\n");
    
    // ========== INITIALIZE LITTLEFS ==========
    Serial.println("[FS] Mounting LittleFS...");
    if (!LittleFS.begin(true)) {
        Serial.println("[FS] ✗ ERROR: LittleFS Mount Failed!");
        Serial.println("[FS] Web files will not be available");
    } else {
        Serial.println("[FS] ✓ LittleFS Mounted Successfully");
        
        // List files in root directory
        File root = LittleFS.open("/");
        if (!root || !root.isDirectory()) {
            Serial.println("[FS] Failed to open root directory");
        } else {
            Serial.println("[FS] Files in LittleFS:");
            File file = root.openNextFile();
            int fileCount = 0;
            while (file) {
                Serial.printf("  - %-20s %6d bytes\n", file.name(), file.size());
                fileCount++;
                file = root.openNextFile();
            }
            if (fileCount == 0) {
                Serial.println("  (no files found - did you upload filesystem?)");
            }
        }
    }
    
    // ========== START ACCESS POINT ==========
    startSoftAccessPoint(ssid, password, localIP, gatewayIP);
    delay(500);
    
    // ========== START DNS SERVER ==========
    setUpDNSServer(dnsServer, localIP);
    
    // ========== SETUP WEBSERVER ==========
    setUpWebserver(server, localIP);
    server.begin();
    Serial.println("[Web] ✓ Web server started on port 80\n");
    
    Serial.println("========================================");
    Serial.println("  CAPTIVE PORTAL READY");
    Serial.println("========================================");
    Serial.printf("Connect to WiFi: '%s'\n", ssid);
    Serial.printf("Open browser: http://%s\n", localIP.toString().c_str());
    Serial.println("========================================\n");
    
    Serial.printf("Startup Time: %d ms\n\n", millis());
}

void WifiPortalLoop() {
	dnsServer.processNextRequest();	 // I call this atleast every 10ms in my other projects (can be higher but I haven't tested it for stability)
	delay(DNS_INTERVAL);			 // seems to help with stability, if you are doing other things in the loop this may not be needed
}