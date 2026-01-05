// src/master/main.cpp


#include <Arduino.h>
#include "WifiPortal.h"
#include "MasterSpi.h"


// ==================== SETUP ====================
void setup() {
  // 1. Initialize WiFi Captive Portal
  WifiPortalSetup();
  
  delay(1000);  // Wait for WiFi to stabilize
  
  // 2. Initialize SPI Master
  MasterSpiSetup();
  
  Serial.println("\n========================================");
  Serial.println("  SYSTEM READY");
  Serial.println("========================================");
  Serial.println("WiFi AP: 'captive' @ http://4.3.2.1");
  Serial.println("SPI: 3 slaves ready for commands");
  Serial.println("Type 'help' for SPI commands");
  Serial.println("========================================\n");
}

// ==================== LOOP ====================
void loop() {
  // Process DNS requests for captive portal
  WifiPortalLoop();
  
  // Process SPI serial commands
  MasterSpiLoop();
  
  delay(10);
}