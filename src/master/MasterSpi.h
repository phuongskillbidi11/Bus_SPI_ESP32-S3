#ifndef MASTER_SPI_H
#define MASTER_SPI_H

#include <Arduino.h>
#include <SPI.h>

// ==================== SPI PINS ====================
#define HSPI_MOSI 35
#define HSPI_MISO 36
#define HSPI_SCLK 37
#define CS_SLAVE1 5
#define CS_SLAVE2 6
#define CS_SLAVE3 7

// ==================== SPI CONFIG ====================
#define SPI_FREQUENCY 4000000  // 4 MHz (tăng từ 1MHz)
#define BUFFER_SIZE 4

// ==================== GLOBAL VARIABLES ====================
SPIClass hspi(HSPI);
uint8_t tx_buf[BUFFER_SIZE] = {0};
uint8_t rx_buf[BUFFER_SIZE] = {0};

// ==================== SPI TRANSFER (OPTIMIZED) ====================
inline void spiTransferFast(uint8_t csPin, uint8_t *txData, uint8_t *rxData) {
  digitalWrite(csPin, LOW);
  delayMicroseconds(1);  // Giảm từ 10µs → 1µs
  
  rxData[0] = hspi.transfer(txData[0]);
  rxData[1] = hspi.transfer(txData[1]);
  rxData[2] = hspi.transfer(txData[2]);
  rxData[3] = hspi.transfer(txData[3]);
  
  delayMicroseconds(1);  // Giảm từ 10µs → 1µs
  digitalWrite(csPin, HIGH);
  delayMicroseconds(5);  // Giảm từ 50µs → 5µs
}

// ==================== NORMAL TRANSFER (FOR LED COMMANDS) ====================
void spiTransfer(uint8_t csPin, uint8_t *txData, uint8_t *rxData, size_t len) {
  digitalWrite(csPin, LOW);
  delayMicroseconds(10);
  
  for (size_t i = 0; i < len; i++) {
    rxData[i] = hspi.transfer(txData[i]);
  }
  
  delayMicroseconds(10);
  digitalWrite(csPin, HIGH);
  delayMicroseconds(50);
}

// ==================== LED COMMANDS ====================
void sendColorCommand(uint8_t slave, uint8_t color) {
  uint8_t csPin;
  
  switch(slave) {
    case 1: csPin = CS_SLAVE1; break;
    case 2: csPin = CS_SLAVE2; break;
    case 3: csPin = CS_SLAVE3; break;
    default:
      Serial.println("[SPI] ERROR: Invalid slave number!");
      return;
  }

  tx_buf[0] = color;
  tx_buf[1] = 0x00;
  tx_buf[2] = 0x00;
  tx_buf[3] = 0x00;

  spiTransfer(csPin, tx_buf, rx_buf, BUFFER_SIZE);

  Serial.printf("[SPI] Sent to Slave %d: 0x%02X\n", slave, color);
  Serial.printf("[SPI] Response: 0x%02X 0x%02X 0x%02X 0x%02X\n",
                rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
}

// ==================== BENCHMARK - MAXIMUM SPEED ====================
void benchmarkSPI(uint8_t slave, uint16_t iterations) {
  uint8_t csPin;
  
  switch(slave) {
    case 1: csPin = CS_SLAVE1; break;
    case 2: csPin = CS_SLAVE2; break;
    case 3: csPin = CS_SLAVE3; break;
    default: return;
  }
  
  Serial.printf("\n[BENCHMARK] Testing Slave %d - %d iterations\n", slave, iterations);
  Serial.println("[BENCHMARK] ========================================");
  
  tx_buf[0] = 0xAA;
  tx_buf[1] = 0x55;
  tx_buf[2] = 0xFF;
  tx_buf[3] = 0x00;
  
  unsigned long startTime = micros();
  
  for (uint16_t i = 0; i < iterations; i++) {
    spiTransferFast(csPin, tx_buf, rx_buf);
  }
  
  unsigned long endTime = micros();
  unsigned long totalTime = endTime - startTime;
  
  float avgTime = (float)totalTime / iterations;
  float throughput = (iterations * 1000000.0) / totalTime;
  float dataRate = (throughput * BUFFER_SIZE * 8) / 1000.0;
  
  Serial.println("[BENCHMARK] ========================================");
  Serial.printf("[BENCHMARK] Total Time:          %lu µs\n", totalTime);
  Serial.printf("[BENCHMARK] Avg per Transaction: %.2f µs\n", avgTime);
  Serial.printf("[BENCHMARK] Throughput:          %.2f trans/sec\n", throughput);
  Serial.printf("[BENCHMARK] Data Rate:           %.2f kbps\n", dataRate);
  Serial.printf("[BENCHMARK] Bytes Transferred:   %d bytes\n", iterations * BUFFER_SIZE);
  Serial.println("[BENCHMARK] ========================================\n");
}

void benchmarkAllSlaves(uint16_t iterations) {
  Serial.println("\n[BENCHMARK] ======== TESTING ALL SLAVES ========");
  
  for (int s = 1; s <= 3; s++) {
    benchmarkSPI(s, iterations);
    delay(500);
  }
  
  Serial.println("[BENCHMARK] ======== ALL TESTS COMPLETED ========\n");
}

// ==================== STRESS TEST - MAXIMUM THROUGHPUT ====================
void stressTestMax(uint16_t duration_seconds) {
  Serial.printf("\n[STRESS MAX] Running MAXIMUM SPEED test for %d seconds...\n", duration_seconds);
  Serial.println("[STRESS MAX] Using optimized timing (1µs setup/hold, 5µs delay)");
  Serial.println("[STRESS MAX] Non-LED commands (0xAA/0xBB/0xCC)\n");
  
  unsigned long startTime = millis();
  unsigned long endTime = startTime + (duration_seconds * 1000);
  uint32_t totalTransactions = 0;
  uint32_t errors = 0;
  
  uint8_t rx_s1[BUFFER_SIZE];
  uint8_t rx_s2[BUFFER_SIZE];
  uint8_t rx_s3[BUFFER_SIZE];
  
  // Pre-fill tx buffers
  uint8_t tx1[4] = {0xAA, 0x00, 0x00, 0x00};
  uint8_t tx2[4] = {0xBB, 0x00, 0x00, 0x00};
  uint8_t tx3[4] = {0xCC, 0x00, 0x00, 0x00};
  
  while (millis() < endTime) {
    // === Slave 1 (FAST) ===
    digitalWrite(CS_SLAVE1, LOW);
    delayMicroseconds(1);
    rx_s1[0] = hspi.transfer(tx1[0]);
    rx_s1[1] = hspi.transfer(tx1[1]);
    rx_s1[2] = hspi.transfer(tx1[2]);
    rx_s1[3] = hspi.transfer(tx1[3]);
    delayMicroseconds(1);
    digitalWrite(CS_SLAVE1, HIGH);
    delayMicroseconds(5);
    
    if (rx_s1[0] != 0xDE || rx_s1[1] != 0xAD) errors++;
    totalTransactions++;
    
    // === Slave 2 (FAST) ===
    digitalWrite(CS_SLAVE2, LOW);
    delayMicroseconds(1);
    rx_s2[0] = hspi.transfer(tx2[0]);
    rx_s2[1] = hspi.transfer(tx2[1]);
    rx_s2[2] = hspi.transfer(tx2[2]);
    rx_s2[3] = hspi.transfer(tx2[3]);
    delayMicroseconds(1);
    digitalWrite(CS_SLAVE2, HIGH);
    delayMicroseconds(5);
    
    if (rx_s2[0] != 0xCA || rx_s2[1] != 0xFE) errors++;
    totalTransactions++;
    
    // === Slave 3 (FAST) ===
    digitalWrite(CS_SLAVE3, LOW);
    delayMicroseconds(1);
    rx_s3[0] = hspi.transfer(tx3[0]);
    rx_s3[1] = hspi.transfer(tx3[1]);
    rx_s3[2] = hspi.transfer(tx3[2]);
    rx_s3[3] = hspi.transfer(tx3[3]);
    delayMicroseconds(1);
    digitalWrite(CS_SLAVE3, HIGH);
    delayMicroseconds(5);
    
    if (rx_s3[0] != 0xBE || rx_s3[1] != 0xEF) errors++;
    totalTransactions++;
  }
  
  unsigned long actualDuration = millis() - startTime;
  float avgRate = (totalTransactions * 1000.0) / actualDuration;
  float errorRate = (errors * 100.0) / totalTransactions;
  
  Serial.println("\n[STRESS MAX] ========================================");
  Serial.printf("[STRESS MAX] Duration:         %lu ms\n", actualDuration);
  Serial.printf("[STRESS MAX] Total Trans:      %u\n", totalTransactions);
  Serial.printf("[STRESS MAX] Errors:           %u\n", errors);
  Serial.printf("[STRESS MAX] Error Rate:       %.4f%%\n", errorRate);
  Serial.printf("[STRESS MAX] Avg Rate:         %.2f trans/sec\n", avgRate);
  Serial.printf("[STRESS MAX] Peak Throughput:  %.2f kbps\n", (avgRate * 32) / 1000.0);
  Serial.println("[STRESS MAX] ========================================\n");
}

// ==================== STRESS TEST - WITH LED ====================
void stressTestLED(uint16_t duration_seconds) {
  Serial.printf("\n[STRESS LED] Running LED test for %d seconds...\n", duration_seconds);
  Serial.println("[STRESS LED] Using LED commands with 150µs delay\n");
  
  unsigned long startTime = millis();
  unsigned long endTime = startTime + (duration_seconds * 1000);
  uint32_t totalTransactions = 0;
  uint32_t errors = 0;
  
  uint8_t rx_s1[BUFFER_SIZE];
  uint8_t rx_s2[BUFFER_SIZE];
  uint8_t rx_s3[BUFFER_SIZE];
  
  while (millis() < endTime) {
    uint8_t color = random(0, 4);
    
    // === Slave 1 ===
    tx_buf[0] = color;
    tx_buf[1] = tx_buf[2] = tx_buf[3] = 0x00;
    
    digitalWrite(CS_SLAVE1, LOW);
    delayMicroseconds(10);
    for (int i = 0; i < 4; i++) rx_s1[i] = hspi.transfer(tx_buf[i]);
    delayMicroseconds(10);
    digitalWrite(CS_SLAVE1, HIGH);
    delayMicroseconds(150);  // Extra delay for LED
    
    if (rx_s1[0] != 0xDE) errors++;
    totalTransactions++;
    
    // === Slave 2 ===
    digitalWrite(CS_SLAVE2, LOW);
    delayMicroseconds(10);
    for (int i = 0; i < 4; i++) rx_s2[i] = hspi.transfer(tx_buf[i]);
    delayMicroseconds(10);
    digitalWrite(CS_SLAVE2, HIGH);
    delayMicroseconds(150);
    
    if (rx_s2[0] != 0xCA) errors++;
    totalTransactions++;
    
    // === Slave 3 ===
    digitalWrite(CS_SLAVE3, LOW);
    delayMicroseconds(10);
    for (int i = 0; i < 4; i++) rx_s3[i] = hspi.transfer(tx_buf[i]);
    delayMicroseconds(10);
    digitalWrite(CS_SLAVE3, HIGH);
    delayMicroseconds(150);
    
    if (rx_s3[0] != 0xBE) errors++;
    totalTransactions++;
  }
  
  unsigned long actualDuration = millis() - startTime;
  float avgRate = (totalTransactions * 1000.0) / actualDuration;
  float errorRate = (errors * 100.0) / totalTransactions;
  
  Serial.println("\n[STRESS LED] ========================================");
  Serial.printf("[STRESS LED] Duration:         %lu ms\n", actualDuration);
  Serial.printf("[STRESS LED] Total Trans:      %u\n", totalTransactions);
  Serial.printf("[STRESS LED] Errors:           %u\n", errors);
  Serial.printf("[STRESS LED] Error Rate:       %.4f%%\n", errorRate);
  Serial.printf("[STRESS LED] Avg Rate:         %.2f trans/sec\n", avgRate);
  Serial.println("[STRESS LED] ========================================\n");
}

// ==================== HELP ====================
void printHelp() {
  Serial.println("\n========== SPI TEST COMMANDS (OPTIMIZED) ==========");
  Serial.println("1. LED Control:");
  Serial.println("   s1r/s1g/s1b/s1o - Slave 1 RED/GREEN/BLUE/OFF");
  Serial.println("   s2r/s2g/s2b/s2o - Slave 2 RED/GREEN/BLUE/OFF");
  Serial.println("   s3r/s3g/s3b/s3o - Slave 3 RED/GREEN/BLUE/OFF");
  Serial.println("");
  Serial.println("2. Benchmark (4 MHz SPI):");
  Serial.println("   b1/b2/b3 - Benchmark individual slaves");
  Serial.println("   ball     - Benchmark all slaves");
  Serial.println("   bfast    - Fast benchmark (10k iterations)");
  Serial.println("");
  Serial.println("3. Stress Tests:");
  Serial.println("   max [sec]  - MAXIMUM SPEED test (no LED)");
  Serial.println("   led [sec]  - LED test (with random colors)");
  Serial.println("");
  Serial.println("4. Batch LED:");
  Serial.println("   allr/allg/allb/allo - All slaves same color");
  Serial.println("==================================================");
  Serial.printf("SPI Frequency: %d MHz\n", SPI_FREQUENCY / 1000000);
  Serial.println("==================================================\n");
}

// ==================== COMMAND HANDLER ====================
void handleSerialCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  
  // LED Commands
  if (cmd == "s1r") sendColorCommand(1, 0x01);
  else if (cmd == "s1g") sendColorCommand(1, 0x02);
  else if (cmd == "s1b") sendColorCommand(1, 0x03);
  else if (cmd == "s1o") sendColorCommand(1, 0x00);
  
  else if (cmd == "s2r") sendColorCommand(2, 0x01);
  else if (cmd == "s2g") sendColorCommand(2, 0x02);
  else if (cmd == "s2b") sendColorCommand(2, 0x03);
  else if (cmd == "s2o") sendColorCommand(2, 0x00);
  
  else if (cmd == "s3r") sendColorCommand(3, 0x01);
  else if (cmd == "s3g") sendColorCommand(3, 0x02);
  else if (cmd == "s3b") sendColorCommand(3, 0x03);
  else if (cmd == "s3o") sendColorCommand(3, 0x00);
  
  // Batch Commands
  else if (cmd == "allr") {
    for (int s = 1; s <= 3; s++) { sendColorCommand(s, 0x01); delay(50); }
  }
  else if (cmd == "allg") {
    for (int s = 1; s <= 3; s++) { sendColorCommand(s, 0x02); delay(50); }
  }
  else if (cmd == "allb") {
    for (int s = 1; s <= 3; s++) { sendColorCommand(s, 0x03); delay(50); }
  }
  else if (cmd == "allo") {
    for (int s = 1; s <= 3; s++) { sendColorCommand(s, 0x00); delay(50); }
  }
  
  // Benchmarks
  else if (cmd == "b1") benchmarkSPI(1, 1000);
  else if (cmd == "b2") benchmarkSPI(2, 1000);
  else if (cmd == "b3") benchmarkSPI(3, 1000);
  else if (cmd == "ball") benchmarkAllSlaves(1000);
  else if (cmd == "bfast") benchmarkAllSlaves(10000);
  
  // Stress Tests
  else if (cmd.startsWith("max")) {
    int duration = 10;
    int spaceIndex = cmd.indexOf(' ');
    if (spaceIndex > 0) {
      duration = cmd.substring(spaceIndex + 1).toInt();
      if (duration < 1) duration = 10;
    }
    stressTestMax(duration);
  }
  else if (cmd.startsWith("led")) {
    int duration = 10;
    int spaceIndex = cmd.indexOf(' ');
    if (spaceIndex > 0) {
      duration = cmd.substring(spaceIndex + 1).toInt();
      if (duration < 1) duration = 10;
    }
    stressTestLED(duration);
  }
  
  // Help
  else if (cmd == "help" || cmd == "h" || cmd == "?") {
    printHelp();
  }
  
  else {
    Serial.println("[SPI] Unknown command. Type 'help'");
  }
}

// ==================== SETUP ====================
void MasterSpiSetup() {
  Serial.println("\n========================================");
  Serial.println("  ESP32-S3 SPI MASTER - OPTIMIZED");
  Serial.println("========================================\n");
  
  pinMode(CS_SLAVE1, OUTPUT);
  pinMode(CS_SLAVE2, OUTPUT);
  pinMode(CS_SLAVE3, OUTPUT);
  digitalWrite(CS_SLAVE1, HIGH);
  digitalWrite(CS_SLAVE2, HIGH);
  digitalWrite(CS_SLAVE3, HIGH);
  
  hspi.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI);
  hspi.setFrequency(SPI_FREQUENCY);
  hspi.setDataMode(SPI_MODE0);
  hspi.setBitOrder(MSBFIRST);
  
  Serial.println("[SPI] ✓ Master initialized");
  Serial.printf("[SPI] Frequency: %d MHz\n", SPI_FREQUENCY / 1000000);
  Serial.printf("[SPI] CS Slaves: GPIO %d, %d, %d\n", CS_SLAVE1, CS_SLAVE2, CS_SLAVE3);
  
  printHelp();
}

void MasterSpiLoop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleSerialCommand(cmd);
  }
}

#endif // MASTER_SPI_H