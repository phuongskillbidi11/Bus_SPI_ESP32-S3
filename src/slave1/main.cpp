// src/slave1/main.cpp - WITH SERVO CONTROL
#include <Arduino.h>
#include <driver/spi_slave.h>
#include <Adafruit_NeoPixel.h>
#include "Control_servo.h" 
#include "WifiPortalControlMotor.h"

// ==================== PINS ====================
#define HSPI_MOSI 35
#define HSPI_MISO 36
#define HSPI_SCLK 37
#define HSPI_CS   46

#define NEOPIXEL_PIN 48
#define NUM_PIXELS 1

Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ==================== BUFFERS ====================
#define BUFFER_SIZE 4
DRAM_ATTR uint8_t tx_buf[BUFFER_SIZE] = {0xDE, 0xAD, 0x00, 0x00};
DRAM_ATTR uint8_t rx_buf[BUFFER_SIZE] = {0};

// ==================== LED STATE ====================
volatile bool led_update_pending = false;
volatile uint8_t pending_color = 0;

// ==================== STATISTICS ====================
volatile uint32_t transaction_count = 0;
volatile uint32_t last_report_count = 0;
unsigned long last_report_time = 0;

// ==================== LED CONTROL ====================
void setLED(uint8_t cmd) {
  pending_color = cmd;
  led_update_pending = true;
}

// ==================== SPI TASK ====================
void spiReceiveTask(void *pvParameters) {
  spi_slave_transaction_t trans;
  uint32_t counter = 0;
  
  while (1) {
    memset(&trans, 0, sizeof(trans));
    trans.length = BUFFER_SIZE * 8;
    trans.tx_buffer = tx_buf;
    trans.rx_buffer = rx_buf;
    
    esp_err_t ret = spi_slave_transmit(SPI2_HOST, &trans, portMAX_DELAY);
    
    if (ret == ESP_OK && trans.trans_len > 0) {
      counter++;
      transaction_count++;
      
      // LED control (existing)
      if (rx_buf[0] <= 0x03) {
        setLED(rx_buf[0]);
      }
      
      // SERVO CONTROL via SPI (NEW)
      // Format: rx_buf[0]=0x10/0x11, rx_buf[1]=servo_num, rx_buf[2]=angle
      else if (rx_buf[0] >= 0x10 && rx_buf[0] <= 0x13) {
        uint8_t servo_num = rx_buf[1];  // 1, 2, or 3
        uint8_t angle = rx_buf[2];      // 0-180
        bool reverse = (rx_buf[0] == 0x11 || rx_buf[0] == 0x13);
        
        if (servo_num >= 1 && servo_num <= 3) {
          ServoSetAngle(servo_num, angle, reverse);
        }
      }
      
      // Update response
      tx_buf[0] = 0xDE;
      tx_buf[1] = 0xAD;
      tx_buf[2] = (counter >> 8) & 0xFF;
      tx_buf[3] = counter & 0xFF;
    }
  }
}

// ==================== SERIAL COMMAND HANDLER ====================
void handleSerialCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  
  // Help
  if (cmd == "help" || cmd == "h" || cmd == "?") {
    ServoPrintHelp();
    return;
  }
  
  // Status
  if (cmd == "status") {
    ServoPrintStatus();
    return;
  }
  
  // Stop all
  if (cmd == "stop") {
    ServoStopAll();
    return;
  }
  
  // Test positions: test1, test2, test3
  if (cmd.startsWith("test")) {
    uint8_t servo_num = cmd.substring(4).toInt();
    if (servo_num >= 1 && servo_num <= 3) {
      ServoTestPositions(servo_num);
    } else {
      Serial.println("[CMD] Usage: test1, test2, or test3");
    }
    return;
  }
  
  // Sweep: sweep1, sweep2, sweep3
  if (cmd.startsWith("sweep")) {
    uint8_t servo_num = cmd.substring(5).toInt();
    if (servo_num >= 1 && servo_num <= 3) {
      ServoSweep(servo_num, 0, 180, 20); // 0-180°, 20ms delay
    } else {
      Serial.println("[CMD] Usage: sweep1, sweep2, or sweep3");
    }
    return;
  }
  
  // Servo control: s1:90, s2:45r, s3:180
  if (cmd.startsWith("s") && cmd.indexOf(':') > 0) {
    int colonPos = cmd.indexOf(':');
    uint8_t servo_num = cmd.substring(1, colonPos).toInt();
    String angleStr = cmd.substring(colonPos + 1);
    
    bool reverse = angleStr.endsWith("r");
    bool smooth = angleStr.endsWith("s");  // s1:90s = smooth
    if (reverse) angleStr.remove(angleStr.length() - 1);
    if (smooth) angleStr.remove(angleStr.length() - 1);
    
    float angle = angleStr.toFloat();
    
    if (servo_num >= 1 && servo_num <= 3) {
      if (smooth) {
        ServoMoveSmooth(servo_num, angle, 500); // 500ms smooth
      } else {
        ServoSetAngle(servo_num, angle, reverse);
      }
    } else {
      Serial.println("[CMD] Invalid servo number (1-3)");
    }
    return;
  }
  
  // Raw PWM: pwm1:300, pwm2:400
  if (cmd.startsWith("pwm") && cmd.indexOf(':') > 0) {
    int colonPos = cmd.indexOf(':');
    uint8_t servo_num = cmd.substring(3, colonPos).toInt();
    uint16_t pwm = cmd.substring(colonPos + 1).toInt();
    
    if (servo_num >= 1 && servo_num <= 3) {
      ServoSetPWM(servo_num, pwm);
    } else {
      Serial.println("[CMD] Invalid servo number (1-3)");
    }
    return;
  }
  
  // Reverse mode: rev1:1, rev2:0
  if (cmd.startsWith("rev") && cmd.indexOf(':') > 0) {
    int colonPos = cmd.indexOf(':');
    uint8_t servo_num = cmd.substring(3, colonPos).toInt();
    bool reverse = cmd.substring(colonPos + 1).toInt() == 1;
    
    if (servo_num >= 1 && servo_num <= 3) {
      ServoSetReverse(servo_num, reverse);
    } else {
      Serial.println("[CMD] Invalid servo number (1-3)");
    }
    return;
  }
  
  // Rotate by angle (continuous servo): rot2:90:50
  if (cmd.startsWith("rot")) {
    int colon1 = cmd.indexOf(':');
    int colon2 = cmd.indexOf(':', colon1 + 1);
    
    if (colon1 > 0 && colon2 > 0) {
      uint8_t servo_num = cmd.substring(3, colon1).toInt();
      float angle = cmd.substring(colon1 + 1, colon2).toFloat();
      uint8_t speed = cmd.substring(colon2 + 1).toInt();
      
      if (servo_num >= 1 && servo_num <= 3) {
        ServoRotateByAngle(servo_num, angle, speed);
      } else {
        Serial.println("[CMD] Invalid servo number (1-3)");
      }
    } else {
      Serial.println("[CMD] Usage: rot2:90:50 (servo 2, rotate 90°, 50% speed)");
    }
    return;
  }
  
  // Unknown command
  Serial.println("[CMD] Unknown command. Type 'help' for commands.");
}

// ==================== SETUP ====================
void setup() {
  // Serial.begin(115200);
  WifiPortalSetup();
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("     ESP32-S3 SPI SLAVE 1");
  Serial.println("     WITH SERVO CONTROL");
  Serial.println("========================================\n");
  
  // Initialize LED
  pixels.begin();
  pixels.setBrightness(50);
  pixels.clear();
  pixels.show();
  
  // Configure SPI bus
  spi_bus_config_t buscfg = {
    .mosi_io_num = HSPI_MOSI,
    .miso_io_num = HSPI_MISO,
    .sclk_io_num = HSPI_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4092
  };
  
  // Configure SPI slave
  spi_slave_interface_config_t slvcfg = {
    .spics_io_num = HSPI_CS,
    .flags = 0,
    .queue_size = 3,
    .mode = 0,
    .post_setup_cb = NULL,
    .post_trans_cb = NULL
  };
  
  esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    Serial.println("ERROR: SPI initialization failed!");
    while(1);
  }
  
  Serial.println("SPI Slave 1 initialized");
  Serial.printf("CS Pin: GPIO %d\n", HSPI_CS);
  Serial.println("Signature: 0xDEAD");
  Serial.println("Waiting for Master...\n");
  
  // Create SPI task
  xTaskCreatePinnedToCore(
    spiReceiveTask,
    "SPI_RX",
    4096,
    NULL,
    2,
    NULL,
    1
  );
  
  // Initialize SERVO (NEW)
  if (!ServoInit()) {
    Serial.println("[SERVO] ERROR: Init failed!");
  }
  
  Serial.println("\n========================================");
  Serial.println("Type 'help' for servo commands");
  Serial.println("========================================\n");
  
  last_report_time = millis();
}

// ==================== LOOP ====================
void loop() {
  WifiPortalLoop();
  // Update LED if pending
  if (led_update_pending) {
    led_update_pending = false;
    
    switch (pending_color) {
      case 0x00: pixels.setPixelColor(0, 0, 0, 0); break;
      case 0x01: pixels.setPixelColor(0, 255, 0, 0); break;
      case 0x02: pixels.setPixelColor(0, 0, 255, 0); break;
      case 0x03: pixels.setPixelColor(0, 0, 0, 255); break;
    }
    pixels.show();
  }
  
  // Handle serial commands (NEW)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleSerialCommand(cmd);
  }
  
  // Report throughput every 5 seconds
  if (millis() - last_report_time >= 5000) {
    uint32_t count = transaction_count;
    uint32_t delta = count - last_report_count;
    float rate = (delta * 1000.0) / 5000.0;
    
    Serial.printf("[SLAVE1] Transactions: %u | Rate: %.2f trans/sec\n", count, rate);
    
    last_report_count = count;
    last_report_time = millis();
  }
  
  vTaskDelay(pdMS_TO_TICKS(10));
}