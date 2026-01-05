// src/slave3/main.cpp - OPTIMIZED FOR MAXIMUM SPEED
#include <Arduino.h>
#include <driver/spi_slave.h>
#include <Adafruit_NeoPixel.h>

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
DRAM_ATTR uint8_t tx_buf[BUFFER_SIZE] = {0xBE, 0xEF, 0x00, 0x00};  // ← SLAVE 3 SIGNATURE
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
      
      // Set LED flag (non-blocking)
      setLED(rx_buf[0]);
      
      // Update response
      tx_buf[0] = 0xBE;  // ← SLAVE 3 SIGNATURE
      tx_buf[1] = 0xEF;  // ← SLAVE 3 SIGNATURE
      tx_buf[2] = (counter >> 8) & 0xFF;
      tx_buf[3] = counter & 0xFF;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("     ESP32-S3 SPI SLAVE 3");
  Serial.println("     OPTIMIZED - 4MHz CAPABLE");
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
  
  Serial.println("SPI Slave 3 initialized");
  Serial.printf("CS Pin: GPIO %d\n", HSPI_CS);
  Serial.println("Signature: 0xBEEF");
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
  
  last_report_time = millis();
}

void loop() {
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
  
  // Report throughput every 5 seconds
  if (millis() - last_report_time >= 5000) {
    uint32_t count = transaction_count;
    uint32_t delta = count - last_report_count;
    float rate = (delta * 1000.0) / 5000.0;
    
    Serial.printf("[SLAVE3] Transactions: %u | Rate: %.2f trans/sec\n", count, rate);
    
    last_report_count = count;
    last_report_time = millis();
  }
  
  vTaskDelay(pdMS_TO_TICKS(10));
}