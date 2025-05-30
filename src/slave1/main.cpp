#include <Arduino.h>
#include <driver/spi_slave.h>
#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Define SPI pins
#define HSPI_MOSI 11
#define HSPI_MISO 12
#define HSPI_SCLK 13
#define HSPI_CS   10

// Define NeoPixel pin and settings
#define NEOPIXEL_PIN 48  // GPIO for NeoPixel (adjust if needed)
#define NUM_PIXELS 1     // Number of NeoPixels (usually 1 for onboard RGB)

Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Buffer size for SPI communication
static const uint32_t BUFFER_SIZE = 1;  // Chỉ gửi/nhận 1 byte
uint8_t tx_buf[BUFFER_SIZE] = {0xDE};   // Response to master
uint8_t rx_buf[BUFFER_SIZE] = {0};

// SPI configuration
spi_slave_interface_config_t slvcfg;
spi_bus_config_t buscfg;

// Task để xử lý giao dịch SPI và điều khiển NeoPixel
void spiReceiveTask(void *pvParameters) {
  const TickType_t checkPeriod = pdMS_TO_TICKS(10); // Kiểm tra mỗi 10ms
  while (1) {
    // Chỉ thực hiện giao dịch SPI khi CS ở mức thấp
    if (digitalRead(HSPI_CS) == LOW) {
      Serial.println("Slave 1: CS pin LOW detected");

      // Prepare SPI transaction
      spi_slave_transaction_t trans;
      trans.length = BUFFER_SIZE * 8;  // Length in bits
      trans.tx_buffer = tx_buf;
      trans.rx_buffer = rx_buf;

      // Perform SPI transaction (blocking mode để đảm bảo nhận dữ liệu)
      esp_err_t ret = spi_slave_transmit(SPI2_HOST, &trans, portMAX_DELAY);
      if (ret == ESP_OK && trans.trans_len > 0) {
        Serial.print("Slave 1 received: 0x");
        Serial.println(rx_buf[0], HEX);

        // Control RGB LED based on command
        switch (rx_buf[0]) {
          case 0x00:  // Off
            pixels.setPixelColor(0, pixels.Color(0, 0, 0));
            pixels.show();
            Serial.println("Slave 1 RGB: Off");
            break;
          case 0x01:  // Red
            pixels.setPixelColor(0, pixels.Color(255, 0, 0));
            pixels.show();
            Serial.println("Slave 1 RGB: Red");
            break;
          case 0x02:  // Green
            pixels.setPixelColor(0, pixels.Color(0, 255, 0));
            pixels.show();
            Serial.println("Slave 1 RGB: Green");
            break;
          case 0x03:  // Blue
            pixels.setPixelColor(0, pixels.Color(0, 0, 255));
            pixels.show();
            Serial.println("Slave 1 RGB: Blue");
            break;
        }
      } else {
        Serial.println("Slave 1: No data received or SPI error");
      }
    }

    vTaskDelay(checkPeriod); // Chờ 10ms không chặn
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize NeoPixel
  pixels.begin();
  pixels.setBrightness(50);  // Set brightness (0-255)
  pixels.clear();
  pixels.show();

  // Configure SPI bus
  buscfg.mosi_io_num = HSPI_MOSI;
  buscfg.miso_io_num = HSPI_MISO;
  buscfg.sclk_io_num = HSPI_SCLK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;

  // Configure SPI slave interface
  slvcfg.mode = 0;  // SPI_MODE0
  slvcfg.spics_io_num = HSPI_CS;
  slvcfg.queue_size = 1;
  slvcfg.flags = 0;
  slvcfg.post_setup_cb = NULL;
  slvcfg.post_trans_cb = NULL;

  // Initialize SPI slave
  esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    Serial.println("Failed to initialize SPI slave");
    return;
  }

  Serial.println("Slave 1 SPI initialized");

  // Tạo task FreeRTOS để xử lý SPI
  xTaskCreatePinnedToCore(
    spiReceiveTask,    // Hàm task
    "SPIReceiveTask",  // Tên task
    4096,              // Kích thước stack
    NULL,              // Tham số truyền vào
    1,                 // Độ ưu tiên
    NULL,              // Handle của task
    1                  // Chạy trên core 1
  );
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(100)); // Chờ nhẹ để không chiếm CPU
}