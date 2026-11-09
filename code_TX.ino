// OLED SSD1306 128x64 I2C, không dành cho SH1106.
#define SERIAL_BAUD 115200
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_ADDRESS 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET -1
#define ESPNOW_CHANNEL 1  // Phải giống node gửi.
#define MESSAGE_BUFFER_SIZE 64
#define DEBUG_INTERVAL_MS 2000UL
#define I2C_TIMEOUT_MS 50

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <esp_err.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/Org_01.h>
#include <string.h>

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
const char longestMessage[] = "Muc pin con lai la : 100%";
char receivedMessage[MESSAGE_BUFFER_SIZE] = {};
portMUX_TYPE receiveMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool newMessage = false;
bool ready = false;
const char *startupStage = "Serial";
uint32_t lastDebugMs = 0;
uint32_t lastPacketMs = 0;
bool hasReceivedPacket = false;

// Core 3.x: const esp_now_recv_info_t *info.
// Core 2.x: const uint8_t *mac. #if tự chọn khi đổi phiên bản core.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
#else
void onDataReceived(const uint8_t *mac, const uint8_t *data, int len);
void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
#endif
  if (data == nullptr || len <= 0 || len >= MESSAGE_BUFFER_SIZE) {
    return;  // Bỏ gói không hợp lệ, không hiển thị chuỗi bị cắt.
  }
  // Callback chỉ sao chép và đặt cờ, không Serial/OLED/I2C.
  portENTER_CRITICAL(&receiveMux);
  memcpy(receivedMessage, data, static_cast<size_t>(len));
  receivedMessage[len] = '\0';
  newMessage = true;
  portEXIT_CRITICAL(&receiveMux);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("\n[BOOT] NODE NHAN - Serial 115200");
  startupStage = "Wire.begin: kiem tra SDA21, SCL22";
  if (!Wire.begin(OLED_SDA, OLED_SCL)) {
    Serial.println("Loi khoi tao I2C.");
    return;
  }
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  startupStage = "I2C ACK: kiem tra nguon 3.3V, day va OLED_ADDRESS";
  Wire.beginTransmission(OLED_ADDRESS);
  const uint8_t i2cError = Wire.endTransmission();
  if (i2cError != 0) {
    Serial.printf("[LOI] I2C 0x%02X khong ACK, ma loi = %u.\n",
                  OLED_ADDRESS, static_cast<unsigned>(i2cError));
    return;
  }
  Serial.printf("[OK] I2C 0x%02X co ACK.\n", OLED_ADDRESS);
  startupStage = "display.begin SSD1306";
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("Loi display.begin: khong khoi tao duoc SSD1306.");
    return;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1, 2);  // Cao gấp đôi, rộng giữ nguyên để đủ một dòng.
  display.setFont(&Org_01);
  display.setTextWrap(false);  // Luôn đúng một dòng.
  display.display();

  startupStage = "kiem tra kich thuoc font";
  // Org_01: chuỗi 100% rộng 116 px, vẫn vừa trong 128 px.
  int16_t x1, y1;
  uint16_t width, height;
  display.getTextBounds(longestMessage, 0, 0, &x1, &y1, &width, &height);
  Serial.printf("Org_01: chuoi 100%% = %u x %u px (toi da %d px).\n",
                static_cast<unsigned>(width), static_cast<unsigned>(height), OLED_WIDTH);
  if (width > OLED_WIDTH || height > OLED_HEIGHT) {
    Serial.println("Loi: font khong vua man hinh.");
    return;
  }

  startupStage = "WiFi.mode WIFI_STA";
  if (!WiFi.mode(WIFI_STA)) {
    Serial.println("Loi khoi tao WIFI_STA.");
    return;
  }
  WiFi.setAutoReconnect(false);
  WiFi.disconnect();
  startupStage = "dat kenh Wi-Fi";
  esp_err_t err = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) {
    Serial.printf("Loi dat kenh Wi-Fi: %s\n", esp_err_to_name(err));
    return;
  }

  // In MAC STA thật để cấu hình unicast nếu cần.
  uint8_t mac[6];
  if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
    Serial.printf("MAC STA node NHAN: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
  startupStage = "esp_now_init";
  err = esp_now_init();
  if (err != ESP_OK) {
    Serial.printf("Loi esp_now_init: %s\n", esp_err_to_name(err));
    return;
  }
  startupStage = "dang ky callback ESP-NOW";
  err = esp_now_register_recv_cb(onDataReceived);
  if (err != ESP_OK) {
    Serial.printf("Loi dang ky callback nhan: %s\n", esp_err_to_name(err));
    return;
  }
  ready = true;
  Serial.printf("Node NHAN san sang, kenh %d.\n", ESPNOW_CHANNEL);

  // Hiện ngay để kiểm tra OLED, kể cả khi node gửi chưa bật.
  const char waitingText[] = "Cho du lieu ESP-NOW";
  display.getTextBounds(waitingText, 0, 0, &x1, &y1, &width, &height);
  display.clearDisplay();
  display.setCursor(-x1, (OLED_HEIGHT - static_cast<int16_t>(height)) / 2 - y1);
  display.print(waitingText);
  display.display();
}

void loop() {
  const uint32_t now = millis();
  // Log lặp để mở Serial muộn vẫn thấy trạng thái, không dùng delay().
  if (static_cast<uint32_t>(now - lastDebugMs) >= DEBUG_INTERVAL_MS) {
    lastDebugMs = now;
    if (!ready) {
      Serial.printf("[LOI] Khoi tao dung tai: %s. Nhan EN de xem log day du.\n",
                    startupStage);
    } else if (!hasReceivedPacket ||
               static_cast<uint32_t>(now - lastPacketMs) >= DEBUG_INTERVAL_MS) {
      Serial.printf("[CHO] Node nhan dang chay, cho goi ESP-NOW tren kenh %d.\n",
                    ESPNOW_CHANNEL);
    }
  }
  if (!ready) {
    return;
  }

  char message[MESSAGE_BUFFER_SIZE];
  bool hasMessage = false;
  // Lấy bản sao riêng để callback không sửa dữ liệu đang vẽ.
  portENTER_CRITICAL(&receiveMux);
  if (newMessage) {
    memcpy(message, receivedMessage, sizeof(message));
    newMessage = false;
    hasMessage = true;
  }
  portEXIT_CRITICAL(&receiveMux);
  if (!hasMessage) {
    return;
  }
  lastPacketMs = now;
  hasReceivedPacket = true;

  int16_t x1, y1;
  uint16_t width, height;
  display.getTextBounds(message, 0, 0, &x1, &y1, &width, &height);
  if (width > OLED_WIDTH || height > OLED_HEIGHT) {
    Serial.println("Loi: chuoi nhan khong vua mot dong.");
    return;
  }
  // GFX dùng baseline: bù y1 để giữa khung chữ nằm giữa màn hình.
  const int16_t baseline = (OLED_HEIGHT - static_cast<int16_t>(height)) / 2 - y1;
  display.clearDisplay();  // Xóa sạch số cũ khi 100 -> 99 -> ... -> 9.
  display.setCursor(-x1, baseline);  // Mép trái phần chữ thực sự ở x = 0.
  display.print(message);
  display.display();
  Serial.printf("NHAN: %s\n", message);
}
