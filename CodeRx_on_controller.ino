// ESP32 NodeMCU-32S, Arduino core 3.x (cũng hỗ trợ core 2.x).
#define SERIAL_BAUD 115200
#define SEND_INTERVAL_MS 1000UL
#define BATTERY_START_PERCENT 100
#define ESPNOW_CHANNEL 1
#define MESSAGE_BUFFER_SIZE 64
#define USE_UNICAST 0  // 0: broadcast; 1: gửi tới RECEIVER_MAC bên dưới.
// Khi dùng unicast, bỏ // và thay đủ 6 byte bằng MAC STA thật của node nhận.
// #define RECEIVER_MAC {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <esp_err.h>
#include <string.h>

static_assert(BATTERY_START_PERCENT >= 0 && BATTERY_START_PERCENT <= 100,
              "BATTERY_START_PERCENT phai trong 0..100");

#if USE_UNICAST
#ifndef RECEIVER_MAC
#error "Hay dinh nghia RECEIVER_MAC bang MAC STA that cua node nhan."
#endif
const uint8_t destinationMac[6] = RECEIVER_MAC;
#else
const uint8_t destinationMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#endif

bool ready = false;
int batteryPercent = BATTERY_START_PERCENT;
uint32_t lastSendMs = 0;
portMUX_TYPE sendMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool sendResultPending = false;
volatile esp_now_send_status_t sendResult = ESP_NOW_SEND_FAIL;

// Core 3.x dùng IDF >= 5.5 đổi tham số callback gửi sang thông tin TX.
// Core 2.x và core 3.x dùng IDF < 5.5 vẫn truyền con trỏ MAC.
// Nhánh #if tự chọn; không cần đổi chữ ký bằng tay khi đổi core.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status);
void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  (void)info;
#else
void onDataSent(const uint8_t *mac, esp_now_send_status_t status);
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  (void)mac;
#endif
  portENTER_CRITICAL(&sendMux);
  sendResult = status;
  sendResultPending = true;
  portEXIT_CRITICAL(&sendMux);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  if (!WiFi.mode(WIFI_STA)) {
    Serial.println("Loi khoi tao WIFI_STA.");
    return;
  }
  WiFi.setAutoReconnect(false);
  WiFi.disconnect();  // Không kết nối router, giữ nguyên kênh ESP-NOW.

  esp_err_t err = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (err != ESP_OK) {
    Serial.printf("Loi dat kenh Wi-Fi: %s\n", esp_err_to_name(err));
    return;
  }
  err = esp_now_init();
  if (err != ESP_OK) {
    Serial.printf("Loi esp_now_init: %s\n", esp_err_to_name(err));
    return;
  }
  err = esp_now_register_send_cb(onDataSent);
  if (err != ESP_OK) {
    Serial.printf("Loi dang ky callback gui: %s\n", esp_err_to_name(err));
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, destinationMac, sizeof(destinationMac));
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  err = esp_now_add_peer(&peer);  // Broadcast cũng phải thêm peer.
  if (err != ESP_OK) {
    Serial.printf("Loi them peer: %s\n", esp_err_to_name(err));
    return;
  }

  lastSendMs = millis();  // Gói 100% đầu tiên sau 1 giây.
  ready = true;
  Serial.printf("Node GUI san sang, kenh %d, %s.\n", ESPNOW_CHANNEL,
                USE_UNICAST ? "unicast" : "broadcast");
}

void loop() {
  if (!ready) {
    return;
  }

  bool hasResult;
  esp_now_send_status_t result;
  portENTER_CRITICAL(&sendMux);
  hasResult = sendResultPending;
  result = sendResult;
  sendResultPending = false;
  portEXIT_CRITICAL(&sendMux);
  if (hasResult) {
    Serial.printf("Ket qua TX: %s\n",
                  result == ESP_NOW_SEND_SUCCESS ? "THANH CONG" : "THAT BAI");
    // Broadcast thành công không xác nhận node nhận đã nhận/vẽ gói tin.
  }

  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - lastSendMs) < SEND_INTERVAL_MS) {
    return;
  }
  lastSendMs = now;  // Không gửi dồn nếu loop bị trễ; an toàn khi millis tràn.

  char message[MESSAGE_BUFFER_SIZE];
  const int length = snprintf(message, sizeof(message),
                              "Muc pin con lai la : %d%%", batteryPercent);
  if (length < 0 || static_cast<size_t>(length) >= sizeof(message)) {
    Serial.println("Loi: bo dem gui qua nho.");
    return;
  }

  // Mỗi chu kỳ gọi gửi đúng một lần, kèm byte kết thúc chuỗi '\0'.
  const esp_err_t err = esp_now_send(destinationMac,
      reinterpret_cast<const uint8_t *>(message), static_cast<size_t>(length) + 1);
  Serial.printf("GUI: %s | esp_now_send: %s\n", message, esp_err_to_name(err));

  // Giảm sau mỗi lần gọi gửi, kể cả khi radio báo lỗi; không tự gửi lại.
  batteryPercent = (batteryPercent == 0) ? BATTERY_START_PERCENT : batteryPercent - 1;
}
