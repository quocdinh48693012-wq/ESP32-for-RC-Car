#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

// ================= CAU HINH ==================
const uint8_t WIFI_CHANNEL = 1;

const uint32_t PACKET_MAGIC = 0x52435431;
const uint8_t PACKET_VERSION = 1;

const uint32_t PRINT_MS = 100;
const uint32_t TIMEOUT_MS = 200;

// PHAI GIONG HET BEN TX.
struct ControlPacket {
  uint32_t magic;
  uint32_t packetId;

  uint8_t version;
  uint8_t ready;
  uint8_t steerState;
  uint8_t throttleState;

  uint16_t steerRaw;
  uint16_t throttleRaw;
  uint16_t trimRaw;

  uint16_t steerFiltered;
  uint16_t throttleFiltered;
  uint16_t trimFiltered;

  uint16_t steerCenter;
  uint16_t throttleCenter;

  uint16_t steerPulseUs;
  uint16_t throttlePulseUs;
};

static_assert(sizeof(ControlPacket) == 32,
              "Sai kich thuoc goi tin");

// ================= DU LIEU DUNG CHUNG =========
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

ControlPacket latestPacket = {};
bool hasReceived = false;
uint32_t lastRecvTime = 0;
uint32_t receivedCount = 0;

bool radioReady = false;
uint32_t lastPrintTime = 0;

// ================= CALLBACK ==================
// Chu ky callback cho Arduino ESP32 Core 3.x.
void onDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data,
                int len) {
  (void)info;

  if (data == nullptr ||
      len != static_cast<int>(sizeof(ControlPacket))) {
    return;
  }

  ControlPacket packet;
  memcpy(&packet, data, sizeof(packet));

  // Bo qua goi khong dung dinh dang cua chuong trinh.
  if (packet.magic != PACKET_MAGIC ||
      packet.version != PACKET_VERSION ||
      packet.ready > 1 ||
      packet.steerState > 3 ||
      packet.throttleState > 3) {
    return;
  }

  const uint32_t receiveTime = millis();

  portENTER_CRITICAL(&rxMux);

  latestPacket = packet;
  lastRecvTime = receiveTime;
  hasReceived = true;
  receivedCount++;

  portEXIT_CRITICAL(&rxMux);

  // Khong in Serial trong callback.
}

// ================= SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (esp_wifi_set_channel(
          WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("LOI: Khong dat duoc kenh Wi-Fi.");
    return;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("LOI: Khong khoi tao duoc ESP-NOW.");
    return;
  }

  if (esp_now_register_recv_cb(onDataRecv) != ESP_OK) {
    Serial.println("LOI: Khong dang ky duoc callback.");
    return;
  }

  radioReady = true;

  Serial.println();
  Serial.println("===== RX CHI NHAN VA HIEN THI =====");
  Serial.print("MAC RX: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Dang cho TX...");
}

// ================= LOOP ======================
void loop() {
  if (!radioReady) {
    delay(100);
    return;
  }

  if (static_cast<uint32_t>(millis() - lastPrintTime)
      < PRINT_MS) {
    delay(1);
    return;
  }

  // Chup du lieu nhat quan, tranh callback ghi giua luc doc.
  ControlPacket packet;
  bool received;
  uint32_t receiveTime;
  uint32_t count;

  portENTER_CRITICAL(&rxMux);

  packet = latestPacket;
  received = hasReceived;
  receiveTime = lastRecvTime;
  count = receivedCount;

  portEXIT_CRITICAL(&rxMux);

  // Lay now sau snapshot de tranh timestamp nhan moi hon now.
  const uint32_t now = millis();
  lastPrintTime = now;

  if (!received) {
    Serial.println("[RX] Dang cho TX...");
    return;
  }

  const uint32_t age = now - receiveTime;

  if (age > TIMEOUT_MS) {
    Serial.printf(
        "[MAT TIN HIEU] Khong co goi moi trong %lu ms\n",
        static_cast<unsigned long>(age));
    return;
  }

  if (!packet.ready) {
    Serial.printf(
        "[CHO CALIB] LAI ADC:%u Buoc:%u | "
        "GA ADC:%u Buoc:%u | TRIM:%u | "
        "High LAI:%u us GA:%u us\n",

        static_cast<unsigned>(packet.steerRaw),
        static_cast<unsigned>(packet.steerState + 1),

        static_cast<unsigned>(packet.throttleRaw),
        static_cast<unsigned>(packet.throttleState + 1),

        static_cast<unsigned>(packet.trimFiltered),
        static_cast<unsigned>(packet.steerPulseUs),
        static_cast<unsigned>(packet.throttlePulseUs));
    return;
  }

  Serial.printf(
      "[#%lu Nhan:%lu Tuoi:%lums] "
      "LAI Raw:%u Loc:%u Tam:%u High:%uus | "
      "GA Raw:%u Loc:%u Tam:%u High:%uus | "
      "TRIM Raw:%u Loc:%u\n",

      static_cast<unsigned long>(packet.packetId),
      static_cast<unsigned long>(count),
      static_cast<unsigned long>(age),

      static_cast<unsigned>(packet.steerRaw),
      static_cast<unsigned>(packet.steerFiltered),
      static_cast<unsigned>(packet.steerCenter),
      static_cast<unsigned>(packet.steerPulseUs),

      static_cast<unsigned>(packet.throttleRaw),
      static_cast<unsigned>(packet.throttleFiltered),
      static_cast<unsigned>(packet.throttleCenter),
      static_cast<unsigned>(packet.throttlePulseUs),

      static_cast<unsigned>(packet.trimRaw),
      static_cast<unsigned>(packet.trimFiltered));
}