#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <string.h>

// ================= PHAN CUNG =================
const uint8_t PIN_STEER = 35;
const uint8_t PIN_THROTTLE = 34;
const uint8_t PIN_TRIM = 32;

const uint8_t LED_PIN = 2;
const uint8_t LED_ON_LEVEL = HIGH;

// ================= ESP-NOW ===================
const uint8_t WIFI_CHANNEL = 1;

const uint8_t BROADCAST_MAC[6] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

const uint32_t PACKET_MAGIC = 0x52435431;
const uint8_t PACKET_VERSION = 1;

// PHAI GIONG HET BEN RX.
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

// ================= THOI GIAN =================
const uint32_t SAMPLE_MS = 10;
const uint32_t SEND_MS = 20;
const uint32_t PRINT_MS = 100;

const float ALPHA = 0.2f;
const int CALIB_SAMPLES = 200;

// ================= HIEU CHUAN ================
const int STEER = 0;
const int THROTTLE = 1;
const int CHANNEL_COUNT = 2;

const uint8_t ADC_PINS[CHANNEL_COUNT] = {
  PIN_STEER, PIN_THROTTLE
};

const char *CHANNEL_NAMES[CHANNEL_COUNT] = {
  "LAI", "GA"
};

// Tam so bo ban da do, dung de nhan biet tha co/lai.
const int EXPECTED_CENTER[CHANNEL_COUNT] = {
  1825, 2070
};

const int MOVE_THRESHOLD[CHANNEL_COUNT] = {
  250, 250
};

const int RETURN_WINDOW[CHANNEL_COUNT] = {
  120, 100
};

const int MAX_SPREAD[CHANNEL_COUNT] = {
  100, 60
};

const int DEADBAND[CHANNEL_COUNT] = {
  50, 20
};

// ================= GIOI HAN ADC / XUNG ========
const int STEER_RIGHT_ADC = 0;
const int STEER_LEFT_ADC = 4095;
const int THROTTLE_FULL_ADC = 0;

const float NEUTRAL_US = 1500.0f;
const float STEER_RIGHT_US = 1000.0f;
const float STEER_LEFT_US = 2000.0f;
const float THROTTLE_FULL_US = 2000.0f;

// ================= TRANG THAI =================
enum CalibState {
  WAIT_MOVE = 0,
  WAIT_RETURN = 1,
  MEASURE_CENTER = 2,
  READY = 3
};

CalibState state[CHANNEL_COUNT] = {
  WAIT_MOVE, WAIT_MOVE
};

int rawADC[CHANNEL_COUNT] = {0, 0};
int centerADC[CHANNEL_COUNT] = {1825, 2070};
float filteredADC[CHANNEL_COUNT] = {1825, 2070};

uint32_t sampleSum[CHANNEL_COUNT] = {0, 0};
int sampleCount[CHANNEL_COUNT] = {0, 0};
int sampleMin[CHANNEL_COUNT] = {4095, 4095};
int sampleMax[CHANNEL_COUNT] = {0, 0};

int trimRaw = 0;
float trimFiltered = 0;

bool allReady = false;
bool radioReady = false;

uint32_t packetId = 0;
uint32_t sendApiErrors = 0;

uint32_t lastSampleTime = 0;
uint32_t lastSendTime = 0;
uint32_t lastPrintTime = 0;

uint16_t steerPulseUs = 1500;
uint16_t throttlePulseUs = 1500;

// ================= HAM HO TRO =================
void setLed(bool on) {
  digitalWrite(
      LED_PIN,
      on ? LED_ON_LEVEL : !LED_ON_LEVEL);
}

void resetSamples(int channel) {
  sampleSum[channel] = 0;
  sampleCount[channel] = 0;
  sampleMin[channel] = 4095;
  sampleMax[channel] = 0;
}

bool nearNeutral(int channel, int value) {
  return abs(value - EXPECTED_CENTER[channel])
         <= RETURN_WINDOW[channel];
}

float mapClamped(float value,
                 float inputMin, float inputMax,
                 float outputMin, float outputMax) {
  if (inputMax <= inputMin) return NEUTRAL_US;

  if (value < inputMin) value = inputMin;
  if (value > inputMax) value = inputMax;

  return outputMin
       + (value - inputMin)
       * (outputMax - outputMin)
       / (inputMax - inputMin);
}

// ================= HIEU CHUAN TUNG KENH ========
void updateCalibration(int channel) {
  const int value = rawADC[channel];

  switch (state[channel]) {
    case WAIT_MOVE:
      if (abs(value - EXPECTED_CENTER[channel])
          >= MOVE_THRESHOLD[channel]) {
        state[channel] = WAIT_RETURN;

        Serial.printf(
            "[%s] Da di chuyen. Hay tha ve trung tinh.\n",
            CHANNEL_NAMES[channel]);
      }
      break;

    case WAIT_RETURN:
      if (nearNeutral(channel, value)) {
        resetSamples(channel);
        state[channel] = MEASURE_CENTER;

        Serial.printf(
            "[%s] Dang do tam, giu tha khoang 2 giay.\n",
            CHANNEL_NAMES[channel]);
      }
      break;

    case MEASURE_CENTER:
      if (!nearNeutral(channel, value)) {
        resetSamples(channel);
        state[channel] = WAIT_RETURN;
        break;
      }

      if (value < sampleMin[channel]) {
        sampleMin[channel] = value;
      }

      if (value > sampleMax[channel]) {
        sampleMax[channel] = value;
      }

      if (sampleMax[channel] - sampleMin[channel]
          > MAX_SPREAD[channel]) {
        resetSamples(channel);

        Serial.printf(
            "[%s] Chua on dinh, do lai.\n",
            CHANNEL_NAMES[channel]);
        break;
      }

      sampleSum[channel] += value;
      sampleCount[channel]++;

      if (sampleCount[channel] >= CALIB_SAMPLES) {
        centerADC[channel] =
            (sampleSum[channel] + sampleCount[channel] / 2)
            / sampleCount[channel];

        filteredADC[channel] = centerADC[channel];
        state[channel] = READY;

        Serial.printf(
            "[%s] XONG: Tam=%d, Min=%d, Max=%d\n",
            CHANNEL_NAMES[channel],
            centerADC[channel],
            sampleMin[channel],
            sampleMax[channel]);
      }
      break;

    case READY:
      // Tam duoc giu co dinh sau hieu chuan.
      break;
  }
}

// ================= TINH HIGH PULSE =============
void calculatePulses() {
  // Chua hieu chuan du ca hai -> chi gui trung tinh.
  steerPulseUs = 1500;
  throttlePulseUs = 1500;

  if (!allReady) return;

  // ----- LAI -----
  const float steerLow =
      centerADC[STEER] - DEADBAND[STEER];

  const float steerHigh =
      centerADC[STEER] + DEADBAND[STEER];

  float steerPulse = NEUTRAL_US;

  if (filteredADC[STEER] < steerLow) {
    // ADC giam -> phai.
    steerPulse = mapClamped(
        filteredADC[STEER],
        STEER_RIGHT_ADC, steerLow,
        STEER_RIGHT_US, NEUTRAL_US);
  } else if (filteredADC[STEER] > steerHigh) {
    // ADC tang -> trai.
    steerPulse = mapClamped(
        filteredADC[STEER],
        steerHigh, STEER_LEFT_ADC,
        NEUTRAL_US, STEER_LEFT_US);
  }

  // ----- GA -----
  const float forwardStart =
      centerADC[THROTTLE] - DEADBAND[THROTTLE];

  float throttlePulse = NEUTRAL_US;

  if (filteredADC[THROTTLE] < forwardStart) {
    // ADC 0 -> tien toi da.
    throttlePulse = mapClamped(
        filteredADC[THROTTLE],
        THROTTLE_FULL_ADC, forwardStart,
        THROTTLE_FULL_US, NEUTRAL_US);
  }

  // Phia thang giu 1500 us.
  steerPulseUs = static_cast<uint16_t>(lroundf(steerPulse));
  throttlePulseUs =
      static_cast<uint16_t>(lroundf(throttlePulse));
}

// ================= KHOI TAO ESP-NOW =============
bool initRadio() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (esp_wifi_set_channel(
          WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("LOI: Khong dat duoc kenh Wi-Fi.");
    return false;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("LOI: Khong khoi tao duoc ESP-NOW.");
    return false;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);

  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("LOI: Khong them duoc broadcast peer.");
    return false;
  }

  return true;
}

// ================= GUI DU LIEU ==================
void sendPacket() {
  ControlPacket packet = {};

  packet.magic = PACKET_MAGIC;
  packet.version = PACKET_VERSION;
  packet.packetId = ++packetId;

  packet.ready = allReady ? 1 : 0;
  packet.steerState = static_cast<uint8_t>(state[STEER]);
  packet.throttleState = static_cast<uint8_t>(state[THROTTLE]);

  packet.steerRaw = rawADC[STEER];
  packet.throttleRaw = rawADC[THROTTLE];
  packet.trimRaw = trimRaw;

  packet.steerFiltered =
      static_cast<uint16_t>(lroundf(filteredADC[STEER]));

  packet.throttleFiltered =
      static_cast<uint16_t>(lroundf(filteredADC[THROTTLE]));

  packet.trimFiltered =
      static_cast<uint16_t>(lroundf(trimFiltered));

  packet.steerCenter = centerADC[STEER];
  packet.throttleCenter = centerADC[THROTTLE];

  packet.steerPulseUs = steerPulseUs;
  packet.throttlePulseUs = throttlePulseUs;

  esp_err_t result = esp_now_send(
      BROADCAST_MAC,
      reinterpret_cast<const uint8_t *>(&packet),
      sizeof(packet));

  // Chi dem loi khi goi API gui.
  // ESP_OK khong co nghia RX chac chan da nhan.
  if (result != ESP_OK) {
    sendApiErrors++;
  }
}

// ================= SETUP ========================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  setLed(true);

  Serial.begin(115200);

  pinMode(PIN_STEER, INPUT);
  pinMode(PIN_THROTTLE, INPUT);
  pinMode(PIN_TRIM, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_STEER, ADC_11db);
  analogSetPinAttenuation(PIN_THROTTLE, ADC_11db);
  analogSetPinAttenuation(PIN_TRIM, ADC_11db);

  delay(500);

  for (int i = 0; i < CHANNEL_COUNT; i++) {
    rawADC[i] = analogRead(ADC_PINS[i]);
    filteredADC[i] = rawADC[i];
  }

  trimRaw = analogRead(PIN_TRIM);
  trimFiltered = trimRaw;

  radioReady = initRadio();

  if (!radioReady) {
    Serial.println("TX dung do loi radio. LED giu sang.");
    return;
  }

  Serial.println();
  Serial.println("===== TX LAI + GA + TRIM =====");
  Serial.print("MAC TX: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Xoay lai roi tha ve giua.");
  Serial.println("Bop ga roi nha hoan toan.");
  Serial.println("Ca hai hieu chuan xong -> LED tat.");
  Serial.println("Trim chi doc/loc, chua cong vao lai.");

  lastSampleTime = millis();
  lastSendTime = millis();
  lastPrintTime = millis();
}

// ================= LOOP =========================
void loop() {
  if (!radioReady) {
    delay(100);
    return;
  }

  uint32_t now = millis();

  if (static_cast<uint32_t>(now - lastSampleTime)
      >= SAMPLE_MS) {
    lastSampleTime = now;

    for (int i = 0; i < CHANNEL_COUNT; i++) {
      rawADC[i] = analogRead(ADC_PINS[i]);

      filteredADC[i] +=
          ALPHA * (rawADC[i] - filteredADC[i]);

      updateCalibration(i);
    }

    trimRaw = analogRead(PIN_TRIM);
    trimFiltered += ALPHA * (trimRaw - trimFiltered);

    if (!allReady &&
        state[STEER] == READY &&
        state[THROTTLE] == READY) {
      allReady = true;
      setLed(false);

      Serial.println("HIEU CHUAN HOAN TAT - LED TAT.");
    }

    calculatePulses();
  }

  if (static_cast<uint32_t>(now - lastSendTime)
      >= SEND_MS) {
    lastSendTime = now;
    sendPacket();
  }

  if (static_cast<uint32_t>(now - lastPrintTime)
      >= PRINT_MS) {
    lastPrintTime = now;

    Serial.printf(
        "READY:%u | LAI ADC:%4d High:%u us | "
        "GA ADC:%4d High:%u us | TRIM:%4d | LoiAPI:%lu\n",

        static_cast<unsigned>(allReady),
        static_cast<int>(lroundf(filteredADC[STEER])),
        static_cast<unsigned>(steerPulseUs),

        static_cast<int>(lroundf(filteredADC[THROTTLE])),
        static_cast<unsigned>(throttlePulseUs),

        static_cast<int>(lroundf(trimFiltered)),
        static_cast<unsigned long>(sendApiErrors));
  }
}