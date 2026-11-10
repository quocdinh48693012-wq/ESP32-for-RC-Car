#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Logic dem vong doc lap phan cung; don vi 1/4096 vong.
struct Counter {
  int64_t total = 0;
  uint16_t previous = 0;
  uint32_t stamp = 0;
  bool anchored = false;
  void reset() { total = 0; anchored = false; }
  void gap() { anchored = false; }
  // 0: lay moc; 1: da cong; -1: mau khong dang tin.
  int update(uint16_t raw, uint32_t now, int direction) {
    if (raw >= 4096) { gap(); return -1; }
    if (!anchored) {
      previous = raw; stamp = now; anchored = true; return 0;
    }
    uint32_t dt = now - stamp;
    int delta = int(raw) - int(previous);
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    int64_t limit = (int64_t(1500) * 4096 * dt * 3) / 120000000 + 8;
    if (!dt || dt > 8000 || delta == 2048 || delta == -2048 ||
        delta > limit || delta < -limit) {
      gap(); return -1;
    }
    total += int64_t(direction) * delta;
    previous = raw; stamp = now;
    return 1;
  }
};

// Chan doc tu anh: can doi chieu truc tiep truoc khi cap nguon.
constexpr int SDA_PIN = 21, SCL_PIN = 22, UNUSED_PGO_PIN = 19;
constexpr uint32_t BAUD = 115200, BUS_HZ = 400000, SAMPLE_MS = 5;
constexpr uint8_t ADDRESS = 0x36, STATUS_REG = 0x0B;
constexpr int DIRECTION = 1; // Doi thanh -1 de dao quy uoc chieu.
constexpr uint32_t RPM_WINDOW_US = 200000, RETRY_MS = 100;
constexpr float EMA_ALPHA = 0.35f, DISPLAY_DEADBAND_RPM = 1.0f;

struct Snapshot {
  int64_t total = 0;
  uint32_t reads = 0, accepted = 0, errors = 0, overruns = 0;
  uint16_t angle = 0;
  float rpm = 0;
  uint8_t status = 0;
  bool valid = false, uncertain = false;
};
Snapshot shared;
portMUX_TYPE guard = portMUX_INITIALIZER_UNLOCKED;
bool resetRequested = false;

bool readSensor(uint8_t &status, uint16_t &raw) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(STATUS_REG);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADDRESS, uint8_t(3), true) != 3) return false;
  status = Wire.read();
  uint8_t hi = Wire.read(), lo = Wire.read();
  raw = (uint16_t(hi & 0x0F) << 8) | lo;
  return true;
}

void readerTask(void *) {
  Counter counter;
  Snapshot state;
  uint32_t rpmStamp = 0;
  int64_t rpmTotal = 0;
  bool rpmReady = false;
  const TickType_t period = pdMS_TO_TICKS(SAMPLE_MS);
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    bool reset;
    portENTER_CRITICAL(&guard);
    reset = resetRequested;
    resetRequested = false;
    portEXIT_CRITICAL(&guard);
    if (reset) {
      counter.reset(); rpmReady = false; state.rpm = 0;
      state.uncertain = false;
    }
    uint8_t status = 0;
    uint16_t raw = 0;
    bool connected = readSensor(status, raw);
    uint32_t now = micros();
    state.status = status;
    if (connected) ++state.reads;
    // MD=1, ML=0, MH=0 moi chap nhan mau.
    bool magnetOK = connected && (status & 0x20) && !(status & 0x18);
    state.valid = false;
    if (!magnetOK) {
      counter.gap(); ++state.errors; state.uncertain = true;
      rpmReady = false; state.rpm = 0;
    } else {
      int result = counter.update(raw, now, DIRECTION);
      state.angle = raw;
      if (result < 0) {
        ++state.errors; state.uncertain = true;
        rpmReady = false; state.rpm = 0;
      } else {
        state.valid = true; ++state.accepted;
        if (!rpmReady || result == 0) {
          rpmTotal = counter.total; rpmStamp = now; rpmReady = true;
          state.rpm = 0;
        } else if (uint32_t(now - rpmStamp) >= RPM_WINDOW_US) {
          float measured = double(counter.total - rpmTotal) * 60000000.0 /
                           (4096.0 * uint32_t(now - rpmStamp));
          state.rpm += EMA_ALPHA * (measured - state.rpm);
          rpmTotal = counter.total; rpmStamp = now;
        }
      }
    }
    state.total = counter.total;
    portENTER_CRITICAL(&guard);
    shared = state;
    portEXIT_CRITICAL(&guard);
    if (!magnetOK) {
      vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
      wake = xTaskGetTickCount();
    } else {
      // Neu qua han: khong doc don nhieu mau de "bu".
      if (TickType_t(xTaskGetTickCount() - wake) >= period) {
        ++state.overruns; wake = xTaskGetTickCount();
      }
      vTaskDelayUntil(&wake, period);
    }
  }
}

void setup() {
  Serial.begin(BAUD);
  pinMode(UNUSED_PGO_PIN, INPUT); // Khong kich hoat chan lap trinh PGO.
  Serial.println("# SDA=21,SCL=22,ADDR=0x36,I2C=400000,TargetHz=200,DesignRPM=1500,MaxGapUs=8000");
  Serial.println("# RPM chua biet: gioi han tren KHONG phai toc do motor da xac minh");
  if (!Wire.begin(SDA_PIN, SCL_PIN, BUS_HZ)) {
    Serial.println("# Loi khoi tao I2C"); return;
  }
  Wire.setTimeOut(3);
  if (xTaskCreate(readerTask, "as5600", 4096, nullptr, 2, nullptr) != pdPASS)
    Serial.println("# Khong tao duoc task doc cam bien");
}

void loop() {
  if (Serial.available() && Serial.read() == 'r') {
    portENTER_CRITICAL(&guard);
    resetRequested = true;
    portEXIT_CRITICAL(&guard);
    Serial.println("# Da yeu cau dat lai moc o lan doc tiep theo");
  }
  static uint32_t printed = 0, report = 0, lastReads = 0, lastAccepted = 0;
  static uint32_t lastErrors = UINT32_MAX;
  uint32_t now = millis();
  if (uint32_t(now - printed) >= 100) {
    printed = now;
    Snapshot s;
    portENTER_CRITICAL(&guard);
    s = shared;
    portEXIT_CRITICAL(&guard);
    if (s.errors != lastErrors) {
      if (s.errors)
        Serial.printf("# Loi=%lu,STATUS=0x%02X; mat mau/nam cham/qua han; so vong co the lech\n",
                      (unsigned long)s.errors, s.status);
      lastErrors = s.errors;
    }
    if (s.valid) {
      float rpm = fabsf(s.rpm) < DISPLAY_DEADBAND_RPM ? 0 : s.rpm;
      Serial.printf("Goc:%.2f,Vong:%.2f,VongTron:%lld,RPM:%.2f,Chieu:%d\n",
                    s.angle * 360.0 / 4096, double(s.total) / 4096,
                    (long long)(s.total / 4096), rpm, (rpm > 0) - (rpm < 0));
    } else Serial.println("# Khong co mau hop le; tam ngung dong so lieu");
    if (uint32_t(now - report) >= 5000) {
      double seconds = uint32_t(now - report) / 1000.0;
      Serial.printf("# DocHz=%.1f,HopLeHz=%.1f,QuaHan=%lu,SoVongKhongChac=%d\n",
                    (s.reads-lastReads)/seconds, (s.accepted-lastAccepted)/seconds,
                    (unsigned long)s.overruns, s.uncertain);
      report = now; lastReads = s.reads; lastAccepted = s.accepted;
    }
  }
  delay(1); // Chi nhuong CPU cua task hien thi, khong cham task cam bien.
}
