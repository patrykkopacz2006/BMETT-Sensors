#include <Wire.h>
#include "TLx493D_inc.hpp"
#include <Adafruit_BNO08x.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>
#include <string.h>

using namespace ifx::tlx493d;

TwoWire I2C_MAG1 = TwoWire(0);
TwoWire I2C_MAG2 = TwoWire(1);
TwoWire I2C_IMU  = TwoWire(2);

TLx493D_A1B6 *mag1 = nullptr;
TLx493D_A1B6 *mag2 = nullptr;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

const int MUSCLE1_PIN = 4;
const int MUSCLE2_PIN = 5;

BLECharacteristic *streamChar = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID "91B10000-4E5A-4B1E-8F00-91B191B191B1"
#define STREAM_UUID  "91B10001-4E5A-4B1E-8F00-91B191B191B1"

#define PKT_MAG    1
#define PKT_IMU    2
#define PKT_MUSCLE 3

#define MAG_SAMPLE_US      5000UL
#define IMU_SAMPLE_US     10000UL
#define MUSCLE_SAMPLE_US   1000UL

#define MAG_SAMPLES_PER_PKT      4
#define IMU_SAMPLES_PER_PKT      2
#define MUSCLE_SAMPLES_PER_PKT  20

struct __attribute__((packed)) PacketHeader {
  uint8_t  type;
  uint8_t  count;
  uint16_t reserved;
  uint32_t ts0_us;
  uint32_t dt_us;
};

struct __attribute__((packed)) MagSample {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct __attribute__((packed)) ImuSample {
  int16_t yaw_cdeg;
  int16_t pitch_cdeg;
  int16_t roll_cdeg;
};

struct __attribute__((packed)) MuscleSample {
  uint16_t m1;
  uint16_t m2;
};

struct __attribute__((packed)) MagPacket {
  PacketHeader h;
  MagSample s[MAG_SAMPLES_PER_PKT];
};

struct __attribute__((packed)) ImuPacket {
  PacketHeader h;
  ImuSample s[IMU_SAMPLES_PER_PKT];
};

struct __attribute__((packed)) MusclePacket {
  PacketHeader h;
  MuscleSample s[MUSCLE_SAMPLES_PER_PKT];
};

MagPacket magPkt;
ImuPacket imuPkt;
MusclePacket musclePkt;

uint8_t magCount = 0;
uint8_t imuCount = 0;
uint8_t muscleCount = 0;

uint32_t lastMagDue = 0;
uint32_t lastImuDue = 0;
uint32_t lastMuscleDue = 0;

float mag1Xf = 0.0f, mag1Yf = 0.0f, mag1Zf = 0.0f;
float mag2Xf = 0.0f, mag2Yf = 0.0f, mag2Zf = 0.0f;
bool mag1Init = false, mag2Init = false;

float yawDeg = 0.0f, pitchDeg = 0.0f, rollDeg = 0.0f;

float muscle1f = 0.0f, muscle2f = 0.0f;
bool muscleInit = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer*) override {
    deviceConnected = false;
  }
};

static inline float lowpass(float x, float prev, float a) {
  return prev + a * (x - prev);
}

static inline int clamp16(int v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return v;
}

static inline uint16_t clampU16(int v) {
  if (v < 0) return 0;
  if (v > 65535) return 65535;
  return (uint16_t)v;
}

static inline int limitStep(int input, int prev, int maxStep) {
  int d = input - prev;
  if (d > maxStep) return prev + maxStep;
  if (d < -maxStep) return prev - maxStep;
  return input;
}

void quatToEuler(float qr, float qi, float qj, float qk, float &yaw, float &pitch, float &roll) {
  float sqr = qr * qr;
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;
  float denom = sqi + sqj + sqk + sqr;
  if (denom <= 0.0f) return;

  float p = -2.0f * (qi * qk - qj * qr) / denom;
  if (p > 1.0f) p = 1.0f;
  if (p < -1.0f) p = -1.0f;

  yaw   = atan2f(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * 180.0f / PI;
  pitch = asinf(p) * 180.0f / PI;
  roll  = atan2f(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) * 180.0f / PI;
}

void setupBLE() {
  BLEDevice::init("ESP32_RESEARCH_STREAM");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);

  streamChar = service->createCharacteristic(
    STREAM_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  streamChar->addDescriptor(new BLE2902());

  service->start();
  server->getAdvertising()->start();
}

void maybeRestartAdvertising() {
  if (!deviceConnected && oldDeviceConnected) {
    delay(100);
    BLEDevice::getAdvertising()->start();
  }
  oldDeviceConnected = deviceConnected;
}

void sendPacket(const uint8_t *data, size_t len) {
  if (deviceConnected && streamChar != nullptr) {
    streamChar->setValue(data, len);
    streamChar->notify();
  }
}

void sampleMag1() {
  if (mag1 == nullptr) return;
  double t = 0.0, x = 0.0, y = 0.0, z = 0.0;
  if (mag1->getMagneticFieldAndTemperature(&x, &y, &z, &t)) {
    if (!mag1Init) {
      mag1Xf = x; mag1Yf = y; mag1Zf = z;
      mag1Init = true;
    } else {
      mag1Xf = lowpass((float)x, mag1Xf, 0.12f);
      mag1Yf = lowpass((float)y, mag1Yf, 0.12f);
      mag1Zf = lowpass((float)z, mag1Zf, 0.12f);
    }
  }
}

void sampleMag2() {
  if (mag2 == nullptr) return;
  double t = 0.0, x = 0.0, y = 0.0, z = 0.0;
  if (mag2->getMagneticFieldAndTemperature(&x, &y, &z, &t)) {
    if (!mag2Init) {
      mag2Xf = x; mag2Yf = y; mag2Zf = z;
      mag2Init = true;
    } else {
      mag2Xf = lowpass((float)x, mag2Xf, 0.12f);
      mag2Yf = lowpass((float)y, mag2Yf, 0.12f);
      mag2Zf = lowpass((float)z, mag2Zf, 0.12f);
    }
  }
}

void maybeReadIMU() {
  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, IMU_SAMPLE_US);
  }

  while (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
      quatToEuler(
        sensorValue.un.arvrStabilizedRV.real,
        sensorValue.un.arvrStabilizedRV.i,
        sensorValue.un.arvrStabilizedRV.j,
        sensorValue.un.arvrStabilizedRV.k,
        yawDeg, pitchDeg, rollDeg
      );
    }
  }
}

void addMagSample(uint32_t ts) {
  if (magCount == 0) {
    magPkt.h.type = PKT_MAG;
    magPkt.h.count = 0;
    magPkt.h.reserved = 0;
    magPkt.h.ts0_us = ts;
    magPkt.h.dt_us = MAG_SAMPLE_US;
  }

  sampleMag1();
  sampleMag2();

  float mx = mag1Xf;
  float my = mag1Yf;
  float mz = mag1Zf;

  magPkt.s[magCount].x = (int16_t)clamp16((int)lroundf(mx * 1000.0f));
  magPkt.s[magCount].y = (int16_t)clamp16((int)lroundf(my * 1000.0f));
  magPkt.s[magCount].z = (int16_t)clamp16((int)lroundf(mz * 1000.0f));

  magCount++;
  magPkt.h.count = magCount;

  if (magCount >= MAG_SAMPLES_PER_PKT) {
    sendPacket((uint8_t*)&magPkt, sizeof(PacketHeader) + magCount * sizeof(MagSample));
    magCount = 0;
  }
}

void addImuSample(uint32_t ts) {
  maybeReadIMU();

  if (imuCount == 0) {
    imuPkt.h.type = PKT_IMU;
    imuPkt.h.count = 0;
    imuPkt.h.reserved = 0;
    imuPkt.h.ts0_us = ts;
    imuPkt.h.dt_us = IMU_SAMPLE_US;
  }

  imuPkt.s[imuCount].yaw_cdeg   = (int16_t)clamp16((int)lroundf(yawDeg * 100.0f));
  imuPkt.s[imuCount].pitch_cdeg = (int16_t)clamp16((int)lroundf(pitchDeg * 100.0f));
  imuPkt.s[imuCount].roll_cdeg  = (int16_t)clamp16((int)lroundf(rollDeg * 100.0f));

  imuCount++;
  imuPkt.h.count = imuCount;

  if (imuCount >= IMU_SAMPLES_PER_PKT) {
    sendPacket((uint8_t*)&imuPkt, sizeof(PacketHeader) + imuCount * sizeof(ImuSample));
    imuCount = 0;
  }
}

void addMuscleSample(uint32_t ts) {
  int raw1 = analogRead(MUSCLE1_PIN);
  int raw2 = analogRead(MUSCLE2_PIN);

  if (!muscleInit) {
    muscle1f = raw1;
    muscle2f = raw2;
    muscleInit = true;
  } else {
    int lim1 = limitStep(raw1, (int)muscle1f, 250);
    int lim2 = limitStep(raw2, (int)muscle2f, 250);
    muscle1f = lowpass((float)lim1, muscle1f, 0.08f);
    muscle2f = lowpass((float)lim2, muscle2f, 0.08f);
  }

  if (muscleCount == 0) {
    musclePkt.h.type = PKT_MUSCLE;
    musclePkt.h.count = 0;
    musclePkt.h.reserved = 0;
    musclePkt.h.ts0_us = ts;
    musclePkt.h.dt_us = MUSCLE_SAMPLE_US;
  }

  musclePkt.s[muscleCount].m1 = clampU16((int)lroundf(muscle1f));
  musclePkt.s[muscleCount].m2 = clampU16((int)lroundf(muscle2f));

  muscleCount++;
  musclePkt.h.count = muscleCount;

  if (muscleCount >= MUSCLE_SAMPLES_PER_PKT) {
    sendPacket((uint8_t*)&musclePkt, sizeof(PacketHeader) + muscleCount * sizeof(MuscleSample));
    muscleCount = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  analogReadResolution(12);

  I2C_MAG1.begin(8, 9);
  delay(30);
  I2C_MAG2.begin(17, 18);
  delay(30);
  I2C_IMU.begin(11, 12);
  delay(30);

  mag1 = new TLx493D_A1B6(I2C_MAG1, TLx493D_IIC_ADDR_A0_e);
  delay(20);
  mag2 = new TLx493D_A1B6(I2C_MAG2, TLx493D_IIC_ADDR_A0_e);
  delay(20);

  if (mag1) {
    mag1->begin();
    delay(30);
  }

  if (mag2) {
    mag2->begin();
    delay(30);
  }

  bno08x.begin_I2C(0x4B, &I2C_IMU);
  bno08x.enableReport(SH2_ARVR_STABILIZED_RV, IMU_SAMPLE_US);

  setupBLE();

  uint32_t now = micros();
  lastMagDue = now;
  lastImuDue = now;
  lastMuscleDue = now;
}

void loop() {
  uint32_t now = micros();

  while ((int32_t)(now - lastMagDue) >= 0) {
    addMagSample(lastMagDue);
    lastMagDue += MAG_SAMPLE_US;
    now = micros();
  }

  while ((int32_t)(now - lastImuDue) >= 0) {
    addImuSample(lastImuDue);
    lastImuDue += IMU_SAMPLE_US;
    now = micros();
  }

  while ((int32_t)(now - lastMuscleDue) >= 0) {
    addMuscleSample(lastMuscleDue);
    lastMuscleDue += MUSCLE_SAMPLE_US;
    now = micros();
  }

  maybeRestartAdvertising();
}
