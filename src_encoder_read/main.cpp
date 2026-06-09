#include <Arduino.h>
#include <Wire.h>

namespace {
constexpr uint8_t kAs5600Address = 0x36;
constexpr uint8_t kAs5600AngleRegister = 0x0C;
constexpr int kI2cSdaPin = 19;
constexpr int kI2cSclPin = 18;
constexpr uint32_t kI2cClockHz = 1000000;
constexpr uint32_t kReportIntervalMs = 100;
}

uint32_t successfulReads = 0;
uint32_t failedWrites = 0;
uint32_t failedReads = 0;
uint16_t lastRawAngle = 0;
unsigned long lastReportMs = 0;
unsigned long lastSampleCount = 0;

bool readRawAngle(uint16_t &rawAngle) {
  Wire.beginTransmission(kAs5600Address);
  Wire.write(kAs5600AngleRegister);

  const uint8_t txError = Wire.endTransmission(true);
  if (txError != 0) {
    ++failedWrites;
    return false;
  }

  if (Wire.requestFrom((int)kAs5600Address, 2, (int)true) != 2) {
    ++failedReads;
    return false;
  }

  const uint8_t msb = Wire.read();
  const uint8_t lsb = Wire.read();
  rawAngle = (uint16_t)(((msb & 0x0F) << 8) | lsb);
  return true;
}

void printReport() {
  const unsigned long now = millis();
  if (now - lastReportMs < kReportIntervalMs) {
    return;
  }

  const unsigned long samplesSinceLastReport = successfulReads - lastSampleCount;
  const unsigned long elapsedMs = now - lastReportMs;
  const float readsPerSecond = elapsedMs > 0 ? (samplesSinceLastReport * 1000.0f) / elapsedMs : 0.0f;
  const float angleRadians = (lastRawAngle / 4096.0f) * 2.0f * PI;

  Serial.print("raw=");
  Serial.print(lastRawAngle);
  Serial.print(" angle=");
  Serial.print(angleRadians, 5);
  Serial.print(" reads_per_s=");
  Serial.print(readsPerSecond, 1);
  Serial.print(" ok=");
  Serial.print(successfulReads);
  Serial.print(" write_err=");
  Serial.print(failedWrites);
  Serial.print(" read_err=");
  Serial.println(failedReads);

  lastSampleCount = successfulReads;
  lastReportMs = now;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(kI2cSdaPin, kI2cSclPin, kI2cClockHz);
  Wire.setClock(kI2cClockHz);
  Wire.setTimeOut(2);

  Serial.println();
  Serial.println("AS5600 fast reader");
  Serial.print("SDA=");
  Serial.print(kI2cSdaPin);
  Serial.print(" SCL=");
  Serial.print(kI2cSclPin);
  Serial.print(" I2C=");
  Serial.println(kI2cClockHz);

  lastReportMs = millis();
}

void loop() {
  uint16_t rawAngle = 0;
  if (readRawAngle(rawAngle)) {
    lastRawAngle = rawAngle;
    ++successfulReads;
  }

  printReport();
}