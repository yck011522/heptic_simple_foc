#include <SimpleFOC.h>
#include <Wire.h>

namespace {
constexpr int kI2cSdaPin = 19;
constexpr int kI2cSclPin = 18;
constexpr uint32_t kI2cClockHz = 400000;
constexpr uint32_t kSamplePeriodMs = 1;
}

MagneticSensorI2C as5600(AS5600_I2C);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(kI2cSdaPin, kI2cSclPin, kI2cClockHz);
  Wire.setClock(kI2cClockHz);
  Wire.setTimeOut(2);

  as5600.init(&Wire);

  Serial.println();
  Serial.println("SimpleFOC AS5600 test");
  Serial.print("SDA=");
  Serial.print(kI2cSdaPin);
  Serial.print(" SCL=");
  Serial.println(kI2cSclPin);
  Serial.println("Sensor init complete");
}

void loop() {
  as5600.update();

  Serial.print("angle=");
  Serial.print(as5600.getAngle(), 5);
  Serial.print(" velocity=");
  Serial.println(as5600.getVelocity(), 5);

  delay(kSamplePeriodMs);
}
