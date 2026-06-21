#ifndef RELIABLE_MAGNETIC_SENSOR_I2C_H
#define RELIABLE_MAGNETIC_SENSOR_I2C_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

// Project-owned AS5600 reader based on SimpleFOC 2.3.5 MagneticSensorI2C.
// Failed I2C reads return a negative angle so Sensor::update() ignores them
// instead of treating a communication failure as a real zero-degree sample.
class ReliableMagneticSensorI2C : public Sensor {
 public:
  // Configure the reader for the AS5600's fixed address and 12-bit angle register.
  ReliableMagneticSensorI2C() = default;

  // Initialize the selected I2C interface and SimpleFOC sensor history.
  void init(TwoWire* wire = &Wire) {
    wire_ = wire; // I2C controller used for every AS5600 transaction.
    wire_->begin();
    Sensor::init();
  }

  // Read the current mechanical angle in radians, or a negative value on error.
  float getSensorAngle() override {
    const int raw_count = getRawCount(); // Current 12-bit angle, or -1 when I2C failed.
    if (raw_count < 0) {
      return -1.0f;
    }
    return (static_cast<float>(raw_count) / kCountsPerRotation) * _2PI;
  }

  uint8_t currWireError = 0; // Latest Wire result: zero is success, nonzero is failure.

 private:
  static constexpr uint8_t kChipAddress = 0x36; // Fixed AS5600 I2C address.
  static constexpr uint8_t kAngleRegister = 0x0C; // AS5600 RAW ANGLE high-byte register.
  static constexpr float kCountsPerRotation = 4096.0f; // AS5600 counts per mechanical turn.

  // Read the AS5600 raw angle count, returning -1 rather than a valid zero on failure.
  int getRawCount() {
    uint8_t read_bytes[2] = {0, 0}; // High and low angle bytes returned by the AS5600.

    wire_->beginTransmission(kChipAddress);
    wire_->write(kAngleRegister);
    currWireError = wire_->endTransmission(true);
    if (currWireError != 0) {
      return -1;
    }

    const size_t received_bytes = wire_->requestFrom(kChipAddress, static_cast<size_t>(2)); // Actual response length.
    if (received_bytes != 2) {
      currWireError = 4; // Match the generic SimpleFOC "other I2C error" value.
      return -1;
    }

    for (uint8_t index = 0; index < 2; index++) {
      read_bytes[index] = wire_->read();
    }

    const uint16_t raw_count =
      (static_cast<uint16_t>(read_bytes[0] & 0x0F) << 8) |
      static_cast<uint16_t>(read_bytes[1]); // Combined 12-bit AS5600 raw angle.
    return static_cast<int>(raw_count);
  }

  TwoWire* wire_ = &Wire; // Active I2C interface, defaulting to the primary ESP32 bus.
};

#endif
