#include <SimpleFOC.h>
#include <Preferences.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {
constexpr int kI2cSdaPin = 19;
constexpr int kI2cSclPin = 18;
constexpr uint32_t kI2cClockHz = 400000;
constexpr int kPhaseAPin = 32;
constexpr int kPhaseBPin = 33;
constexpr int kPhaseCPin = 25;
constexpr int kEnablePin = 12;
constexpr int kMotorPolePairs = 14;
constexpr float kSupplyVoltage = 12.0f;
constexpr float kVoltageLimit = 10.0f; // Do not use 12V or the motor will feel clicky.
constexpr float kSensorAlignVoltage = 6.0f; // Seems strong enough
constexpr float kVelocityLimit = 40.0f;

constexpr uint32_t kSerialBaud = 230400;
constexpr uint16_t kDefaultTelemetryIntervalMs = 10;
constexpr uint32_t kFocRateWindowMs = 200;
constexpr uint32_t kFaultHoldMs = 500;
constexpr size_t kMaxProtocolLineLength = 127; // Excluding newline.
constexpr float kRadPerDecideg = PI / 1800.0f;
constexpr float kDecidegPerRad = 1800.0f / PI;
constexpr char kFirmwareVersion[] = "0.5.0";
}

enum StatusBit : uint8_t {
  kStatusTrackingEnabled = 0,
  kStatusBoundsEnabled = 1,
  kStatusOobKickEnabled = 2,
  kStatusDetentEnabled = 3,
  kStatusVibrationEnabled = 4,
  kStatusOutOfBounds = 5,
  kStatusFaultActive = 6,
};

struct RuntimeParams {
  float tracking_kp = 5.0f;
  float tracking_kd = 0.1f;
  float tracking_max_torque = 2.0f;

  float bounds_kp = 20.0f;
  float bounds_max_torque = 3.0f;

  float detent_kp = 5.0f;
  float detent_distance_rad = 10.0f * PI / 180.0f;
  float detent_max_torque = 1.0f;

  float vibration_amplitude = 1.0f;
  uint16_t vibration_pulse_interval_ms = 1000;

  float oob_kick_amplitude = 1.0f;
  uint16_t oob_kick_pulse_interval_ms = 40;

  bool enable_tracking = true;
  bool enable_bounds_restoration = true;
  bool enable_oob_kick = true;
  bool enable_detent = false;
  bool enable_vibration = false;

  uint16_t telemetry_interval_ms = kDefaultTelemetryIntervalMs;
};

struct ControlState {
  uint8_t dial_id = 0;
  uint32_t last_c_seq = 0;

  float target_angle_rad = 0.0f;
  float bound_min_rad = -5.0f;
  float bound_max_rad = 5.0f;

  float logical_angle_offset_rad = 0.0f;
  float filtered_velocity_rad_s = 0.0f;

  float last_torque_command = 0.0f;
  bool oob_pulse_on = false;
  unsigned long last_oob_pulse_ms = 0;

  bool out_of_bounds = false;
  long foc_control_rate_hz = 0;
  uint32_t status_bits = 0;
};

MagneticSensorI2C sensor(AS5600_I2C);
BLDCMotor motor(kMotorPolePairs);
BLDCDriver3PWM driver(kPhaseAPin, kPhaseBPin, kPhaseCPin, kEnablePin);
Preferences preferences;
Commander commander(Serial);

RuntimeParams g_params;
ControlState g_state;

unsigned long g_last_telemetry_ms = 0;
unsigned long g_fault_until_ms = 0;
unsigned long g_last_foc_window_ms = 0;
unsigned int g_foc_window_count = 0;
unsigned long g_last_control_micros = 0;

char g_line_buffer[kMaxProtocolLineLength + 1] = {0};
size_t g_line_len = 0;
bool g_line_overflow = false;

template <typename T>
T clampValue(T value, T min_value, T max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

float decidegToRad(long decideg) {
  return static_cast<float>(decideg) * kRadPerDecideg;
}

long radToDecideg(float rad) {
  return lroundf(rad * kDecidegPerRad);
}

long radPerSecToDecidegPerSec(float rad_per_sec) {
  return lroundf(rad_per_sec * kDecidegPerRad);
}

long torqueToMilliVolt(float torque_command) {
  const long raw_mv = lroundf(torque_command * 1000.0f);
  return clampValue<long>(raw_mv, -10000, 10000);
}

bool parseLong(const char* input, long& out) {
  if (input == nullptr || *input == '\0') {
    return false;
  }
  char* end_ptr = nullptr;
  const long parsed = strtol(input, &end_ptr, 10);
  if (end_ptr == input || *end_ptr != '\0') {
    return false;
  }
  out = parsed;
  return true;
}

bool parseUInt32(const char* input, uint32_t& out) {
  if (input == nullptr || *input == '\0') {
    return false;
  }
  char* end_ptr = nullptr;
  const unsigned long parsed = strtoul(input, &end_ptr, 10);
  if (end_ptr == input || *end_ptr != '\0') {
    return false;
  }
  out = static_cast<uint32_t>(parsed);
  return true;
}

void latchProtocolFault() {
  g_fault_until_ms = millis() + kFaultHoldMs;
}

void clearProtocolFault() {
  g_fault_until_ms = 0;
}

void persistDialId() {
  preferences.putUChar("dial_id", g_state.dial_id);
}

void persistParams() {
  preferences.putFloat("trk_kp", g_params.tracking_kp);
  preferences.putFloat("trk_kd", g_params.tracking_kd);
  preferences.putFloat("trk_max", g_params.tracking_max_torque);

  preferences.putFloat("bnd_kp", g_params.bounds_kp);
  preferences.putFloat("bnd_max", g_params.bounds_max_torque);

  preferences.putFloat("det_kp", g_params.detent_kp);
  preferences.putFloat("det_dst", g_params.detent_distance_rad);
  preferences.putFloat("det_max", g_params.detent_max_torque);

  preferences.putFloat("vib_amp", g_params.vibration_amplitude);
  preferences.putUShort("vib_int", g_params.vibration_pulse_interval_ms);

  preferences.putFloat("oob_amp", g_params.oob_kick_amplitude);
  preferences.putUShort("oob_int", g_params.oob_kick_pulse_interval_ms);

  preferences.putBool("en_trk", g_params.enable_tracking);
  preferences.putBool("en_bnd", g_params.enable_bounds_restoration);
  preferences.putBool("en_oob", g_params.enable_oob_kick);
  preferences.putBool("en_det", g_params.enable_detent);
  preferences.putBool("en_vib", g_params.enable_vibration);

  preferences.putUShort("tele_ms", g_params.telemetry_interval_ms);
}

void loadPersistentState() {
  g_state.dial_id = preferences.getUChar("dial_id", g_state.dial_id);

  g_params.tracking_kp = preferences.getFloat("trk_kp", g_params.tracking_kp);
  g_params.tracking_kd = preferences.getFloat("trk_kd", g_params.tracking_kd);
  g_params.tracking_max_torque = preferences.getFloat("trk_max", g_params.tracking_max_torque);

  g_params.bounds_kp = preferences.getFloat("bnd_kp", g_params.bounds_kp);
  g_params.bounds_max_torque = preferences.getFloat("bnd_max", g_params.bounds_max_torque);

  g_params.detent_kp = preferences.getFloat("det_kp", g_params.detent_kp);
  g_params.detent_distance_rad = preferences.getFloat("det_dst", g_params.detent_distance_rad);
  g_params.detent_max_torque = preferences.getFloat("det_max", g_params.detent_max_torque);

  g_params.vibration_amplitude = preferences.getFloat("vib_amp", g_params.vibration_amplitude);
  g_params.vibration_pulse_interval_ms = preferences.getUShort("vib_int", g_params.vibration_pulse_interval_ms);

  g_params.oob_kick_amplitude = preferences.getFloat("oob_amp", g_params.oob_kick_amplitude);
  g_params.oob_kick_pulse_interval_ms = preferences.getUShort("oob_int", g_params.oob_kick_pulse_interval_ms);

  g_params.enable_tracking = preferences.getBool("en_trk", g_params.enable_tracking);
  g_params.enable_bounds_restoration = preferences.getBool("en_bnd", g_params.enable_bounds_restoration);
  g_params.enable_oob_kick = preferences.getBool("en_oob", g_params.enable_oob_kick);
  g_params.enable_detent = preferences.getBool("en_det", g_params.enable_detent);
  g_params.enable_vibration = preferences.getBool("en_vib", g_params.enable_vibration);

  g_params.telemetry_interval_ms = preferences.getUShort("tele_ms", g_params.telemetry_interval_ms);
  g_params.telemetry_interval_ms = clampValue<uint16_t>(g_params.telemetry_interval_ms, 1, 200);
}

void onMotor(char* cmd) {
  commander.motor(&motor, cmd);
}

void updateStatusBits() {
  uint32_t bits = 0;
  if (g_params.enable_tracking) {
    bits |= (1UL << kStatusTrackingEnabled);
  }
  if (g_params.enable_bounds_restoration) {
    bits |= (1UL << kStatusBoundsEnabled);
  }
  if (g_params.enable_oob_kick) {
    bits |= (1UL << kStatusOobKickEnabled);
  }
  if (g_params.enable_detent) {
    bits |= (1UL << kStatusDetentEnabled);
  }
  if (g_params.enable_vibration) {
    bits |= (1UL << kStatusVibrationEnabled);
  }
  if (g_state.out_of_bounds) {
    bits |= (1UL << kStatusOutOfBounds);
  }
  if (millis() < g_fault_until_ms) {
    bits |= (1UL << kStatusFaultActive);
  }
  g_state.status_bits = bits;
}

void sendTelemetry(float logical_angle_rad) {
  updateStatusBits();
  const long angle_decideg = radToDecideg(logical_angle_rad);
  const long speed_decideg_per_s = radPerSecToDecidegPerSec(g_state.filtered_velocity_rad_s);
  const long torque_mv = torqueToMilliVolt(g_state.last_torque_command);

  Serial.print("T,");
  Serial.print(g_state.dial_id);
  Serial.print(",");
  Serial.print(g_state.last_c_seq);
  Serial.print(",");
  Serial.print(angle_decideg);
  Serial.print(",");
  Serial.print(speed_decideg_per_s);
  Serial.print(",");
  Serial.print(torque_mv);
  Serial.print(",");
  Serial.print(g_state.foc_control_rate_hz);
  Serial.print(",");
  Serial.println(g_state.status_bits);
}

bool handleSParam(const char* param_name, bool is_set, long set_value, long& response_value, bool& known_param) {
  known_param = true;

  if (strcmp(param_name, "tracking_kp") == 0) {
    if (is_set) g_params.tracking_kp = static_cast<float>(set_value) / 1000.0f;
    response_value = lroundf(g_params.tracking_kp * 1000.0f);
    return true;
  }
  if (strcmp(param_name, "tracking_kd") == 0) {
    if (is_set) g_params.tracking_kd = static_cast<float>(set_value) / 1000.0f;
    response_value = lroundf(g_params.tracking_kd * 1000.0f);
    return true;
  }
  if (strcmp(param_name, "tracking_max_torque") == 0) {
    if (is_set) g_params.tracking_max_torque = static_cast<float>(set_value) / 1000.0f;
    response_value = lroundf(g_params.tracking_max_torque * 1000.0f);
    return true;
  }
  if (strcmp(param_name, "bounds_kp") == 0) {
    if (is_set) g_params.bounds_kp = static_cast<float>(set_value) / 1000.0f;
    response_value = lroundf(g_params.bounds_kp * 1000.0f);
    return true;
  }
  if (strcmp(param_name, "bounds_max_torque") == 0) {
    if (is_set) g_params.bounds_max_torque = static_cast<float>(set_value) / 1000.0f;
    response_value = lroundf(g_params.bounds_max_torque * 1000.0f);
    return true;
  }
  if (strcmp(param_name, "detent_kp") == 0) {
    if (is_set) g_params.detent_kp = static_cast<float>(set_value) / 1000.0f;
    response_value = lroundf(g_params.detent_kp * 1000.0f);
    return true;
  }
  if (strcmp(param_name, "detent_distance") == 0) {
    if (is_set) {
      const float decideg = static_cast<float>(set_value) / 1000.0f;
      g_params.detent_distance_rad = decideg * kRadPerDecideg;
    }
    response_value = lroundf((radToDecideg(g_params.detent_distance_rad)) * 1000.0f);
    return true;
  }
  if (strcmp(param_name, "detent_max_torque") == 0) {
    if (is_set) g_params.detent_max_torque = static_cast<float>(set_value) / 1000.0f;
    response_value = lroundf(g_params.detent_max_torque * 1000.0f);
    return true;
  }
  if (strcmp(param_name, "vibration_amplitude") == 0) {
    if (is_set) g_params.vibration_amplitude = static_cast<float>(set_value) / 1000.0f;
    response_value = lroundf(g_params.vibration_amplitude * 1000.0f);
    return true;
  }
  if (strcmp(param_name, "vibration_pulse_interval_ms") == 0) {
    if (is_set) g_params.vibration_pulse_interval_ms = clampValue<uint16_t>(static_cast<uint16_t>(set_value), 1, 5000);
    response_value = g_params.vibration_pulse_interval_ms;
    return true;
  }
  if (strcmp(param_name, "oob_kick_amplitude") == 0) {
    if (is_set) g_params.oob_kick_amplitude = static_cast<float>(set_value) / 1000.0f;
    response_value = lroundf(g_params.oob_kick_amplitude * 1000.0f);
    return true;
  }
  if (strcmp(param_name, "oob_kick_pulse_interval_ms") == 0) {
    if (is_set) g_params.oob_kick_pulse_interval_ms = clampValue<uint16_t>(static_cast<uint16_t>(set_value), 1, 5000);
    response_value = g_params.oob_kick_pulse_interval_ms;
    return true;
  }
  if (strcmp(param_name, "enable_tracking") == 0) {
    if (is_set) g_params.enable_tracking = (set_value != 0);
    response_value = g_params.enable_tracking ? 1 : 0;
    return true;
  }
  if (strcmp(param_name, "enable_bounds_restoration") == 0) {
    if (is_set) g_params.enable_bounds_restoration = (set_value != 0);
    response_value = g_params.enable_bounds_restoration ? 1 : 0;
    return true;
  }
  if (strcmp(param_name, "enable_oob_kick") == 0) {
    if (is_set) g_params.enable_oob_kick = (set_value != 0);
    response_value = g_params.enable_oob_kick ? 1 : 0;
    return true;
  }
  if (strcmp(param_name, "enable_detent") == 0) {
    if (is_set) g_params.enable_detent = (set_value != 0);
    response_value = g_params.enable_detent ? 1 : 0;
    return true;
  }
  if (strcmp(param_name, "enable_vibration") == 0) {
    if (is_set) g_params.enable_vibration = (set_value != 0);
    response_value = g_params.enable_vibration ? 1 : 0;
    return true;
  }
  if (strcmp(param_name, "telemetry_interval") == 0) {
    if (is_set) g_params.telemetry_interval_ms = clampValue<uint16_t>(static_cast<uint16_t>(set_value), 1, 200);
    response_value = g_params.telemetry_interval_ms;
    return true;
  }

  known_param = false;
  return true;
}

void handleProtocolLine(char* line) {
  if (line[0] == '\0') {
    return;
  }

  if (line[0] == 'M' && strchr(line, ',') == nullptr) {
    commander.run(line);
    return;
  }

  char* save_ptr = nullptr;
  char* cmd_token = strtok_r(line, ",", &save_ptr);
  char* seq_token = strtok_r(nullptr, ",", &save_ptr);
  if (cmd_token == nullptr || seq_token == nullptr || cmd_token[1] != '\0') {
    latchProtocolFault();
    return;
  }

  uint32_t seq = 0;
  if (!parseUInt32(seq_token, seq)) {
    latchProtocolFault();
    return;
  }

  const char cmd = cmd_token[0];
  switch (cmd) {
    case 'C': {
      char* target_token = strtok_r(nullptr, ",", &save_ptr);
      char* min_token = strtok_r(nullptr, ",", &save_ptr);
      char* max_token = strtok_r(nullptr, ",", &save_ptr);
      if (target_token == nullptr || min_token == nullptr || max_token == nullptr || strtok_r(nullptr, ",", &save_ptr) != nullptr) {
        latchProtocolFault();
        return;
      }

      long target_decideg = 0;
      long min_decideg = 0;
      long max_decideg = 0;
      if (!parseLong(target_token, target_decideg) || !parseLong(min_token, min_decideg) || !parseLong(max_token, max_decideg)) {
        latchProtocolFault();
        return;
      }

      if (min_decideg > max_decideg) {
        latchProtocolFault();
        return;
      }

      g_state.target_angle_rad = decidegToRad(target_decideg);
      g_state.bound_min_rad = decidegToRad(min_decideg);
      g_state.bound_max_rad = decidegToRad(max_decideg);
      g_state.last_c_seq = seq;
      clearProtocolFault();
      return;
    }

    case 'R': {
      char* current_pos_token = strtok_r(nullptr, ",", &save_ptr);
      if (current_pos_token == nullptr || strtok_r(nullptr, ",", &save_ptr) != nullptr) {
        latchProtocolFault();
        return;
      }
      long current_pos_decideg = 0;
      if (!parseLong(current_pos_token, current_pos_decideg)) {
        latchProtocolFault();
        return;
      }

      const float requested_logical_rad = decidegToRad(current_pos_decideg);
      const float raw_angle = motor.shaftAngle();
      g_state.logical_angle_offset_rad = requested_logical_rad - raw_angle;
      g_state.filtered_velocity_rad_s = 0.0f;
      g_state.oob_pulse_on = false;
      g_state.last_oob_pulse_ms = millis();

      if (g_state.last_c_seq <= seq) {
        g_state.target_angle_rad = requested_logical_rad;
      }

      Serial.print("R,");
      Serial.println(seq);
      clearProtocolFault();
      return;
    }

    case 'S': {
      char* param_name = strtok_r(nullptr, ",", &save_ptr);
      if (param_name == nullptr) {
        latchProtocolFault();
        return;
      }
      char* value_token = strtok_r(nullptr, ",", &save_ptr);
      if (strtok_r(nullptr, ",", &save_ptr) != nullptr) {
        latchProtocolFault();
        return;
      }

      const bool is_set = (value_token != nullptr);
      long set_value = 0;
      if (is_set && !parseLong(value_token, set_value)) {
        latchProtocolFault();
        return;
      }

      long response_value = 0;
      bool known_param = false;
      handleSParam(param_name, is_set, set_value, response_value, known_param);

      if (known_param && is_set) {
        persistParams();
      }

      Serial.print("S,");
      Serial.print(seq);
      Serial.print(",");
      Serial.print(param_name);
      Serial.print(",");
      if (known_param) {
        Serial.println(response_value);
      } else {
        Serial.println("?");
      }
      clearProtocolFault();
      return;
    }

    case 'I': {
      char* id_token = strtok_r(nullptr, ",", &save_ptr);
      if (strtok_r(nullptr, ",", &save_ptr) != nullptr) {
        latchProtocolFault();
        return;
      }

      if (id_token != nullptr) {
        long dial_id_value = 0;
        if (!parseLong(id_token, dial_id_value) || dial_id_value < 0 || dial_id_value > 255) {
          latchProtocolFault();
          return;
        }
        g_state.dial_id = static_cast<uint8_t>(dial_id_value);
        persistDialId();
      }

      Serial.print("I,");
      Serial.print(seq);
      Serial.print(",");
      Serial.println(g_state.dial_id);
      clearProtocolFault();
      return;
    }

    case 'V': {
      if (strtok_r(nullptr, ",", &save_ptr) != nullptr) {
        latchProtocolFault();
        return;
      }
      Serial.print("V,");
      Serial.print(seq);
      Serial.print(",");
      Serial.println(kFirmwareVersion);
      clearProtocolFault();
      return;
    }

    case 'E': {
      if (strtok_r(nullptr, ",", &save_ptr) != nullptr) {
        latchProtocolFault();
        return;
      }
      Serial.print("E,");
      Serial.println(seq);
      clearProtocolFault();
      return;
    }

    default:
      latchProtocolFault();
      return;
  }
}

void serviceSerialInput() {
  while (Serial.available()) {
    const int read_value = Serial.read();
    if (read_value < 0) {
      return;
    }
    const char ch = static_cast<char>(read_value);

    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      if (g_line_overflow) {
        g_line_overflow = false;
        g_line_len = 0;
        g_line_buffer[0] = '\0';
        latchProtocolFault();
        continue;
      }
      g_line_buffer[g_line_len] = '\0';
      if (g_line_len > 0) {
        handleProtocolLine(g_line_buffer);
      }
      g_line_len = 0;
      g_line_buffer[0] = '\0';
      continue;
    }

    if (g_line_len >= kMaxProtocolLineLength) {
      g_line_overflow = true;
      continue;
    }
    g_line_buffer[g_line_len++] = ch;
  }
}

float computeCommandedTorque(float logical_angle_rad, float velocity_rad_s, unsigned long now_ms) {
  g_state.out_of_bounds = (logical_angle_rad < g_state.bound_min_rad) || (logical_angle_rad > g_state.bound_max_rad);

  float torque_sum = 0.0f;

  if (g_params.enable_tracking) {
    const float error = g_state.target_angle_rad - logical_angle_rad;
    const float tracking_torque = clampValue<float>(
      (g_params.tracking_kp * error) - (g_params.tracking_kd * velocity_rad_s),
      -g_params.tracking_max_torque,
      g_params.tracking_max_torque);
    torque_sum += tracking_torque;
  }

  if (g_params.enable_bounds_restoration && g_state.out_of_bounds) {
    float bound_error = 0.0f;
    if (logical_angle_rad < g_state.bound_min_rad) {
      bound_error = g_state.bound_min_rad - logical_angle_rad;
    } else if (logical_angle_rad > g_state.bound_max_rad) {
      bound_error = g_state.bound_max_rad - logical_angle_rad;
    }
    const float bounds_torque = clampValue<float>(
      g_params.bounds_kp * bound_error,
      -g_params.bounds_max_torque,
      g_params.bounds_max_torque);
    torque_sum += bounds_torque;
  }

  if (g_params.enable_oob_kick && g_state.out_of_bounds) {
    if ((now_ms - g_state.last_oob_pulse_ms) >= g_params.oob_kick_pulse_interval_ms) {
      g_state.last_oob_pulse_ms = now_ms;
      g_state.oob_pulse_on = !g_state.oob_pulse_on;
    }

    if (g_state.oob_pulse_on) {
      const float inward_direction = (logical_angle_rad < g_state.bound_min_rad) ? 1.0f : -1.0f;
      torque_sum += inward_direction * g_params.oob_kick_amplitude;
    }
  } else {
    g_state.oob_pulse_on = false;
  }

  return clampValue<float>(torque_sum, -kVoltageLimit, kVoltageLimit);
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(1000);

  preferences.begin("haptic", false);
  loadPersistentState();

  // This block is calibrated to reduce I2C bus error.
  Wire.begin(kI2cSdaPin, kI2cSclPin, kI2cClockHz);
  Wire.setClock(kI2cClockHz);
  Wire.setTimeOut(25);
  sensor.min_elapsed_time = 2000; // 2000 microseconds ~ 500Hz update rate, which is sufficient for this application and reduces I2C bus load
  sensor.init(&Wire);
  motor.linkSensor(&sensor);

  // This block is calibrated to reduce I2C bus error.
  driver.voltage_power_supply = kSupplyVoltage;
  driver.voltage_limit = kVoltageLimit;
  driver.pwm_frequency = 16000; // Lower from 20kHz to avoid polluting I2C bus, very helpful when speed is slow.
  // 12k seems to still cause I2C bus issue.  16k seems reasonable.
  driver.init();
  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;

  // This block is rough calibrated for a meaningful angle tracking force
  motor.voltage_limit = kVoltageLimit;
  motor.voltage_sensor_align = kSensorAlignVoltage;
  motor.velocity_limit = kVelocityLimit;
  motor.PID_velocity.P = 0.5f; // 0.2f;
  motor.PID_velocity.I = 0.0f; // 10.0f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 1000.0f;
  motor.LPF_velocity.Tf = 0.01f;
  motor.P_angle.P = 40.0f;

  motor.init();
  motor.initFOC();

  commander.verbose = VerboseMode::nothing;
  commander.add('M', onMotor, "motor config");

  const float raw_angle = motor.shaftAngle();
  g_state.logical_angle_offset_rad = -raw_angle;
  g_state.target_angle_rad = 0.0f;
  g_state.filtered_velocity_rad_s = 0.0f;
  g_state.last_oob_pulse_ms = millis();
}

void loop() {
  motor.loopFOC();

  const unsigned long now_ms = millis();
  const unsigned long now_us = micros();

  if (g_last_control_micros == 0) {
    g_last_control_micros = now_us;
  }
  const float dt = static_cast<float>(now_us - g_last_control_micros) * 1e-6f;
  g_last_control_micros = now_us;

  const float raw_angle_rad = motor.shaftAngle();
  const float logical_angle_rad = raw_angle_rad + g_state.logical_angle_offset_rad;
  const float raw_velocity_rad_s = motor.shaftVelocity();

  const float tau = 0.01f;
  const float alpha = (dt > 0.0f) ? clampValue<float>(dt / (tau + dt), 0.0f, 1.0f) : 1.0f;
  g_state.filtered_velocity_rad_s += alpha * (raw_velocity_rad_s - g_state.filtered_velocity_rad_s);

  const float torque_command = computeCommandedTorque(logical_angle_rad, g_state.filtered_velocity_rad_s, now_ms);
  g_state.last_torque_command = torque_command;
  motor.move(torque_command);

  serviceSerialInput();

  g_foc_window_count++;
  if ((now_ms - g_last_foc_window_ms) >= kFocRateWindowMs) {
    const unsigned long elapsed_ms = now_ms - g_last_foc_window_ms;
    if (elapsed_ms > 0) {
      g_state.foc_control_rate_hz = static_cast<long>((static_cast<unsigned long>(g_foc_window_count) * 1000UL) / elapsed_ms);
    }
    g_last_foc_window_ms = now_ms;
    g_foc_window_count = 0;
  }

  const uint16_t telemetry_interval = clampValue<uint16_t>(g_params.telemetry_interval_ms, 1, 200);
  if ((now_ms - g_last_telemetry_ms) >= telemetry_interval) {
    g_last_telemetry_ms = now_ms;
    sendTelemetry(logical_angle_rad);
  }
}
