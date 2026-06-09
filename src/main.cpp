#include <SimpleFOC.h>
#include <Preferences.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Firmware architecture notes:
// 1) All protocol wire values are integer CSV fields.
// 2) Control math is run in normal units (rad, rad/s, voltage-mode torque command).
// 3) Unit conversion happens only at parser/telemetry boundaries.
namespace {
constexpr int kI2cSdaPin = 19;          // I2C SDA GPIO used by AS5600 sensor.
constexpr int kI2cSclPin = 18;          // I2C SCL GPIO used by AS5600 sensor.
constexpr uint32_t kI2cClockHz = 400000; // Sensor bus speed (400kHz, calibrated stable).
constexpr int kPhaseAPin = 32;          // Driver phase A PWM pin.
constexpr int kPhaseBPin = 33;          // Driver phase B PWM pin.
constexpr int kPhaseCPin = 25;          // Driver phase C PWM pin.
constexpr int kEnablePin = 12;          // Driver enable pin.
constexpr int kMotorPolePairs = 14;     // Motor electrical pole-pair count.
constexpr float kSupplyVoltage = 12.0f; // Physical driver supply rail voltage (V).
constexpr float kVoltageLimit = 10.0f; // Do not use 12V or the motor will feel clicky.
constexpr float kSensorAlignVoltage = 6.0f; // Sensor alignment voltage (V) used by initFOC().
constexpr float kVelocityLimit = 40.0f; // Motor velocity limit used by internal SimpleFOC guards.

constexpr uint32_t kSerialBaud = 115200; // Protocol UART baud rate.
constexpr uint16_t kDefaultTelemetryIntervalMs = 10; // Default telemetry period (ms).
constexpr uint32_t kFocRateWindowMs = 200; // Rolling window (ms) for FOC loop-rate estimate.
constexpr uint32_t kFaultHoldMs = 500; // Protocol fault latch duration (ms).
constexpr size_t kMaxProtocolLineLength = 127; // Excluding newline.
constexpr float kRadPerDecideg = PI / 1800.0f; // Wire angle conversion factor.
constexpr float kDecidegPerRad = 1800.0f / PI; // Inverse wire angle conversion factor.
constexpr char kFirmwareVersion[] = "0.5.0"; // V command reported firmware version.
}

// status_bits layout sent in telemetry (T line field #8).
enum StatusBit : uint8_t {
  kStatusTrackingEnabled = 0,
  kStatusBoundsEnabled = 1,
  kStatusOobKickEnabled = 2,
  kStatusDetentEnabled = 3,
  kStatusVibrationEnabled = 4,
  kStatusOutOfBounds = 5,
  kStatusFaultActive = 6,
};

// Tunable runtime parameters exposed by S command and persisted in NVS.
struct RuntimeParams {
  float tracking_kp = 10.0f;         // Tracking proportional gain [torque/rad].
  float tracking_kd = 0.1f;          // Tracking derivative gain [torque/(rad/s)].
  float tracking_max_torque = 5.0f;  // Tracking torque clamp [V-equivalent].

  float bounds_kp = 20.0f;          // Bounds spring gain [torque/rad].
  float bounds_max_torque = 1.0f;   // Bounds torque clamp [V-equivalent].

  float detent_kp = 5.0f;                           // Stored detent gain (currently not applied).
  float detent_distance_rad = 10.0f * PI / 180.0f; // Stored detent spacing [rad].
  float detent_max_torque = 1.0f;                  // Stored detent torque clamp.

  float vibration_amplitude = 1.0f;              // Stored vibration amplitude (not applied).
  uint16_t vibration_pulse_interval_ms = 1000;   // Stored vibration period [ms].

  float oob_kick_amplitude = 2.0f;             // OOB kick amplitude [V-equivalent].
  uint16_t oob_kick_pulse_interval_ms = 40;    // OOB pulse toggle period [ms].

  bool enable_tracking = true;            // status_bits bit0 default ON.
  bool enable_bounds_restoration = true;  // status_bits bit1 default ON.
  bool enable_oob_kick = true;            // status_bits bit2 default ON.
  bool enable_detent = false;             // status_bits bit3 default OFF.
  bool enable_vibration = false;          // status_bits bit4 default OFF.

  uint16_t telemetry_interval_ms = kDefaultTelemetryIntervalMs; // Telemetry period [ms].
};

// Live control and telemetry state updated each loop iteration.
struct ControlState {
  uint8_t dial_id = 0;      // Persistent dial identity; 0 means unconfigured.
  uint32_t last_c_seq = 0;  // Last accepted C command sequence.

  float target_angle_rad = 0.0f; // Latest target angle from C command [rad].
  float bound_min_rad = -5.0f;   // Active soft lower bound [rad].
  float bound_max_rad = 5.0f;    // Active soft upper bound [rad].

  float logical_angle_offset_rad = 0.0f; // Logical frame offset applied to raw sensor angle.
  float filtered_velocity_rad_s = 0.0f;  // Low-pass filtered shaft velocity [rad/s].

  float last_torque_command = 0.0f;    // Last commanded torque value sent to motor.move().
  bool oob_pulse_on = false;           // Current OOB pulse phase state.
  unsigned long last_oob_pulse_ms = 0; // Last OOB pulse phase toggle timestamp.

  bool out_of_bounds = false;      // True when logical angle is outside current bounds.
  long foc_control_rate_hz = 0;    // Estimated loop frequency from rolling window.
  uint32_t status_bits = 0;        // Packed status bits emitted in telemetry.
};

MagneticSensorI2C sensor(AS5600_I2C); // AS5600 magnetic angle sensor interface.
BLDCMotor motor(kMotorPolePairs); // SimpleFOC motor object.
BLDCDriver3PWM driver(kPhaseAPin, kPhaseBPin, kPhaseCPin, kEnablePin); // 3PWM gate-driver interface.
Preferences preferences; // ESP32 NVS-backed key/value storage.
Commander commander(Serial); // SimpleFOC command parser used for M command compatibility.

RuntimeParams g_params; // Live + persisted S-parameter set.
ControlState g_state; // Live control/telemetry working state.

unsigned long g_last_telemetry_ms = 0;  // Last emitted telemetry timestamp.
unsigned long g_fault_until_ms = 0;     // Fault latch expiry timestamp.
unsigned long g_last_foc_window_ms = 0; // Start time for current FOC-rate window.
unsigned int g_foc_window_count = 0;    // loop() iterations accumulated in the current window.
unsigned long g_last_control_micros = 0; // Previous loop timestamp for dt calculation.

char g_line_buffer[kMaxProtocolLineLength + 1] = {0}; // Incoming protocol line assembly buffer.
size_t g_line_len = 0; // Current fill length in g_line_buffer.
bool g_line_overflow = false; // True once an overlength line is detected before newline.

// Generic clamp helper for numeric safety across protocol and control paths.
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

// Convert wire angle (decideg) to radians for internal control logic.
float decidegToRad(long decideg) {
  return static_cast<float>(decideg) * kRadPerDecideg;
}

// Convert internal angle (rad) to wire angle (decideg).
long radToDecideg(float rad) {
  return lroundf(rad * kDecidegPerRad);
}

// Convert internal speed (rad/s) to wire speed (decideg/s).
long radPerSecToDecidegPerSec(float rad_per_sec) {
  return lroundf(rad_per_sec * kDecidegPerRad);
}

// Convert voltage-mode torque command to wire telemetry units (millivolt-equivalent).
long torqueToMilliVolt(float torque_command) {
  const long raw_mv = lroundf(torque_command * 1000.0f);
  return clampValue<long>(raw_mv, -10000, 10000);
}

// Parse strict integer field used by protocol wire format.
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

// Parse sequence field as uint32-compatible integer.
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

// Latch a temporary protocol-fault window visible through status bit 6.
void latchProtocolFault() {
  g_fault_until_ms = millis() + kFaultHoldMs;
}

// Clear the fault latch after successfully processed frames.
void clearProtocolFault() {
  g_fault_until_ms = 0;
}

// Persist dial identity in NVS.
void persistDialId() {
  preferences.putUChar("dial_id", g_state.dial_id);
}

// Persist all runtime S parameters in NVS.
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

// Load persisted identity and runtime parameters from NVS with defaults.
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

// Commander callback for direct SimpleFOC M command support.
void onMotor(char* cmd) {
  commander.motor(&motor, cmd);
}

// Rebuild status_bits field from enable flags and fault/out-of-bounds state.
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

// Emit one telemetry frame using strict protocol integer fields.
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

// Handle S parameter read/write and return response value if known.
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
    if (is_set) g_params.oob_kick_amplitude = fabsf(static_cast<float>(set_value) / 1000.0f);
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

// Parse and execute one fully assembled protocol line.
// Supported commands: C, R, S, I, V, E, and non-CSV M passthrough.
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

// Read serial bytes, assemble newline-terminated lines, enforce max line length,
// and dispatch complete frames to handleProtocolLine().
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

// Compute composite torque from enabled haptic effects.
// Internally uses normal units and clamps final command to safe voltage limit.
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
      // OOB kick is implemented as an inward-torque ripple that reduces
      // the current restorative torque magnitude without flipping direction.
      // Positive wire amplitude therefore means "reduce inward torque by X".
      const float inward_direction = (logical_angle_rad < g_state.bound_min_rad) ? 1.0f : -1.0f;
      const float inward_component = torque_sum * inward_direction;
      if (inward_component > 0.0f) {
        const float reduction = fminf(g_params.oob_kick_amplitude, inward_component);
        torque_sum -= inward_direction * reduction;
      }
    }
  } else {
    g_state.oob_pulse_on = false;
  }

  return clampValue<float>(torque_sum, -kVoltageLimit, kVoltageLimit);
}

// Initialize serial, persistent settings, sensor/driver/motor, and protocol state.
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

// Real-time control loop:
// 1) Run FOC core.
// 2) Update filtered state and compute torque command.
// 3) Process serial protocol frames.
// 4) Update performance metrics and periodic telemetry.
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
