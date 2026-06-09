#include <SimpleFOC.h>
#include <Wire.h>

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
}

MagneticSensorI2C sensor(AS5600_I2C);
BLDCMotor motor(kMotorPolePairs);
BLDCDriver3PWM driver(kPhaseAPin, kPhaseBPin, kPhaseCPin, kEnablePin);

uint8_t dialID = 0; // Persistent identity of this dial |
uint32_t lastSequenceNumber = 0; //Sequence number of the last processed `C` command (0 if none received)
long focControlRate = 0; // FOC loop rate (Hz), measured over 200 ms windows. Range: 0–2000 |
long statusBits = 0; // Decimal ASCII bitfield describing runtime state |

// `status_bits` layout:

// - bit 0: tracking enabled
// - bit 1: bounds restoration enabled
// - bit 2: OOB kick enabled
// - bit 3: detent enabled
// - bit 4: vibration enabled
// - bit 5: currently out of bounds
// - bit 6: fault active

float trackingAngle = 0.0f;

float boundaryMin = -5.0f;
float boundaryMax = 5.0f;
float vibrationAmplitudeTorque = 0.2f;
unsigned long vibrationPeriodMs = 35.0f;

unsigned long lastStatusPrintMs = 0;
Commander commander(Serial);

void onTarget(char* cmd) {commander.scalar(&trackingAngle, cmd);}
void onMotor(char* cmd){ commander.motor(&motor,cmd); }

void setup() {
  Serial.begin(115200);
  delay(1000);

  SimpleFOCDebug::enable(&Serial);

  Wire.begin(kI2cSdaPin, kI2cSclPin, kI2cClockHz);
  Wire.setClock(kI2cClockHz);
  Wire.setTimeOut(25);
  sensor.min_elapsed_time = 5000; // 2000 microseconds ~ 500Hz update rate, which is sufficient for this application and reduces I2C bus load
  sensor.init(&Wire);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = kSupplyVoltage;
  driver.voltage_limit = kVoltageLimit;
  driver.pwm_frequency = 16000; // Lower from 20kHz to avoid polluting I2C bus, very helpful when speed is slow.
  // 12k seems to still cause I2C bus issue.  16k seems reasonable.
  driver.init();
  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;
  motor.torque_controller = TorqueControlType::voltage;

  motor.voltage_limit = kVoltageLimit;
  motor.voltage_sensor_align = kSensorAlignVoltage;
  motor.velocity_limit = kVelocityLimit;
  motor.PID_velocity.P = 0.5f; // 0.2f;
  motor.PID_velocity.I = 0.0f; // 10.0f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 1000.0f;
  motor.LPF_velocity.Tf = 0.01f;
  motor.P_angle.P = 40.0f;


  motor.useMonitoring(Serial);
  motor.monitor_downsample = 50;

  motor.init();
  motor.initFOC();
  trackingAngle = motor.shaftAngle();

  commander.add('T', onTarget, "target angle [rad]");
  commander.add('M',onMotor,"full motor config");

  Serial.println();
  Serial.println("SimpleFOC position control");
  Serial.print("SDA=");
  Serial.print(kI2cSdaPin);
  Serial.print(" SCL=");
  Serial.println(kI2cSclPin);
  Serial.print("PWM=");
  Serial.print(kPhaseAPin);
  Serial.print(",");
  Serial.print(kPhaseBPin);
  Serial.print(",");
  Serial.print(kPhaseCPin);
  Serial.print(" EN=");
  Serial.println(kEnablePin);
  Serial.println("Motor ready.");
  Serial.println("Send T<angle_in_radians> over serial, e.g. T3.14");
}

float lastTorque = 0.0f;
float currentAngle = 0.0f;
float currentVelocity = 0.0f;

unsigned long currentTimeMs = 0.0f;

void loop() {
  motor.loopFOC();

  currentAngle = motor.shaftAngle();
  currentVelocity = motor.shaftVelocity();
  currentTimeMs = millis();

  motor.move(trackingAngle);

  lastTorque = motor.current_sp;
  commander.run();

// Telemetry Format:
// T,<dial_id>,<seq>,<ang>,<spd>,<tor>,<foc_rate>,<status_bits>\n
// | Field      | Type   | Description |
// |------------|--------|-------------|
// | dial_id    | uint8  | Persistent identity of this dial |
// | seq        | uint32 | Sequence number of the last processed `C` command (0 if none received) |
// | ang        | long   | Current dial angle (decidegrees) |
// | spd        | long   | Current dial speed (decidegrees/s) |
// | tor        | long   | Current applied torque (milliamps) |
// | foc_rate   | long   | FOC loop rate (Hz), measured over 200 ms windows. Range: 0–2000 |
// | status_bits | long  | Decimal ASCII bitfield describing runtime state |

  const unsigned long now = millis();
  if (now - lastStatusPrintMs >= 200) {
    lastStatusPrintMs = now;
    Serial.print("T,");
    Serial.print(dialID);
    Serial.print(",");
    Serial.print(lastSequenceNumber);
    Serial.print(",");
    Serial.print(currentAngle, 4);
    Serial.print(",");
    Serial.print(currentVelocity, 4);
    Serial.print(",");
    Serial.print(lastTorque, 4);
    Serial.print(",");
    Serial.print(focControlRate, 4);
    Serial.print(",");
    Serial.println(statusBits, 4);
  }

  //Compute FOC control rate
  static unsigned long lastFocControlRateComputeMs = 0;
  static unsigned int focControlRateComputeCount = 0;
  focControlRateComputeCount++;
  if (now - lastFocControlRateComputeMs >= 200) {
    focControlRate = (focControlRateComputeCount * 1000) / (now - lastFocControlRateComputeMs);
    lastFocControlRateComputeMs = now;
    focControlRateComputeCount = 0;
  }
}
