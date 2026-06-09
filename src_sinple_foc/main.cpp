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
constexpr float kVoltageLimit = 6.0f;
constexpr float kSensorAlignVoltage = 3.0f;
constexpr float kVelocityLimit = 20.0f;
}

MagneticSensorI2C sensor(AS5600_I2C);
BLDCMotor motor(kMotorPolePairs);
BLDCDriver3PWM driver(kPhaseAPin, kPhaseBPin, kPhaseCPin, kEnablePin);

float targetAngle = 0.0f;
unsigned long lastStatusPrintMs = 0;
Commander command(Serial);

void onTarget(char* cmd) {
  command.scalar(&targetAngle, cmd);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  SimpleFOCDebug::enable(&Serial);

  Wire.begin(kI2cSdaPin, kI2cSclPin, kI2cClockHz);
  Wire.setClock(kI2cClockHz);
  Wire.setTimeOut(2);

  sensor.init(&Wire);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = kSupplyVoltage;
  driver.voltage_limit = kVoltageLimit;
  driver.pwm_frequency = 30000;
  driver.init();
  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;
  motor.torque_controller = TorqueControlType::voltage;

  motor.voltage_limit = kVoltageLimit;
  motor.voltage_sensor_align = kSensorAlignVoltage;
  motor.velocity_limit = kVelocityLimit;
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 10.0f; // 10.0f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 1000.0f;
  motor.LPF_velocity.Tf = 0.01f;
  motor.P_angle.P = 40.0f;


  motor.useMonitoring(Serial);
  motor.monitor_downsample = 50;

  motor.init();
  motor.initFOC();
  targetAngle = motor.shaftAngle();

  command.add('T', onTarget, "target angle [rad]");

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

void loop() {
  motor.loopFOC();
  motor.move(targetAngle);
  command.run();

  const unsigned long now = millis();
  if (now - lastStatusPrintMs >= 200) {
    lastStatusPrintMs = now;
    Serial.print("target=");
    Serial.print(targetAngle, 4);
    Serial.print(" angle=");
    Serial.print(motor.shaftAngle(), 4);
    Serial.print(" velocity=");
    Serial.println(motor.shaftVelocity(), 4);
  }
}
