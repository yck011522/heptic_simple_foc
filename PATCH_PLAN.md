# PATCH PLAN - `src/main.cpp` Protocol + Torque-Control Migration

## 1) Objective

Modify the current firmware main script to:

1. Enforce the line-based communication contract in `HAPTIC_COMM_PROTOCOL.md`.
2. Remove non-protocol serial chatter and ad hoc debug/error prints.
3. Switch control loop from angle control to torque control (`MotionControlType::torque`).
4. Implement local position-tracking torque computation in firmware.
5. Preserve already calibrated electrical/timing parameters unless there is a hard protocol/safety reason to change.
6. Keep serial command compatibility for direct motor tuning via existing `M` command if feasible.
7. Keep internal control math in normal units (radians, rad/s, volts) and use wire-unit conversion only at protocol boundaries.

This plan only describes changes; it does not implement them yet.

---

## 2) Current State Summary (from `src/main.cpp`)

- Control mode is currently `MotionControlType::angle`.
- Tracking target is set as angle (`trackingAngle`) and sent to `motor.move(trackingAngle)`.
- Serial baud is `115200` (protocol requires `230400`).
- `SimpleFOCDebug::enable(&Serial)` and several startup `Serial.println(...)` lines generate non-protocol text.
- Telemetry is emitted as `T,...`, but angle/speed/torque currently printed as floats, not protocol integer wire units.
- Existing `Commander` is active with:
  - `T` command for target angle
  - `M` command for motor config (`commander.motor(&motor, cmd)`)

---

## 3) Calibrated Values To Preserve

The following values are treated as calibrated and should remain unchanged through this patch unless explicitly approved:

- I2C + sensor timing:
  - `kI2cSdaPin = 19`
  - `kI2cSclPin = 18`
  - `kI2cClockHz = 400000`
  - `Wire.setTimeOut(25)`
  - `sensor.min_elapsed_time = 2000`
- Driver/electrical:
  - `kSupplyVoltage = 12.0f`
  - `kVoltageLimit = 10.0f`
  - `driver.pwm_frequency = 16000`
- Motor setup:
  - `kMotorPolePairs = 14`
  - `kSensorAlignVoltage = 6.0f`
  - `kVelocityLimit = 40.0f`
  - Existing velocity filter/velocity PID config unless protocol migration requires specific adaptation.

Note: Current sensing will not be added in this patch. Torque control remains `TorqueControlType::voltage`, and telemetry `tor` will report voltage-mode command in millivolt-equivalent wire units.

---

## 4) Planned Architecture Changes

### 4.1 Protocol Engine (strict line parser with Commander-assisted `M` path)

Replace ad hoc command handling with a dedicated parser for protocol commands, while reusing SimpleFOC Commander only for `M` compatibility if it remains silent enough:

- Supported commands:
  - `C,<seq>,<target>,<min>,<max>`
  - `R,<seq>,<current_pos>`
  - `S,<seq>,<param_name>[,<value>]`
  - `I,<seq>[,<dial_id>]`
  - `V,<seq>`
  - `E,<seq>`
- Enforce:
  - Max line length 127 bytes (excluding `\n`)
  - Ignore `\r`
  - Integer-only wire fields for protocol transport
  - Unknown/malformed commands are ignored (no extra serial text), while fault state is reflected in `status_bits` bit 6

Implementation direction:

- Parse complete CSV lines in firmware first to preserve strict protocol behavior.
- Remove legacy Commander `T` command registration.
- Keep Commander only for direct `M...` handling path.

### 4.2 Runtime State Model

Introduce explicit runtime state structs:

- Control state:
  - `last_c_seq`, `target_decideg`, `bound_min_decideg`, `bound_max_decideg`
- Parameter state (`S` parameters):
  - `tracking_kp`, `tracking_kd`, `tracking_max_torque`
  - `bounds_kp`, `bounds_max_torque`
  - `oob_kick_amplitude`, `oob_kick_pulse_interval_ms`
  - detent/vibration parameters (stored and acknowledged even if effect disabled)
  - feature enables and `telemetry_interval`
- Fault/diagnostics state:
  - fault latch time window
  - `status_bits` computation source fields

### 4.3 Torque-Control Loop

Switch motor control configuration to:

- `motor.controller = MotionControlType::torque`
- `motor.torque_controller = TorqueControlType::voltage` (retained for now)

Compute commanded torque every loop using local position/velocity in normal units:

- Keep control math in radians and rad/s.
- Convert protocol `C` fields (`target/min/max`) from decidegrees to radians at parse time.
- Tracking torque (PD):
  - `error = target - current`
  - `t_track = kp * error - kd * velocity`
  - clamp to `tracking_max_torque`
- Bounds restoration torque:
  - if `current < min` or `current > max`, apply restoring spring with clamp `bounds_max_torque`
- OOB kick torque:
  - while out-of-bounds and enabled, apply periodic ripple/pulse signed toward in-bounds direction
- Composite torque:
  - sum enabled effect torques, saturate by safe limit (`kVoltageLimit` equivalent in voltage torque mode)
- Apply with `motor.move(torque_cmd)` where `torque_cmd` is the voltage-mode torque command.

### 4.4 Rebase (`R`) behavior

On `R,<seq>,<current_pos>`:

- Rebase logical angle offset so next telemetry reports the requested current position.
- Reset velocity estimate filter state used by protocol tracking controller.
- Reset derivative memory / pulse timing to avoid impulse.
- Update tracking target to `current_pos` unless newer `C` already processed.
- Respond exactly: `R,<seq>`.

### 4.5 Telemetry + Responses Only

Remove non-protocol prints and emit only protocol lines:

- Telemetry format:
  - `T,<dial_id>,<seq>,<ang>,<spd>,<tor>,<foc_rate>,<status_bits>`
- All numeric fields are integer ASCII per protocol units.
- Unit conversion is only at transport boundaries:
  - `ang`, `spd`: internal rad/rad/s -> decideg/decideg/s
  - `tor`: internal voltage command -> millivolt-equivalent integer
- Telemetry interval is runtime-configurable via `S` and defaults to protocol target.
- Command responses:
  - `I,<seq>,...`, `V,<seq>,...`, `E,<seq>`, `S,<seq>,...`, `R,<seq>`
- No startup banners, no Commander help text, no debug logs.

---

## 5) Commander `M` Compatibility Strategy

Goal: keep direct motor parameter command capability while preserving strict protocol traffic.

Plan:

1. Keep `Commander` object only for `M` command callback path.
2. Gate `commander.run()` behind a compatibility flag so it does not consume/emit unexpected text during normal protocol operation.
3. Default behavior: strict protocol mode enabled (no free-form chatter).
4. Optional compatibility mode:
   - Allow `M...` lines to be routed to commander.
  - Do not register legacy `T` command (superseded by protocol `C`).
5. If commander introduces unavoidable extra output, fallback plan is to implement minimal `M` parser wrapper with equivalent parameter access and no extra serial text.

Decision checkpoint during implementation:
- Verify whether current SimpleFOC commander path for `M` can run silently enough in strict mode.

---

## 6) Protocol Compliance Mapping

- `C`: accept and apply immediately if valid; reject invalid bounds (`min > max`) with fault bit set.
- `R`: apply immediate rebase + state reset + ack.
- `S`: support read/write for listed parameter names, persist values, ack unknown names with `?` value.
- `I`: read/write persistent `dial_id` (NVS/Preferences), ack with stored value.
- `V`: return firmware version string constant.
- `E`: echo ack.
- Telemetry starts automatically after serial open.

Wire/internal unit policy:

- Protocol transport remains integer CSV in wire units.
- Internal control state remains in rad, rad/s, and voltage-mode torque command units.
- Conversions are centralized helper functions at parser and telemetry edges.

---

## 7) Persistence Plan

Use ESP32 `Preferences` (NVS) namespace for:

- `dial_id`
- `S` parameters

Behavior:

- Load on boot with defaults from protocol table.
- Apply write immediately and persist.
- Keep implementation defensive against missing/uninitialized keys.

---

## 8) Concrete Edit Steps

1. Refactor `src/main.cpp` into sections:
  - Constants, unit conversion helpers, state structs, parser, command handlers, control synthesis, telemetry.
2. Update serial setup:
   - Set `Serial.begin(230400)`.
   - Remove debug/intro print block.
3. Switch to torque control mode and integrate custom torque synthesis.
4. Replace float telemetry output with integer wire-unit output.
5. Add strict line input buffer and parser (`\n`-terminated, max 127 bytes).
6. Implement `C/R/S/I/V/E` handlers and exact response lines.
7. Implement status bit computation and fault latch behavior.
8. Integrate persistence for identity and parameters.
9. Keep `M` support path with strict-mode safeguards and remove Commander `T` registration.
10. Run PlatformIO build for `env:main`; resolve compile issues.
11. Do a serial protocol sanity pass against example flows from protocol doc.

---

## 9) Validation Plan

### 9.1 Build checks

- `platformio run --environment main` succeeds.

### 9.2 Serial behavior checks

- On port open: only telemetry lines appear.
- No startup chatter, no debug text.
- `V`, `I`, `E`, `R`, `S` return exact CSV shape.
- `C` updates reflected via telemetry `seq` and motion response.
- Legacy `T...` command is no longer accepted.
- `M...` behavior works only through the defined compatibility path.
- Invalid lines set fault bit without extra prints.

### 9.3 Control checks

- Torque control active (not angle mode).
- Tracking toward target produced by local PD torque logic.
- Out-of-bounds behavior follows bounds spring + OOB kick settings.
- `R` rebase does not create a torque impulse.

### 9.4 Performance checks

- FOC rate remains near target (>= 800 Hz required).
- Telemetry interval parameter functions and remains stable.

---

## 10) Risks and Mitigations

- Risk: protocol strictness breaks existing ad hoc host tools.
  - Mitigation: keep optional `M` compatibility path and strict parser with clear command boundaries.
- Risk: voltage-mode torque value may be misinterpreted by host as current.
  - Mitigation: define `tor` as millivolt-equivalent in firmware notes and host integration docs; clamp safely.
- Risk: Commander integration may emit helper text unexpectedly.
  - Mitigation: isolate Commander to `M` route only; fallback to firmware-native `M` wrapper if needed.
- Risk: `R` rebase can produce derivative kick.
  - Mitigation: explicitly reset derivative/filter/pulse timing state during rebase.
- Risk: parser overflow or malformed frame handling can block loop timing.
  - Mitigation: O(1) incremental parser and immediate discard on overflow.

---

## 11) Out-of-Scope for This Patch

- Implementing fully functional detent and vibration effects (store/config only unless requested).
- Multi-dial support.
- Binary transport.
- Host-side software changes.

---

## 12) Review Questions Before Implementation

1. Should `M` compatibility be always-on, or enabled only by a dedicated runtime `S` parameter flag?
2. Confirm desired firmware version string for `V` response (e.g., `0.5.0`).
3. Confirm telemetry default interval should be `10 ms` (P5 target) unless overridden by persisted setting.
