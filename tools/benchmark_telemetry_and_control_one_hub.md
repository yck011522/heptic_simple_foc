# Haptic Device Rate Benchmark

Generated: 2026-06-10 14:25:56

## Configuration

- Devices discovered: 6
- Telemetry interval setpoint: 10 ms (100 Hz)
- Experiment 1 settle: 2.0 s
- Experiment 1 measure: 10.000 s
- Experiment 2 measure: 10.156 s
- Control frame: `C,<seq>,0,-500,500` sent unthrottled (max host throughput)
- WARNING: telemetry interval command failed on: COM21, COM25, COM26

## Devices

| Port | Dial ID | FW Version |
| --- | --- | --- |
| COM21 | 26 | 0.5.0 |
| COM22 | 25 | 0.5.0 |
| COM23 | 24 | 0.5.0 |
| COM24 | 23 | 0.5.0 |
| COM25 | 22 | 0.5.0 |
| COM26 | 21 | 0.5.0 |

## Experiment 1: Telemetry Receive Rate (No Control Streaming)

| Port | Dial ID | Telemetry Frames | Measured Rx Hz |
| --- | --- | --- | --- |
| COM21 | 26 | 1000 | 100.00 |
| COM22 | 25 | 1000 | 100.00 |
| COM23 | 24 | 1000 | 100.00 |
| COM24 | 23 | 1000 | 100.00 |
| COM25 | 22 | 1000 | 100.00 |
| COM26 | 21 | 1001 | 100.10 |

Summary: min=100.00 Hz, avg=100.02 Hz, max=100.10 Hz

## Experiment 2: Control Send Rate and Telemetry Receive Rate

| Port | Dial ID | Control Frames Sent | Measured Tx Hz | Telemetry Frames | Measured Rx Hz |
| --- | --- | --- | --- | --- | --- |
| COM21 | 26 | 5621 | 553.47 | 995 | 97.97 |
| COM22 | 25 | 5621 | 553.47 | 994 | 97.87 |
| COM23 | 24 | 5621 | 553.47 | 997 | 98.17 |
| COM24 | 23 | 5621 | 553.47 | 997 | 98.17 |
| COM25 | 22 | 5621 | 553.47 | 995 | 97.97 |
| COM26 | 21 | 5621 | 553.47 | 994 | 97.87 |

Control send summary: min=553.47 Hz, avg=553.47 Hz, max=553.47 Hz

Telemetry receive summary (during control): min=97.87 Hz, avg=98.00 Hz, max=98.17 Hz
