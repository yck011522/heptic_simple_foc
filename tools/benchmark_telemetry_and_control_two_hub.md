# Haptic Device Rate Benchmark

Generated: 2026-06-10 14:30:14

## Configuration

- Devices discovered: 6
- Telemetry interval setpoint: 10 ms (100 Hz)
- Experiment 1 settle: 2.0 s
- Experiment 1 measure: 10.000 s
- Experiment 2 measure: 10.156 s
- Control frame: `C,<seq>,0,-500,500` sent unthrottled (max host throughput)
- WARNING: telemetry interval command failed on: COM21, COM22, COM23, COM28

## Devices

| Port | Dial ID | FW Version |
| --- | --- | --- |
| COM21 | 26 | 0.5.0 |
| COM22 | 25 | 0.5.0 |
| COM23 | 24 | 0.5.0 |
| COM27 | 23 | 0.5.0 |
| COM28 | 22 | 0.5.0 |
| COM29 | 21 | 0.5.0 |

## Experiment 1: Telemetry Receive Rate (No Control Streaming)

| Port | Dial ID | Telemetry Frames | Measured Rx Hz |
| --- | --- | --- | --- |
| COM21 | 26 | 1000 | 100.00 |
| COM22 | 25 | 1000 | 100.00 |
| COM23 | 24 | 1000 | 100.00 |
| COM27 | 23 | 1000 | 100.00 |
| COM28 | 22 | 1000 | 100.00 |
| COM29 | 21 | 1000 | 100.00 |

Summary: min=100.00 Hz, avg=100.00 Hz, max=100.00 Hz

## Experiment 2: Control Send Rate and Telemetry Receive Rate

| Port | Dial ID | Control Frames Sent | Measured Tx Hz | Telemetry Frames | Measured Rx Hz |
| --- | --- | --- | --- | --- | --- |
| COM21 | 26 | 6475 | 637.55 | 978 | 96.30 |
| COM22 | 25 | 6475 | 637.55 | 978 | 96.30 |
| COM23 | 24 | 6475 | 637.55 | 982 | 96.69 |
| COM27 | 23 | 6475 | 637.55 | 977 | 96.20 |
| COM28 | 22 | 6475 | 637.55 | 977 | 96.20 |
| COM29 | 21 | 6475 | 637.55 | 976 | 96.10 |

Control send summary: min=637.55 Hz, avg=637.55 Hz, max=637.55 Hz

Telemetry receive summary (during control): min=96.10 Hz, avg=96.30 Hz, max=96.69 Hz
