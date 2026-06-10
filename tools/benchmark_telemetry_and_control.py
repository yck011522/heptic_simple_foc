"""Benchmark telemetry receive and control send rates across all connected haptic devices.

Experiment 1:
- Discover up to 12 devices
- Set telemetry interval to 10 ms (100 Hz)
- Let devices settle for 2 seconds
- Measure telemetry receive rate for 10 seconds

Experiment 2:
- Keep telemetry at 10 ms
- Send control frames at 100 Hz to every device for 10 seconds
- Measure both actual control send rate and telemetry receive rate

A Markdown report is written next to this script with the same basename.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path

import serial

from device_discovery import BAUD, DRAIN_DELAY, discover_devices


MAX_DEVICES = 12
TARGET_TELEMETRY_INTERVAL_MS = 10
SETTLE_SECONDS = 2.0
MEASURE_SECONDS = 10.0


@dataclass
class DeviceStats:
    port: str
    fw_version: str
    dial_id: int
    telemetry_count: int = 0
    control_count: int = 0


class DeviceSession:
    def __init__(self, port: str, fw_version: str, dial_id: int, baud: int = BAUD):
        self.stats = DeviceStats(port=port, fw_version=fw_version, dial_id=dial_id)
        self.ser = serial.Serial(port, baud, timeout=0.0)
        self._buf = ""
        self._seq = 1

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def reset_counts(self) -> None:
        self.stats.telemetry_count = 0
        self.stats.control_count = 0

    def next_seq(self) -> int:
        seq = self._seq
        self._seq += 1
        return seq

    def send_line(self, line: str) -> None:
        self.ser.write((line + "\n").encode())

    def poll_lines(self) -> None:
        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except (serial.SerialException, OSError):
            return

        if not data:
            return

        self._buf += data.decode(errors="ignore")
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.strip()
            if line.startswith("T,"):
                self.stats.telemetry_count += 1

    def send_set_telemetry_interval(self, interval_ms: int, timeout_s: float = 1.5) -> bool:
        seq = self.next_seq()
        cmd = f"S,{seq},telemetry_interval,{interval_ms}"
        prefix = f"S,{seq},telemetry_interval,"

        self.ser.reset_input_buffer()
        self.send_line(cmd)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll_lines()
            # Parse non-telemetry responses from buffered text quickly
            if "\n" not in self._buf:
                # Try to read at least one char if idle to advance buffered parsing
                try:
                    data = self.ser.read(self.ser.in_waiting or 1)
                except (serial.SerialException, OSError):
                    return False
                if data:
                    self._buf += data.decode(errors="ignore")

            while "\n" in self._buf:
                line, self._buf = self._buf.split("\n", 1)
                line = line.strip()
                if line.startswith("T,"):
                    self.stats.telemetry_count += 1
                    continue
                if line.startswith(prefix):
                    return True

        return False


def discover_sessions(baud: int, max_devices: int = MAX_DEVICES) -> list[DeviceSession]:
    discovered = discover_devices(baud=baud, filter_by_vid_pid=True)
    discovered = discovered[:max_devices]

    sessions: list[DeviceSession] = []
    for item in discovered:
        try:
            session = DeviceSession(
                port=item["port"],
                fw_version=item.get("fw_version", "unknown"),
                dial_id=int(item.get("dial_id", 0)),
                baud=baud,
            )
            sessions.append(session)
        except (serial.SerialException, OSError, ValueError):
            continue

    return sessions


def run_telemetry_only_experiment(sessions: list[DeviceSession], settle_s: float, measure_s: float) -> dict:
    # Drain startup noise and make sure all streams are active.
    end = time.monotonic() + DRAIN_DELAY
    while time.monotonic() < end:
        for session in sessions:
            session.poll_lines()

    # Settle period: allow control loops and streams to stabilize.
    settle_end = time.monotonic() + settle_s
    while time.monotonic() < settle_end:
        for session in sessions:
            session.poll_lines()
        time.sleep(0.001)

    for session in sessions:
        session.reset_counts()

    t0 = time.monotonic()
    measure_end = t0 + measure_s
    while time.monotonic() < measure_end:
        for session in sessions:
            session.poll_lines()
        time.sleep(0.001)
    t1 = time.monotonic()

    duration = t1 - t0
    per_device = []
    for session in sessions:
        count = session.stats.telemetry_count
        per_device.append(
            {
                "port": session.stats.port,
                "dial_id": session.stats.dial_id,
                "count": count,
                "rx_hz": count / duration if duration > 0 else 0.0,
            }
        )

    return {"duration": duration, "per_device": per_device}


def run_control_and_telemetry_experiment(sessions: list[DeviceSession], measure_s: float) -> dict:
    for session in sessions:
        session.reset_counts()

    t0 = time.monotonic()
    end_time = t0 + measure_s
    while time.monotonic() < end_time:
        for session in sessions:
            session.poll_lines()

        # Unthrottled sender: write one control frame per device each loop.
        for session in sessions:
            seq = session.next_seq()
            session.send_line(f"C,{seq},0,-500,500")
            session.stats.control_count += 1

    # Drain a short tail so telemetry generated near the end is counted.
    drain_until = time.monotonic() + 0.15
    while time.monotonic() < drain_until:
        for session in sessions:
            session.poll_lines()

    t1 = time.monotonic()
    duration = t1 - t0

    per_device = []
    for session in sessions:
        tx_count = session.stats.control_count
        rx_count = session.stats.telemetry_count
        per_device.append(
            {
                "port": session.stats.port,
                "dial_id": session.stats.dial_id,
                "control_count": tx_count,
                "tx_hz": tx_count / duration if duration > 0 else 0.0,
                "telemetry_count": rx_count,
                "rx_hz": rx_count / duration if duration > 0 else 0.0,
            }
        )

    return {"duration": duration, "per_device": per_device}


def markdown_table(headers: list[str], rows: list[list[str]]) -> str:
    head = "| " + " | ".join(headers) + " |"
    sep = "| " + " | ".join(["---"] * len(headers)) + " |"
    body = ["| " + " | ".join(row) + " |" for row in rows]
    return "\n".join([head, sep] + body)


def summarize_rates(values: list[float]) -> str:
    if not values:
        return "n/a"
    return f"min={min(values):.2f} Hz, avg={sum(values)/len(values):.2f} Hz, max={max(values):.2f} Hz"


def write_report(
    report_path: Path,
    sessions: list[DeviceSession],
    telemetry_result: dict,
    control_result: dict,
    telemetry_set_failures: list[str],
) -> None:
    ts = time.strftime("%Y-%m-%d %H:%M:%S")

    device_rows = []
    for session in sessions:
        device_rows.append(
            [
                session.stats.port,
                str(session.stats.dial_id),
                session.stats.fw_version,
            ]
        )

    tele_rows = []
    tele_rates = []
    for item in telemetry_result["per_device"]:
        tele_rates.append(item["rx_hz"])
        tele_rows.append(
            [
                item["port"],
                str(item["dial_id"]),
                str(item["count"]),
                f"{item['rx_hz']:.2f}",
            ]
        )

    control_rows = []
    tx_rates = []
    rx_rates = []
    for item in control_result["per_device"]:
        tx_rates.append(item["tx_hz"])
        rx_rates.append(item["rx_hz"])
        control_rows.append(
            [
                item["port"],
                str(item["dial_id"]),
                str(item["control_count"]),
                f"{item['tx_hz']:.2f}",
                str(item["telemetry_count"]),
                f"{item['rx_hz']:.2f}",
            ]
        )

    lines = []
    lines.append("# Haptic Device Rate Benchmark")
    lines.append("")
    lines.append(f"Generated: {ts}")
    lines.append("")
    lines.append("## Configuration")
    lines.append("")
    lines.append(f"- Devices discovered: {len(sessions)}")
    lines.append(f"- Telemetry interval setpoint: {TARGET_TELEMETRY_INTERVAL_MS} ms (100 Hz)")
    lines.append(f"- Experiment 1 settle: {SETTLE_SECONDS:.1f} s")
    lines.append(f"- Experiment 1 measure: {telemetry_result['duration']:.3f} s")
    lines.append(f"- Experiment 2 measure: {control_result['duration']:.3f} s")
    lines.append("- Control frame: `C,<seq>,0,-500,500` sent unthrottled (max host throughput)")
    if telemetry_set_failures:
        lines.append(f"- WARNING: telemetry interval command failed on: {', '.join(telemetry_set_failures)}")
    lines.append("")
    lines.append("## Devices")
    lines.append("")
    lines.append(markdown_table(["Port", "Dial ID", "FW Version"], device_rows))
    lines.append("")
    lines.append("## Experiment 1: Telemetry Receive Rate (No Control Streaming)")
    lines.append("")
    lines.append(markdown_table(["Port", "Dial ID", "Telemetry Frames", "Measured Rx Hz"], tele_rows))
    lines.append("")
    lines.append(f"Summary: {summarize_rates(tele_rates)}")
    lines.append("")
    lines.append("## Experiment 2: Control Send Rate and Telemetry Receive Rate")
    lines.append("")
    lines.append(
        markdown_table(
            ["Port", "Dial ID", "Control Frames Sent", "Measured Tx Hz", "Telemetry Frames", "Measured Rx Hz"],
            control_rows,
        )
    )
    lines.append("")
    lines.append(f"Control send summary: {summarize_rates(tx_rates)}")
    lines.append("")
    lines.append(f"Telemetry receive summary (during control): {summarize_rates(rx_rates)}")
    lines.append("")

    report_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Benchmark multi-device telemetry and control rates.")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--max-devices", type=int, default=MAX_DEVICES)
    parser.add_argument("--settle", type=float, default=SETTLE_SECONDS)
    parser.add_argument("--measure", type=float, default=MEASURE_SECONDS)
    args = parser.parse_args()

    report_path = Path(__file__).with_suffix(".md")

    sessions = discover_sessions(baud=args.baud, max_devices=min(MAX_DEVICES, max(1, args.max_devices)))
    if not sessions:
        print("No haptic devices discovered.")
        return 1

    print(f"Discovered {len(sessions)} device(s):")
    for session in sessions:
        print(f"  {session.stats.port}  fw={session.stats.fw_version}  dial_id={session.stats.dial_id}")

    telemetry_set_failures: list[str] = []

    try:
        time.sleep(DRAIN_DELAY)
        for session in sessions:
            ok = session.send_set_telemetry_interval(TARGET_TELEMETRY_INTERVAL_MS)
            if not ok:
                telemetry_set_failures.append(session.stats.port)

        telemetry_result = run_telemetry_only_experiment(
            sessions=sessions,
            settle_s=args.settle,
            measure_s=args.measure,
        )

        control_result = run_control_and_telemetry_experiment(
            sessions=sessions,
            measure_s=args.measure,
        )

        write_report(
            report_path=report_path,
            sessions=sessions,
            telemetry_result=telemetry_result,
            control_result=control_result,
            telemetry_set_failures=telemetry_set_failures,
        )

        print(f"Report written: {report_path}")
        return 0
    finally:
        for session in sessions:
            session.close()


if __name__ == "__main__":
    raise SystemExit(main())
