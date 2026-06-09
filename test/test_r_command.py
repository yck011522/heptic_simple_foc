"""Focused regression test for protocol command R (Set Current Position).

The test checks two behaviors for each requested rebase target:
1) Telemetry angle quickly reports the requested logical angle.
2) The dial does not show a motion spike after rebase (angle spread / speed).

By default this test runs two targets in sequence: 0 and 1000 decideg.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import serial


BAUD = 115200
DEFAULT_PORT = "COM5"


@dataclass
class Telemetry:
    dial_id: int
    seq: int
    angle_decideg: int
    speed_decideg_s: int
    torque_milli: int
    foc_rate_hz: int
    status_bits: int


class RCommandTestError(RuntimeError):
    pass


class SerialLineReader:
    """Line reader that keeps partial serial chunks between reads."""

    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self._buf = ""

    def read_line(self, timeout_s: float) -> str:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            data = self.ser.read(self.ser.in_waiting or 1)
            if not data:
                continue

            self._buf += data.decode(errors="ignore")
            while "\n" in self._buf:
                line, self._buf = self._buf.split("\n", 1)
                line = line.strip()
                if line:
                    return line

        raise TimeoutError(f"Timed out waiting for line in {timeout_s:.2f}s")


def parse_telemetry(line: str) -> Telemetry:
    parts = line.split(",")
    if len(parts) != 8 or parts[0] != "T":
        raise RCommandTestError(f"Invalid telemetry shape: {line!r}")

    try:
        return Telemetry(
            dial_id=int(parts[1]),
            seq=int(parts[2]),
            angle_decideg=int(parts[3]),
            speed_decideg_s=int(parts[4]),
            torque_milli=int(parts[5]),
            foc_rate_hz=int(parts[6]),
            status_bits=int(parts[7]),
        )
    except ValueError as exc:
        raise RCommandTestError(f"Telemetry has non-integer field: {line!r}") from exc


def wait_for_prefix(reader: SerialLineReader, prefix: str, timeout_s: float) -> str:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            line = reader.read_line(max(0.01, deadline - time.monotonic()))
        except TimeoutError:
            continue
        if line.startswith(prefix):
            return line
    raise TimeoutError(f"Timed out waiting for prefix {prefix!r}")


def wait_for_telemetry(reader: SerialLineReader, timeout_s: float) -> Telemetry:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        line = wait_for_prefix(reader, "T,", max(0.01, deadline - time.monotonic()))
        try:
            return parse_telemetry(line)
        except Exception as exc:
            last_error = exc
            continue
    raise RCommandTestError(f"Could not parse valid telemetry in time. Last error: {last_error}")


def send_cmd(ser: serial.Serial, command: str) -> None:
    ser.write((command + "\n").encode())
    ser.flush()


def send_cmd_expect_prefix(reader: SerialLineReader, ser: serial.Serial, command: str, prefix: str, timeout_s: float = 0.8) -> str:
    send_cmd(ser, command)
    return wait_for_prefix(reader, prefix, timeout_s)


def collect_telemetry_window(reader: SerialLineReader, duration_s: float) -> list[Telemetry]:
    samples: list[Telemetry] = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        timeout_left = max(0.01, deadline - time.monotonic())
        try:
            samples.append(wait_for_telemetry(reader, timeout_s=timeout_left))
        except TimeoutError:
            # If no complete frame arrives before the window ends, finish gracefully.
            break
    return samples


def analyze_rebase_response(
    target_decideg: int,
    samples: list[Telemetry],
    angle_tolerance_decideg: int,
    max_spread_decideg: int,
    max_speed_decideg_s: int,
) -> None:
    if not samples:
        raise RCommandTestError("No telemetry samples collected after R command")

    first = samples[0]
    error = abs(first.angle_decideg - target_decideg)
    if error > angle_tolerance_decideg:
        raise RCommandTestError(
            f"First telemetry angle after R is too far from target. "
            f"target={target_decideg}, first={first.angle_decideg}, error={error}, tol={angle_tolerance_decideg}"
        )

    angles = [sample.angle_decideg for sample in samples]
    speeds = [abs(sample.speed_decideg_s) for sample in samples]

    spread = max(angles) - min(angles)
    max_speed = max(speeds)

    if spread > max_spread_decideg:
        raise RCommandTestError(
            f"Post-R angle spread too large (possible physical motion). spread={spread}, limit={max_spread_decideg}"
        )

    if max_speed > max_speed_decideg_s:
        raise RCommandTestError(
            f"Post-R max speed too large (possible physical motion). max_speed={max_speed}, limit={max_speed_decideg_s}"
        )


def run_r_test(
    reader: SerialLineReader,
    ser: serial.Serial,
    seq: int,
    target_decideg: int,
    angle_tolerance_decideg: int,
    max_spread_decideg: int,
    max_speed_decideg_s: int,
    observe_window_s: float,
    print_telemetry: bool,
) -> int:
    print(f"[STEP] Send R rebase to {target_decideg} decideg")
    send_cmd_expect_prefix(reader, ser, f"R,{seq},{target_decideg}", f"R,{seq}")

    samples = collect_telemetry_window(reader, duration_s=observe_window_s)

    if print_telemetry:
        print(f"[INFO] Telemetry after R target={target_decideg} ({len(samples)} samples):")
        for sample in samples:
            print(
                f"  T,{sample.dial_id},{sample.seq},{sample.angle_decideg},"
                f"{sample.speed_decideg_s},{sample.torque_milli},"
                f"{sample.foc_rate_hz},{sample.status_bits}"
            )

    analyze_rebase_response(
        target_decideg=target_decideg,
        samples=samples,
        angle_tolerance_decideg=angle_tolerance_decideg,
        max_spread_decideg=max_spread_decideg,
        max_speed_decideg_s=max_speed_decideg_s,
    )

    first = samples[0]
    spread = max(sample.angle_decideg for sample in samples) - min(sample.angle_decideg for sample in samples)
    max_speed = max(abs(sample.speed_decideg_s) for sample in samples)
    print(
        "[PASS] "
        f"R target={target_decideg}, first_angle={first.angle_decideg}, spread={spread}, max_speed={max_speed}"
    )

    return seq + 1


def run_low_seq_regression(reader: SerialLineReader, ser: serial.Serial, target_decideg: int) -> None:
    """Regression for R with an older sequence than the last C.

    Steps:
    1) Push a high-seq C target away from current location.
    2) Send R with a lower seq to rebase current position.
    3) Verify rebased angle sticks and no motion spike appears.
    """

    print("[STEP] Low-seq R regression (C seq high, R seq low)")

    # High-seq C to emulate a newer control frame already processed.
    send_cmd(ser, "C,5000,-5000,-108000,108000")
    time.sleep(0.08)

    # Low-seq R should still remain yank-free after rebase.
    send_cmd_expect_prefix(reader, ser, f"R,1,{target_decideg}", "R,1")
    samples = collect_telemetry_window(reader, duration_s=0.35)

    analyze_rebase_response(
        target_decideg=target_decideg,
        samples=samples,
        angle_tolerance_decideg=100,
        max_spread_decideg=120,
        max_speed_decideg_s=350,
    )

    spread = max(sample.angle_decideg for sample in samples) - min(sample.angle_decideg for sample in samples)
    max_speed = max(abs(sample.speed_decideg_s) for sample in samples)
    first = samples[0]
    print(
        "[PASS] "
        f"Low-seq R regression: first_angle={first.angle_decideg}, spread={spread}, max_speed={max_speed}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate R command rebase behavior without physical kick")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--observe-window", type=float, default=0.35, help="Seconds to monitor telemetry after each R")
    parser.add_argument("--angle-tolerance", type=int, default=80, help="Allowed first-sample angle error in decideg")
    parser.add_argument("--max-spread", type=int, default=80, help="Allowed angle spread after R in decideg")
    parser.add_argument("--max-speed", type=int, default=250, help="Allowed absolute speed after R in decideg/s")
    parser.add_argument("--start-seq", type=int, default=900, help="Starting sequence number for normal R tests")
    parser.add_argument(
        "--no-print-telemetry",
        action="store_true",
        help="Disable per-sample telemetry printing during R observation windows",
    )
    parser.add_argument(
        "--skip-low-seq-regression",
        action="store_true",
        help="Skip the regression case where R uses a lower sequence than prior C",
    )
    args = parser.parse_args()

    try:
        print(f"[STEP] Open {args.port} @ {args.baud}")
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        reader = SerialLineReader(ser)
        time.sleep(0.25)
        ser.reset_input_buffer()

        print("[STEP] Wait for telemetry stream")
        t0 = wait_for_telemetry(reader, timeout_s=2.5)
        print(f"[PASS] Telemetry detected: {t0}")

        # Reduce external effects so this test isolates R behavior itself.
        seq = args.start_seq
        print("[STEP] Configure quiet force environment for R test")
        send_cmd_expect_prefix(reader, ser, f"S,{seq},enable_bounds_restoration,0", f"S,{seq},")
        seq += 1
        send_cmd_expect_prefix(reader, ser, f"S,{seq},enable_oob_kick,0", f"S,{seq},")
        seq += 1
        send_cmd_expect_prefix(reader, ser, f"S,{seq},telemetry_interval,10", f"S,{seq},")
        seq += 1

        # Wide bounds + C frame keeps protocol state deterministic for later telemetry.
        send_cmd(ser, f"C,{seq},0,-108000,108000")
        seq += 1

        seq = run_r_test(
            reader,
            ser,
            seq,
            target_decideg=0,
            angle_tolerance_decideg=args.angle_tolerance,
            max_spread_decideg=args.max_spread,
            max_speed_decideg_s=args.max_speed,
            observe_window_s=args.observe_window,
            print_telemetry=not args.no_print_telemetry,
        )

        seq = run_r_test(
            reader,
            ser,
            seq,
            target_decideg=1000,
            angle_tolerance_decideg=args.angle_tolerance,
            max_spread_decideg=args.max_spread,
            max_speed_decideg_s=args.max_speed,
            observe_window_s=args.observe_window,
            print_telemetry=not args.no_print_telemetry,
        )

        if not args.skip_low_seq_regression:
            run_low_seq_regression(reader, ser, target_decideg=1000)

        print("[RESULT] R command rebase behavior PASSED")
        return 0
    except Exception as exc:
        print(f"[FAIL] {exc}")
        return 1
    finally:
        try:
            ser.close()  # type: ignore[name-defined]
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
