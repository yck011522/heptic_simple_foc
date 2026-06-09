import argparse
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial


BAUD = 230400
DEFAULT_PORT = "COM5"
DEFAULT_ENV = "main"
DEFAULT_FOC_MIN_HZ = 800
DEFAULT_TELEMETRY_MS = 10

ROOT_DIR = Path(__file__).resolve().parent.parent
PLATFORMIO_EXE = Path.home() / ".platformio" / "penv" / "Scripts" / "platformio.exe"


@dataclass
class Telemetry:
    dial_id: int
    seq: int
    ang_decideg: int
    spd_decideg_s: int
    tor_milli: int
    foc_rate_hz: int
    status_bits: int


class ProtocolTestError(RuntimeError):
    pass


@dataclass
class TestStats:
    malformed_telemetry_lines: int = 0
    valid_telemetry_lines: int = 0
    malformed_samples: list[str] | None = None

    def __post_init__(self):
        if self.malformed_samples is None:
            self.malformed_samples = []


def parse_telemetry(line: str) -> Telemetry:
    parts = line.strip().split(",")
    if len(parts) != 8 or parts[0] != "T":
        raise ProtocolTestError(f"Invalid telemetry line shape: {line!r}")

    try:
        return Telemetry(
            dial_id=int(parts[1]),
            seq=int(parts[2]),
            ang_decideg=int(parts[3]),
            spd_decideg_s=int(parts[4]),
            tor_milli=int(parts[5]),
            foc_rate_hz=int(parts[6]),
            status_bits=int(parts[7]),
        )
    except ValueError as exc:
        raise ProtocolTestError(f"Telemetry contains non-integer fields: {line!r}") from exc


def bit_is_set(value: int, bit: int) -> bool:
    return ((value >> bit) & 1) == 1


def read_line(ser: serial.Serial, timeout_s: float) -> str:
    if not hasattr(read_line, "_buffers"):
        read_line._buffers = {}

    buffers = read_line._buffers
    key = id(ser)
    if key not in buffers:
        buffers[key] = ""

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        raw = ser.read(ser.in_waiting or 1)
        if not raw:
            continue

        buffers[key] += raw.decode(errors="ignore")
        while "\n" in buffers[key]:
            line, remainder = buffers[key].split("\n", 1)
            buffers[key] = remainder
            line = line.strip()
            if line:
                return line

    raise TimeoutError(f"Timed out waiting for serial line within {timeout_s:.2f}s")


def wait_for_prefix(ser: serial.Serial, prefix: str, timeout_s: float) -> str:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = read_line(ser, timeout_s=max(0.01, deadline - time.monotonic()))
        if line.startswith(prefix):
            return line
    raise TimeoutError(f"Timed out waiting for prefix {prefix!r}")


def wait_for_telemetry(ser: serial.Serial, timeout_s: float, stats: TestStats | None = None) -> Telemetry:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        line = wait_for_prefix(ser, "T,", timeout_s=max(0.01, deadline - time.monotonic()))
        try:
            telemetry = parse_telemetry(line)
            if stats is not None:
                stats.valid_telemetry_lines += 1
            return telemetry
        except Exception as exc:
            last_error = exc
            if stats is not None:
                stats.malformed_telemetry_lines += 1
                if len(stats.malformed_samples) < 8:
                    stats.malformed_samples.append(str(exc))
            continue
    raise ProtocolTestError(f"Timed out waiting for valid telemetry line. Last parse error: {last_error}")


def wait_for_telemetry_seq(ser: serial.Serial, seq: int, timeout_s: float, stats: TestStats | None = None) -> Telemetry:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        t = wait_for_telemetry(ser, timeout_s=max(0.01, deadline - time.monotonic()), stats=stats)
        if t.seq == seq:
            return t
    raise TimeoutError(f"Timed out waiting for telemetry seq {seq}")


def send_cmd(ser: serial.Serial, command: str) -> None:
    ser.write((command + "\n").encode())
    ser.flush()


def send_cmd_expect_prefix(ser: serial.Serial, command: str, prefix: str, timeout_s: float = 1.0) -> str:
    send_cmd(ser, command)
    return wait_for_prefix(ser, prefix, timeout_s=timeout_s)


def flash_firmware(port: str, env_name: str) -> None:
    if not PLATFORMIO_EXE.exists():
        raise ProtocolTestError(f"PlatformIO executable not found: {PLATFORMIO_EXE}")

    cmd = [
        str(PLATFORMIO_EXE),
        "run",
        "--environment",
        env_name,
        "--target",
        "upload",
        "--upload-port",
        port,
    ]
    print(f"[STEP] Flashing firmware to {port}: {' '.join(cmd)}")
    completed = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    if completed.returncode != 0:
        print(completed.stdout)
        print(completed.stderr)
        raise ProtocolTestError(f"Flash failed with exit code {completed.returncode}")
    print("[PASS] Flash completed")


def open_and_wait_streaming_telemetry(port: str, baud: int, timeout_s: float) -> tuple[serial.Serial, Telemetry]:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None

    while time.monotonic() < deadline:
        ser = None
        try:
            ser = serial.Serial(port, baudrate=baud, timeout=0.1)
            time.sleep(0.25)
            ser.reset_input_buffer()
            t = wait_for_telemetry(ser, timeout_s=2.5)
            return ser, t
        except Exception as exc:
            last_error = exc
            if ser is not None:
                ser.close()
            time.sleep(0.3)

    raise ProtocolTestError(f"Telemetry did not start on {port} in time. Last error: {last_error}")


def wait_for_valid_telemetry_after_noise(ser: serial.Serial, timeout_s: float, stats: TestStats | None = None) -> Telemetry:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        line = read_line(ser, timeout_s=max(0.01, deadline - time.monotonic()))
        if not line.startswith("T,"):
            continue
        try:
            telemetry = parse_telemetry(line)
            if stats is not None:
                stats.valid_telemetry_lines += 1
            return telemetry
        except Exception as exc:
            last_error = exc
            if stats is not None:
                stats.malformed_telemetry_lines += 1
            continue
    raise ProtocolTestError(f"Unable to recover a valid telemetry line after noise: {last_error}")


def validate_protocol_and_performance(ser: serial.Serial, foc_min_hz: int, exercise_m: bool, stats: TestStats) -> None:
    seq = 100

    print("[STEP] Checking V/I/E command acks")
    v = send_cmd_expect_prefix(ser, f"V,{seq}", f"V,{seq},")
    seq += 1
    i = send_cmd_expect_prefix(ser, f"I,{seq}", f"I,{seq},")
    seq += 1
    e = send_cmd_expect_prefix(ser, f"E,{seq}", f"E,{seq}")
    seq += 1
    print(f"[PASS] V ack: {v}")
    print(f"[PASS] I ack: {i}")
    print(f"[PASS] E ack: {e}")

    print("[STEP] Checking S read/write for telemetry_interval")
    s_set = send_cmd_expect_prefix(ser, f"S,{seq},telemetry_interval,{DEFAULT_TELEMETRY_MS}", f"S,{seq},telemetry_interval,")
    if not s_set.endswith(f",{DEFAULT_TELEMETRY_MS}"):
        raise ProtocolTestError(f"Unexpected telemetry_interval set ack: {s_set}")
    seq += 1

    s_get = send_cmd_expect_prefix(ser, f"S,{seq},telemetry_interval", f"S,{seq},telemetry_interval,")
    if not s_get.endswith(f",{DEFAULT_TELEMETRY_MS}"):
        raise ProtocolTestError(f"Unexpected telemetry_interval query ack: {s_get}")
    seq += 1
    print(f"[PASS] S ack set: {s_set}")
    print(f"[PASS] S ack get: {s_get}")

    print("[STEP] Checking C apply timing and seq echo")
    c_seq = seq
    send_t0 = time.monotonic()
    send_cmd(ser, f"C,{c_seq},0,-108000,108000")
    t_for_seq = wait_for_telemetry_seq(ser, c_seq, timeout_s=0.6, stats=stats)
    latency_ms = (time.monotonic() - send_t0) * 1000.0
    if latency_ms > 100.0:
        raise ProtocolTestError(f"C->telemetry echo latency too high: {latency_ms:.1f}ms (>100ms)")
    seq += 1
    print(f"[PASS] C seq echo latency: {latency_ms:.1f}ms | telemetry: {t_for_seq}")

    print("[STEP] Checking malformed bounds handling and fault bit")
    bad_seq = seq
    send_cmd(ser, f"C,{bad_seq},0,100,-100")
    t_after_bad = wait_for_telemetry(ser, timeout_s=0.4, stats=stats)
    if t_after_bad.seq == bad_seq:
        raise ProtocolTestError("Invalid bounds C frame was incorrectly accepted")
    if not bit_is_set(t_after_bad.status_bits, 6):
        raise ProtocolTestError("Fault bit (bit 6) was not set after invalid C frame")
    seq += 1
    print(f"[PASS] Invalid C rejected and fault bit set: status_bits={t_after_bad.status_bits}")

    print("[STEP] Checking R rebase reflection")
    r_seq = seq
    target_pos = 1234
    send_cmd_expect_prefix(ser, f"R,{r_seq},{target_pos}", f"R,{r_seq}", timeout_s=0.6)
    t_after_r = wait_for_telemetry(ser, timeout_s=0.3, stats=stats)
    if abs(t_after_r.ang_decideg - target_pos) > 80:
        raise ProtocolTestError(
            f"R did not reflect expected angle quickly enough. got={t_after_r.ang_decideg}, expected~={target_pos}"
        )
    seq += 1
    print(f"[PASS] R reflected in telemetry: angle={t_after_r.ang_decideg}")

    if exercise_m:
        print("[STEP] Checking M always-on path and telemetry continuity")
        send_cmd(ser, "M")
        t_after_m = wait_for_valid_telemetry_after_noise(ser, timeout_s=1.0, stats=stats)
        print(f"[PASS] Telemetry recovered after M command: seq={t_after_m.seq}")

    print("[STEP] Checking simple motion response with C targets")
    send_cmd(ser, f"C,{seq},900,-108000,108000")
    seq += 1
    angles_pos = []
    start = time.monotonic()
    while time.monotonic() - start < 0.6:
                angles_pos.append(wait_for_telemetry(ser, timeout_s=0.2, stats=stats).ang_decideg)

    send_cmd(ser, f"C,{seq},-900,-108000,108000")
    seq += 1
    angles_neg = []
    start = time.monotonic()
    while time.monotonic() - start < 0.6:
                angles_neg.append(wait_for_telemetry(ser, timeout_s=0.2, stats=stats).ang_decideg)

    if not angles_pos or not angles_neg:
        raise ProtocolTestError("Insufficient telemetry samples for motion check")

    pos_shift = max(angles_pos) - min(angles_pos)
    neg_shift = max(angles_neg) - min(angles_neg)
    if pos_shift < 20 and neg_shift < 20:
        raise ProtocolTestError(
            f"Motor did not show expected movement to C targets. pos_shift={pos_shift}, neg_shift={neg_shift}"
        )
    print(f"[PASS] Motion observed: pos_shift={pos_shift} decideg, neg_shift={neg_shift} decideg")

    print("[STEP] Checking telemetry field constraints and FOC rate")
    rates = []
    for _ in range(25):
        t = wait_for_telemetry(ser, timeout_s=0.2, stats=stats)
        rates.append(t.foc_rate_hz)
        if abs(t.tor_milli) > 10000:
            raise ProtocolTestError(f"Torque telemetry exceeded clamp range: {t.tor_milli}")

    avg_rate = sum(rates) / len(rates)
    if avg_rate < foc_min_hz:
        raise ProtocolTestError(f"Average FOC rate too low: {avg_rate:.1f}Hz < {foc_min_hz}Hz")
    print(f"[PASS] Average FOC rate: {avg_rate:.1f}Hz")

    malformed = stats.malformed_telemetry_lines
    valid = max(1, stats.valid_telemetry_lines)
    malformed_ratio = malformed / float(valid)
    print(f"[INFO] Telemetry parse stats: valid={valid}, malformed={malformed}, ratio={malformed_ratio:.3f}")
    if stats.malformed_samples:
        print("[INFO] Malformed telemetry examples:")
        for sample in stats.malformed_samples:
            print(f"  - {sample}")
    if malformed_ratio > 0.05:
        raise ProtocolTestError(
            f"Telemetry corruption ratio too high: {malformed_ratio:.3f} (> 0.05)"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Flash and validate haptic controller against COMM protocol")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--environment", default=DEFAULT_ENV)
    parser.add_argument("--no-flash", action="store_true", help="Skip firmware upload")
    parser.add_argument("--boot-timeout", type=float, default=35.0, help="Seconds to wait for post-reboot telemetry")
    parser.add_argument("--foc-min-hz", type=int, default=DEFAULT_FOC_MIN_HZ)
    parser.add_argument("--exercise-m", action="store_true", help="Also send M command and verify telemetry recovery")
    args = parser.parse_args()

    try:
        if not args.no_flash:
            flash_firmware(args.port, args.environment)

        print(f"[STEP] Waiting for post-reboot telemetry on {args.port}")
        ser, first_telemetry = open_and_wait_streaming_telemetry(args.port, args.baud, timeout_s=args.boot_timeout)
        print(f"[PASS] Telemetry streaming detected: {first_telemetry}")

        try:
            stats = TestStats()
            validate_protocol_and_performance(
                ser,
                foc_min_hz=args.foc_min_hz,
                exercise_m=args.exercise_m,
                stats=stats,
            )
        finally:
            ser.close()

        print("[RESULT] All protocol/performance checks passed")
        return 0
    except Exception as exc:
        print(f"[FAIL] {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
