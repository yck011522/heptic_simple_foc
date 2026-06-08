import argparse
import subprocess
import time

import serial

from common import DEFAULT_PLATFORMIO, discover_target_devices, prompt_yes_no
from device_discovery import BAUD, list_ports_by_exact_vid_pid, probe_port


# Default post-upload runtime configuration (wire-format integer values).
DEFAULT_BOOTSTRAP_TARGET_DECIDEG = 0
DEFAULT_BOOTSTRAP_MIN_DECIDEG = -3600
DEFAULT_BOOTSTRAP_MAX_DECIDEG = 3600
DEFAULT_BOOTSTRAP_SEQ_START = 1000

DEFAULT_BOOTSTRAP_PARAMS: tuple[tuple[str, int], ...] = (
    ("tracking_kp", 6000),
    ("tracking_kd", 2),
    ("detent_kp", 5000),
    ("bounds_kp", 10000),
    ("tracking_max_torque", 6000),
    ("bounds_max_torque", 300),
    ("detent_max_torque", 200),
    ("oob_kick_amplitude", 3000),
    ("vibration_amplitude", 300),
    ("vibration_pulse_interval_ms", 1000),
    ("oob_kick_pulse_interval_ms", 40),
    ("enable_tracking", 1),
    ("enable_bounds_restoration", 1),
    ("enable_oob_kick", 1),
    ("enable_detent", 0),
    ("enable_vibration", 0),
    ("telemetry_interval", 20),
)

POST_UPLOAD_PORT_READY_TIMEOUT_S = 35.0
POST_UPLOAD_TELEMETRY_TIMEOUT_S = 8.0
POST_UPLOAD_TELEMETRY_POLL_WINDOW_S = 2.0
POST_UPLOAD_ACK_TIMEOUT_S = 2.0
POST_UPLOAD_RETRY_INTERVAL_S = 0.25


def discover_upload_targets(vid: int, pid: int, baud: int = BAUD) -> list[dict]:
    """Return all matching ports, with probe info when available.

    Upload should target every connected controller matching VID/PID,
    even if protocol probing fails on some units.
    """

    exact_ports = list_ports_by_exact_vid_pid(vid, pid)
    if not exact_ports:
        return []

    probed_by_port = {device["port"]: device for device in discover_target_devices(vid=vid, pid=pid, baud=baud)}
    targets = []
    for candidate in exact_ports:
        port = candidate["port"]
        probed = probed_by_port.get(port)
        if probed is None:
            # Best effort: one direct probe attempt for metadata in logs.
            probed = probe_port(port, baud=baud)

        targets.append(
            {
                "port": port,
                "description": candidate.get("description", "unknown"),
                "fw_version": (probed or {}).get("fw_version", "unknown"),
                "dial_id": (probed or {}).get("dial_id", "unknown"),
                "probed": probed is not None,
            }
        )

    return targets


def upload_to_port(platformio: str, environment: str, port: str, retries: int = 1, retry_delay_s: float = 1.5) -> None:
    """Upload firmware to one port with a small retry window for reconnect races."""

    last_error = None
    for attempt in range(1, retries + 2):
        try:
            subprocess.run(
                [
                    platformio,
                    "run",
                    "--environment",
                    environment,
                    "--target",
                    "upload",
                    "--upload-port",
                    port,
                ],
                check=True,
            )
            return
        except subprocess.CalledProcessError as error:
            last_error = error
            if attempt <= retries:
                print(f"Upload to {port} failed on attempt {attempt}; retrying...")
                time.sleep(retry_delay_s)

    if last_error is not None:
        raise last_error


def wait_for_port_and_streaming_telemetry(
    port: str,
    baud: int,
    timeout_s: float = POST_UPLOAD_PORT_READY_TIMEOUT_S,
) -> tuple[serial.Serial, str]:
    """Retry reconnect/read cycles until telemetry starts streaming."""

    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None

    while time.monotonic() < deadline:
        ser = None
        try:
            ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(0.25)

            poll_timeout = min(POST_UPLOAD_TELEMETRY_POLL_WINDOW_S, max(0.2, deadline - time.monotonic()))
            telemetry_line = wait_for_prefix(ser, "T,", timeout_s=poll_timeout)
            return ser, telemetry_line
        except (serial.SerialException, OSError, TimeoutError) as error:
            last_error = error
            if ser is not None:
                ser.close()
            time.sleep(POST_UPLOAD_RETRY_INTERVAL_S)

    if last_error is not None:
        raise TimeoutError(f"Timed out waiting for {port} telemetry after upload ({last_error})")
    raise TimeoutError(f"Timed out waiting for {port} telemetry after upload")


def wait_for_prefix(ser: serial.Serial, prefix: str, timeout_s: float) -> str:
    """Read serial lines until one starts with the requested prefix."""

    deadline = time.monotonic() + timeout_s
    buffer = ""
    while time.monotonic() < deadline:
        data = ser.read(ser.in_waiting or 1)
        if not data:
            continue

        buffer += data.decode(errors="ignore")
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            if line.startswith(prefix):
                return line

    raise TimeoutError(f"Timed out waiting for serial line starting with '{prefix}'")


def parse_telemetry_seq(line: str) -> int | None:
    parts = line.split(",")
    if len(parts) < 3 or parts[0] != "T":
        return None
    try:
        return int(parts[2])
    except ValueError:
        return None


def wait_for_telemetry_seq(ser: serial.Serial, expected_seq: int, timeout_s: float) -> str:
    """Wait for telemetry whose echoed control seq matches expected_seq."""

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = wait_for_prefix(ser, "T,", timeout_s=max(0.05, deadline - time.monotonic()))
        telemetry_seq = parse_telemetry_seq(line)
        if telemetry_seq == expected_seq:
            return line

    raise TimeoutError(f"Timed out waiting for telemetry seq={expected_seq}")


def send_and_wait(ser: serial.Serial, command: str, ack_prefix: str, timeout_s: float = POST_UPLOAD_ACK_TIMEOUT_S) -> str:
    ser.reset_input_buffer()
    ser.write((command + "\n").encode())
    return wait_for_prefix(ser, ack_prefix, timeout_s)


def bootstrap_uploaded_device(port: str, baud: int = BAUD) -> None:
    """After a successful upload, wait for telemetry then apply default runtime settings."""

    ser, telemetry_line = wait_for_port_and_streaming_telemetry(port, baud=baud)
    try:
        print(f"  Telemetry detected on {port}: {telemetry_line}")

        seq = DEFAULT_BOOTSTRAP_SEQ_START

        # Rebase logical position to zero first to avoid stale offset behavior.
        send_and_wait(ser, f"R,{seq},{DEFAULT_BOOTSTRAP_TARGET_DECIDEG}", f"R,{seq}")
        seq += 1

        for param, value in DEFAULT_BOOTSTRAP_PARAMS:
            send_and_wait(ser, f"S,{seq},{param},{value}", f"S,{seq}")
            seq += 1

        control_seq = seq
        ser.reset_input_buffer()
        ser.write(
            (
                f"C,{control_seq},{DEFAULT_BOOTSTRAP_TARGET_DECIDEG},"
                f"{DEFAULT_BOOTSTRAP_MIN_DECIDEG},{DEFAULT_BOOTSTRAP_MAX_DECIDEG}\n"
            ).encode()
        )
        applied_line = wait_for_telemetry_seq(ser, expected_seq=control_seq, timeout_s=POST_UPLOAD_TELEMETRY_TIMEOUT_S)
        print(f"  Defaults applied on {port}: {applied_line}")
    finally:
        ser.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Upload firmware to all connected controllers that match one exact VID/PID pair.")
    parser.add_argument("--platformio", default=str(DEFAULT_PLATFORMIO), help="Path to the PlatformIO executable")
    parser.add_argument("--environment", default="lolin32_lite", help="PlatformIO environment name")
    parser.add_argument("--baud", type=int, default=BAUD, help="Serial baud for post-upload initialization")
    parser.add_argument("--vid", type=lambda value: int(value, 0), default=0x1A86, help="USB VID to target")
    parser.add_argument("--pid", type=lambda value: int(value, 0), default=0x7523, help="USB PID to target")
    parser.add_argument("--yes", action="store_true", help="Deprecated alias; uploads are non-interactive by default")
    parser.add_argument("--confirm", action="store_true", help="Ask for confirmation before starting uploads")
    parser.add_argument(
        "--skip-post-config",
        action="store_true",
        help="Skip reconnecting after upload and applying default runtime settings",
    )
    args = parser.parse_args()

    devices = discover_upload_targets(vid=args.vid, pid=args.pid, baud=args.baud)
    if not devices:
        print("No matching controllers found.")
        return

    print("Controllers selected for upload:")
    for device in devices:
        probe_note = "" if device["probed"] else " (probe unavailable; uploading by VID/PID)"
        print(f"  {device['port']}: fw={device['fw_version']} dial_id={device['dial_id']}{probe_note}")

    if args.confirm and not prompt_yes_no("Upload the current firmware to all listed controllers?", default=False):
        print("Upload cancelled.")
        return

    upload_successes = []
    upload_failures = []
    config_successes = []
    config_failures = []
    for device in devices:
        port = device["port"]
        print(f"\nUploading to {port}...")
        try:
            upload_to_port(args.platformio, args.environment, port)
            upload_successes.append(port)
        except subprocess.CalledProcessError as error:
            upload_failures.append(port)
            print(f"Upload failed for {port} (exit code {error.returncode}). Continuing to next device.")
            continue

        if args.skip_post_config:
            continue

        print(f"Post-upload initialization on {port}...")
        try:
            bootstrap_uploaded_device(port, baud=args.baud)
            config_successes.append(port)
        except (TimeoutError, serial.SerialException, OSError, ValueError) as error:
            config_failures.append(port)
            print(f"Post-upload initialization failed for {port}: {error}")

    print("\nUpload complete.")
    print(f"  Upload successful: {len(upload_successes)}")
    print(f"  Upload failed: {len(upload_failures)}")
    if upload_successes:
        print("  Ports uploaded successfully: " + ", ".join(upload_successes))
    if upload_failures:
        print("  Ports failed upload: " + ", ".join(upload_failures))

    if not args.skip_post_config:
        print(f"  Post-config successful: {len(config_successes)}")
        print(f"  Post-config failed: {len(config_failures)}")
        if config_successes:
            print("  Ports post-configured successfully: " + ", ".join(config_successes))
        if config_failures:
            print("  Ports failed post-config: " + ", ".join(config_failures))


if __name__ == "__main__":
    main()