import sys
import threading
import time
from pathlib import Path

import serial


ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from device_discovery import (
    BAUD,
    DRAIN_DELAY,
    DEFAULT_DEPLOY_PID,
    DEFAULT_DEPLOY_VID,
    assign_identity,
    list_ports_by_exact_vid_pid,
    probe_port,
)


DEFAULT_PLATFORMIO = Path.home() / ".platformio" / "penv" / "Scripts" / "platformio.exe"
DEFAULT_FIRMWARE_VERSION = "0.5.0"
DEFAULT_MOTION_DETECT_WINDOW = 2.0
DEFAULT_MOTION_THRESHOLD = 100


def discover_target_devices(vid=DEFAULT_DEPLOY_VID, pid=DEFAULT_DEPLOY_PID, baud=BAUD, required_fw_version=None):
    devices = []
    for candidate in list_ports_by_exact_vid_pid(vid, pid):
        info = probe_port(candidate["port"], baud)
        if info is None:
            continue
        if required_fw_version and info["fw_version"] != required_fw_version:
            continue
        devices.append({**candidate, **info})
    return devices


def prompt_yes_no(prompt: str, default: bool = False) -> bool:
    suffix = "[Y/n]" if default else "[y/N]"
    while True:
        answer = input(f"{prompt} {suffix}: ").strip().lower()
        if not answer:
            return default
        if answer in ("y", "yes"):
            return True
        if answer in ("n", "no"):
            return False
        print("Please answer y or n.")


def prompt_numeric_id(prompt: str) -> int:
    while True:
        answer = input(prompt).strip()
        try:
            value = int(answer)
        except ValueError:
            print("Please enter an integer dial ID.")
            continue

        if value <= 0 or value > 255:
            print("Dial ID must be between 1 and 255.")
            continue

        return value


class DeviceMonitor:
    def __init__(self, port: str, baud=BAUD):
        self.port = port
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(DRAIN_DELAY)
        self.ser.reset_input_buffer()
        self._buf = ""
        self._angles = []
        self.lock = threading.Lock()

    def reset_tracking(self):
        with self.lock:
            self._angles.clear()

    def read_telemetry(self):
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
            if not line.startswith("T,"):
                continue

            parts = line.split(",")
            if len(parts) < 4:
                continue

            try:
                angle = int(parts[3])
            except ValueError:
                continue

            with self.lock:
                self._angles.append(angle)

    def get_motion(self) -> int:
        with self.lock:
            if len(self._angles) < 2:
                return 0
            return max(self._angles) - min(self._angles)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def open_monitors(devices, baud=BAUD):
    return [DeviceMonitor(device["port"], baud=baud) for device in devices]


def detect_motion(monitors, duration=DEFAULT_MOTION_DETECT_WINDOW, threshold=DEFAULT_MOTION_THRESHOLD):
    for monitor in monitors:
        monitor.reset_tracking()

    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        for monitor in monitors:
            monitor.read_telemetry()
        time.sleep(0.005)

    best_monitor = None
    best_motion = 0
    for monitor in monitors:
        motion = monitor.get_motion()
        if motion > best_motion:
            best_motion = motion
            best_monitor = monitor

    if best_motion < threshold:
        return None, 0
    return best_monitor, best_motion


def close_monitors(monitors):
    for monitor in monitors:
        monitor.close()
