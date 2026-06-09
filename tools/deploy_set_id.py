import argparse

from common import (
    BAUD,
    DEFAULT_FIRMWARE_VERSION,
    DEFAULT_MOTION_DETECT_WINDOW,
    DEFAULT_MOTION_THRESHOLD,
    assign_identity,
    close_monitors,
    detect_motion,
    discover_target_devices,
    open_monitors,
    prompt_numeric_id,
    prompt_yes_no,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Interactively move each connected controller, then assign a user-entered dial ID to the detected device."
    )
    parser.add_argument("--baud", type=int, default=BAUD, help="Serial baud rate")
    parser.add_argument("--vid", type=lambda value: int(value, 0), default=0x1A86, help="USB VID to target")
    parser.add_argument("--pid", type=lambda value: int(value, 0), default=0x7523, help="USB PID to target")
    parser.add_argument("--firmware-version", default=DEFAULT_FIRMWARE_VERSION, help="Required firmware version for assignment")
    parser.add_argument("--motion-window", type=float, default=DEFAULT_MOTION_DETECT_WINDOW, help="Seconds to watch for dial movement after each prompt")
    parser.add_argument("--motion-threshold", type=int, default=DEFAULT_MOTION_THRESHOLD, help="Minimum decidegree range that counts as intentional movement")
    args = parser.parse_args()

    devices = discover_target_devices(
        vid=args.vid,
        pid=args.pid,
        baud=args.baud,
        required_fw_version=args.firmware_version,
    )
    if not devices:
        print("No matching controllers found.")
        return

    print("Controllers available for ID assignment:")
    for device in devices:
        print(f"  {device['port']}: fw={device['fw_version']} dial_id={device['dial_id']}")

    monitors = open_monitors(devices, baud=args.baud)
    assignments = {}
    assigned_ids = set()

    try:
        while len(assignments) < len(monitors):
            print("-" * 60)
            input("Move exactly one unassigned dial now, then press Enter to detect it... ")
            monitor, motion = detect_motion(monitors, duration=args.motion_window, threshold=args.motion_threshold)

            if monitor is None:
                print(f"No motion detected above {args.motion_threshold} decideg. Try again.")
                continue

            if monitor.port in assignments:
                print(f"{monitor.port} is already assigned to dial ID {assignments[monitor.port]}. Move a different dial.")
                continue

            print(f"Detected {monitor.port} (moved {motion} decideg = {motion / 10:.1f} deg)")
            while True:
                dial_id = prompt_numeric_id(f"Enter the numeric dial ID to write to {monitor.port}: ")
                if dial_id in assigned_ids:
                    print(f"Dial ID {dial_id} is already queued for another controller. Choose a different ID.")
                    continue
                assignments[monitor.port] = dial_id
                assigned_ids.add(dial_id)
                break

            if len(assignments) < len(monitors) and not prompt_yes_no("Continue assigning the remaining controllers?", default=True):
                break

        if not assignments:
            print("No assignments queued.")
            return

        print("\nProposed assignments:")
        for port, dial_id in assignments.items():
            print(f"  {port}: dial_id = {dial_id}")

        if not prompt_yes_no("Write these dial IDs to controller flash?", default=False):
            print("Assignment cancelled.")
            return
    finally:
        close_monitors(monitors)

    for port, dial_id in assignments.items():
        confirmed = assign_identity(port, dial_id, baud=args.baud)
        print(f"  {port}: wrote {dial_id}, confirmed = {confirmed}")

    print("\nDone.")


if __name__ == "__main__":
    main()