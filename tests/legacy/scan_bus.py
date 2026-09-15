#!/usr/bin/env python3
"""Bus diagnostic scanner for Feetech STS3215 servos.

Scans all 254 IDs at all standard baud rates to find every servo on the bus.
Also listens for unsolicited bus activity to detect bus-poisoning servos.

Usage:
    python scan_bus.py
    python scan_bus.py --port /dev/ttyACM0
    python scan_bus.py --baud 1000000       # scan only at this baud rate
    python scan_bus.py --quick              # only scan IDs 0-10 at 1Mbps
"""

from __future__ import annotations

import argparse
import sys
import time

import serial as pyserial
import scservo_sdk as scs

from _common import JOINT_NAMES, STANDARD_BAUDS, find_port


def _listen_for_activity(port: str, duration_s: float = 1.0) -> bytes:
    """Listen on the bus for unsolicited data (bus-poisoning detection)."""
    ser = pyserial.Serial(port, 1_000_000, timeout=duration_s)
    ser.reset_input_buffer()
    time.sleep(0.01)
    data = ser.read(500)
    ser.close()
    return data


def _dump_servo(pk: scs.PacketHandler, ph: scs.PortHandler, sid: int) -> None:
    """Print detailed register info for a found servo."""
    regs = [
        (56, 2, "Present_Position"),
        (63, 1, "Temperature"),
        (33, 1, "Mode"),
        (40, 1, "Torque_Enable"),
        (37, 1, "Acceleration"),
        (21, 1, "P_Coefficient"),
        (22, 1, "D_Coefficient"),
        (23, 1, "I_Coefficient"),
        (62, 1, "Voltage"),
        (58, 2, "Present_Load"),
        (56, 2, "Present_Speed"),
        (16, 2, "Max_Torque"),
        (48, 2, "Torque_Limit"),
        (5, 1, "ID"),
        (4, 1, "Baud_Rate_Reg"),
        (8, 1, "Response_Level"),
    ]

    for addr, size, name in regs:
        if size == 1:
            val, res, _ = pk.read1ByteTxRx(ph, sid, addr)
        else:
            val, res, _ = pk.read2ByteTxRx(ph, sid, addr)
        if res == 0:
            print(f"      {name:<20} (addr {addr:>2}): {val}")
        else:
            print(f"      {name:<20} (addr {addr:>2}): READ ERROR (res={res})")
        time.sleep(0.01)


def main() -> None:
    parser = argparse.ArgumentParser(description="Scan bus for Feetech servos")
    parser.add_argument("--port", type=str, default=None, help="Serial port (auto-detected if omitted)")
    parser.add_argument("--baud", type=int, default=None, help="Scan only this baud rate")
    parser.add_argument("--quick", action="store_true", help="Quick scan: IDs 0-10 at 1Mbps only")
    args = parser.parse_args()

    port = args.port or find_port()
    if port is None:
        print("ERROR: No SO-101 device found. Is the USB cable connected?")
        sys.exit(1)

    print(f"Port: {port}")

    # Bus activity check
    print()
    print("Listening for unsolicited bus activity (1s)...")
    data = _listen_for_activity(port)
    if data:
        print(f"  WARNING: Received {len(data)} unsolicited bytes!")
        print(f"  First 30: {[hex(b) for b in data[:30]]}")
        print("  A servo may be flooding the bus (bus-poisoning).")
        print("  Disconnect servos one at a time to identify the culprit.")
    else:
        print("  Bus is quiet.")

    # Scan
    bauds = [args.baud] if args.baud else ([1_000_000] if args.quick else STANDARD_BAUDS)
    max_id = 11 if args.quick else 254

    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    if not ph.openPort():
        print(f"ERROR: Cannot open port {port}")
        sys.exit(1)

    found: list[tuple[int, int]] = []

    for baud in bauds:
        ph.setBaudRate(baud)
        time.sleep(0.1)
        print(f"\nScanning baud={baud}, IDs 0-{max_id - 1}...")

        for sid in range(0, max_id):
            val, res, _ = pk.read2ByteTxRx(ph, sid, 56)
            if res == 0:
                joint = JOINT_NAMES.get(sid, "")
                label = f" ({joint})" if joint else ""
                print(f"  ID {sid}{label}: pos={val} FOUND")
                found.append((baud, sid))
            time.sleep(0.005)

    # Detailed dump for found servos
    if found:
        print(f"\n{'=' * 50}")
        print(f"Found {len(found)} servo(s). Detailed info:")
        print(f"{'=' * 50}")
        for baud, sid in found:
            ph.setBaudRate(baud)
            time.sleep(0.05)
            joint = JOINT_NAMES.get(sid, "unknown")
            print(f"\n  Servo ID {sid} ({joint}) @ baud {baud}:")
            _dump_servo(pk, ph, sid)
    else:
        print("\nNo servos found.")
        print("Check: 12V power on? Cable connected? Jumper in USB position?")

    ph.closePort()
    sys.exit(0 if found else 1)


if __name__ == "__main__":
    main()
