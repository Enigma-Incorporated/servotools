#!/usr/bin/env python3
"""Emergency torque disable (or enable) for all SO-101 servos.

Sends broadcast torque-off to ID 254 plus individual commands to IDs 1-6.
Uses fire-and-forget writes — no response required, works even on a
partially corrupted bus.

Usage:
    python torque_off.py              # disable torque on all servos
    python torque_off.py --on         # enable torque on all servos
    python torque_off.py --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import sys
import time

import serial as pyserial

from _common import find_port

ADDR_TORQUE_ENABLE = 40


def _make_write1(sid: int, addr: int, val: int) -> bytes:
    """Build a raw Feetech WRITE instruction packet (1-byte value)."""
    ln = 4
    inst = 0x03
    cs = (~(sid + ln + inst + addr + val)) & 0xFF
    return bytes([0xFF, 0xFF, sid, ln, inst, addr, val, cs])


def main() -> None:
    parser = argparse.ArgumentParser(description="Emergency torque control for SO-101")
    parser.add_argument("--on", action="store_true", help="Enable torque instead of disabling")
    parser.add_argument("--port", type=str, default=None, help="Serial port (auto-detected if omitted)")
    args = parser.parse_args()

    port = args.port or find_port()
    if port is None:
        print("ERROR: No SO-101 device found. Is the USB cable connected?")
        sys.exit(1)

    val = 1 if args.on else 0
    action = "ON" if args.on else "OFF"

    ser = pyserial.Serial(port, 1_000_000, timeout=0.1)
    time.sleep(0.1)

    # Broadcast first (ID 254), then individual IDs for reliability
    targets = [254] + list(range(1, 7))
    for sid in targets:
        ser.write(_make_write1(sid, ADDR_TORQUE_ENABLE, val))
        time.sleep(0.01)

    ser.close()
    print(f"Torque {action} sent to broadcast + IDs 1-6 on {port}")


if __name__ == "__main__":
    main()
