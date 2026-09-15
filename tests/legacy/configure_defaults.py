#!/usr/bin/env python3
"""Write project-standard defaults to all connected SO-101 servos.

Sets acceleration, PID coefficients, and mode on all reachable servos
(IDs 1-6). Verifies each write with readback.

Usage:
    python configure_defaults.py
    python configure_defaults.py --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import sys
import time

import scservo_sdk as scs

from _common import (
    EXPECTED_ACCELERATION,
    EXPECTED_D_COEFF,
    EXPECTED_P_COEFF,
    JOINT_NAMES,
    find_port,
)

ADDR_P_COEFF = 21
ADDR_D_COEFF = 22
ADDR_MODE = 33
ADDR_ACCEL_EEPROM = 37
ADDR_TORQUE_ENABLE = 40
ADDR_LOCK = 55

DEFAULTS = [
    (ADDR_ACCEL_EEPROM, EXPECTED_ACCELERATION, "Acceleration"),
    (ADDR_P_COEFF, EXPECTED_P_COEFF, "P_Coefficient"),
    (ADDR_D_COEFF, EXPECTED_D_COEFF, "D_Coefficient"),
    (ADDR_MODE, 0, "Mode"),
]


def main() -> None:
    parser = argparse.ArgumentParser(description="Configure project defaults on SO-101 servos")
    parser.add_argument("--port", type=str, default=None, help="Serial port (auto-detected if omitted)")
    args = parser.parse_args()

    port = args.port or find_port()
    if port is None:
        print("ERROR: No SO-101 device found. Is the USB cable connected?")
        sys.exit(1)

    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    if not ph.openPort():
        print(f"ERROR: Cannot open port {port}")
        sys.exit(1)
    ph.setBaudRate(1_000_000)

    print(f"Port: {port}")
    print(f"Defaults: Accel={EXPECTED_ACCELERATION}, P={EXPECTED_P_COEFF}, D={EXPECTED_D_COEFF}, Mode=0")
    print()

    all_ok = True

    for sid in range(1, 7):
        model, res, _ = pk.ping(ph, sid)
        if res != 0:
            print(f"  Servo {sid} ({JOINT_NAMES.get(sid, '?')}): not found, skipping")
            continue

        joint = JOINT_NAMES.get(sid, "?")

        # Torque off (required for EEPROM writes)
        pk.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 0)
        time.sleep(0.1)

        # Unlock EEPROM
        pk.write1ByteTxRx(ph, sid, ADDR_LOCK, 0)
        time.sleep(0.1)

        servo_ok = True
        for addr, val, name in DEFAULTS:
            pk.write1ByteTxRx(ph, sid, addr, val)
            time.sleep(0.1)
            readback, r, _ = pk.read1ByteTxRx(ph, sid, addr)
            if r != 0 or readback != val:
                print(f"  Servo {sid} ({joint}): FAILED {name} (wrote {val}, got {readback})")
                servo_ok = False

        # Lock EEPROM
        pk.write1ByteTxRx(ph, sid, ADDR_LOCK, 1)
        time.sleep(0.05)

        if servo_ok:
            print(f"  Servo {sid} ({joint}): OK")
        else:
            all_ok = False

    ph.closePort()

    print()
    if all_ok:
        print("All reachable servos configured.")
    else:
        print("WARNING: Some writes failed. See above.")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
