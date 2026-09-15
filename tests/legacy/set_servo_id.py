#!/usr/bin/env python3
"""Assign an ID to a Feetech STS3215 servo and configure project defaults.

Scans all baud rates and IDs to find the connected servo, displays its
current state for verification, then writes the new ID along with
project-standard acceleration and PID values.

Usage:
    python set_servo_id.py 2
    python set_servo_id.py 5 --port /dev/ttyACM0
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
    STANDARD_BAUDS,
    find_port,
)

ADDR_ID = 5
ADDR_P_COEFF = 21
ADDR_D_COEFF = 22
ADDR_MODE = 33
ADDR_ACCEL_EEPROM = 37
ADDR_TORQUE_ENABLE = 40
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_TEMP = 63


def _find_servo(
    ph: scs.PortHandler, pk: scs.PacketHandler
) -> tuple[int, int] | None:
    """Scan baud rates and IDs to find a single connected servo.

    Returns (baud_rate, servo_id) or None.
    """
    for baud in STANDARD_BAUDS:
        ph.setBaudRate(baud)
        time.sleep(0.1)
        for sid in range(0, 254):
            val, res, _ = pk.read2ByteTxRx(ph, sid, ADDR_PRESENT_POSITION)
            if res == 0:
                return baud, sid
            time.sleep(0.005)
    return None


def _display_servo_info(
    pk: scs.PacketHandler, ph: scs.PortHandler, sid: int, baud: int
) -> None:
    pos, _, _ = pk.read2ByteTxRx(ph, sid, ADDR_PRESENT_POSITION)
    temp, _, _ = pk.read1ByteTxRx(ph, sid, ADDR_PRESENT_TEMP)
    mode, _, _ = pk.read1ByteTxRx(ph, sid, ADDR_MODE)
    torque, _, _ = pk.read1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE)
    accel, _, _ = pk.read1ByteTxRx(ph, sid, ADDR_ACCEL_EEPROM)
    p_coeff, _, _ = pk.read1ByteTxRx(ph, sid, ADDR_P_COEFF)
    d_coeff, _, _ = pk.read1ByteTxRx(ph, sid, ADDR_D_COEFF)

    joint = JOINT_NAMES.get(sid, "unknown")
    print(f"  Current ID:     {sid} ({joint})")
    print(f"  Baud rate:      {baud}")
    print(f"  Position:       {pos}")
    print(f"  Temperature:    {temp}C")
    print(f"  Mode:           {mode} ({'position' if mode == 0 else 'velocity' if mode == 1 else f'unknown({mode})'})")
    print(f"  Torque:         {'ON' if torque else 'OFF'}")
    print(f"  Acceleration:   {accel}")
    print(f"  PID P/D:        {p_coeff}/{d_coeff}")


def _write_and_verify_1(
    pk: scs.PacketHandler, ph: scs.PortHandler, sid: int, addr: int, val: int, name: str
) -> bool:
    pk.write1ByteTxRx(ph, sid, addr, val)
    time.sleep(0.1)
    readback, res, _ = pk.read1ByteTxRx(ph, sid, addr)
    if res != 0 or readback != val:
        print(f"  FAILED: {name} (addr {addr}) = {readback}, expected {val}")
        return False
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="Set STS3215 servo ID and configure defaults")
    parser.add_argument("new_id", type=int, help="New servo ID (1-253)")
    parser.add_argument("--port", type=str, default=None, help="Serial port (auto-detected if omitted)")
    parser.add_argument("--baud", type=int, default=None, help="Baud rate (auto-detected if omitted)")
    args = parser.parse_args()

    if not 1 <= args.new_id <= 253:
        print("ERROR: ID must be 1-253")
        sys.exit(1)

    port = args.port or find_port()
    if port is None:
        print("ERROR: No Waveshare adapter found. Is the USB cable connected?")
        sys.exit(1)

    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    if not ph.openPort():
        print(f"ERROR: Cannot open port {port}")
        sys.exit(1)

    print(f"Port: {port}")

    if args.baud:
        ph.setBaudRate(args.baud)
        time.sleep(0.1)
        val, res, _ = pk.read2ByteTxRx(ph, args.new_id if args.new_id else 1, ADDR_PRESENT_POSITION)
        if res == 0:
            result = (args.baud, args.new_id)
        else:
            result = None
    else:
        print("Scanning for servo...")
        result = _find_servo(ph, pk)

    if result is None:
        print("ERROR: No servo found at any baud rate / ID.")
        print("Check: cable connected? 12V power on? servo plugged into bus port?")
        ph.closePort()
        sys.exit(1)

    baud, current_id = result
    ph.setBaudRate(baud)

    print()
    print("Found servo:")
    _display_servo_info(pk, ph, current_id, baud)

    new_id = args.new_id
    joint_name = JOINT_NAMES.get(new_id, "unknown")
    print()
    print(f"Will set ID to {new_id} ({joint_name}) with project defaults.")

    # Torque off
    pk.write1ByteTxRx(ph, current_id, ADDR_TORQUE_ENABLE, 0)
    time.sleep(0.1)

    # Unlock EEPROM
    pk.write1ByteTxRx(ph, current_id, ADDR_LOCK, 0)
    time.sleep(0.1)

    # Write new ID
    pk.write1ByteTxRx(ph, current_id, ADDR_ID, new_id)
    time.sleep(0.1)

    # Configure defaults (on new ID now)
    ok = True
    ok &= _write_and_verify_1(pk, ph, new_id, ADDR_ACCEL_EEPROM, EXPECTED_ACCELERATION, "Acceleration")
    ok &= _write_and_verify_1(pk, ph, new_id, ADDR_P_COEFF, EXPECTED_P_COEFF, "P_Coefficient")
    ok &= _write_and_verify_1(pk, ph, new_id, ADDR_D_COEFF, EXPECTED_D_COEFF, "D_Coefficient")
    ok &= _write_and_verify_1(pk, ph, new_id, ADDR_MODE, 0, "Mode")

    # Lock EEPROM
    pk.write1ByteTxRx(ph, new_id, ADDR_LOCK, 1)
    time.sleep(0.1)

    # Final verification
    pos, res, _ = pk.read2ByteTxRx(ph, new_id, ADDR_PRESENT_POSITION)
    if res != 0:
        print(f"\nERROR: Cannot read servo at new ID {new_id}")
        ph.closePort()
        sys.exit(1)

    print()
    print(f"Servo configured as ID {new_id} ({joint_name}):")
    _display_servo_info(pk, ph, new_id, baud)
    print()

    if ok:
        print("All writes verified OK.")
    else:
        print("WARNING: Some writes failed. Check values above.")

    ph.closePort()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
