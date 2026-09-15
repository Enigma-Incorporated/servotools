#!/usr/bin/env python3
"""Health check for an SO-101 arm. Verifies all 6 servos are communicating
and reports temperature, mode, torque, load, acceleration, and PID values.

Usage:
    python health_check.py
    python health_check.py --port /dev/ttyACM0
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

TEMP_WARNING_C = 45
TEMP_CRITICAL_C = 60


def run_health_check(port: str) -> bool:
    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)

    if not ph.openPort():
        print(f"ERROR: Cannot open port {port}")
        return False

    ph.setBaudRate(1_000_000)

    print(f"Port: {port}")
    print()
    print("=" * 72)
    print(f"{'ID':<4} {'Joint':<16} {'Pos':>5} {'Temp':>5} {'Mode':>5} {'Torq':>5} {'Load':>5}  Status")
    print("-" * 72)

    all_ok = True
    servo_data: list[dict] = []

    for sid in range(1, 7):
        model, res, _ = pk.ping(ph, sid)
        if res != 0:
            print(f"{sid:<4} {JOINT_NAMES.get(sid, '?'):<16} {'—':>5} {'—':>5} {'—':>5} {'—':>5} {'—':>5}  MISSING")
            all_ok = False
            continue

        pos, r1, _ = pk.read2ByteTxRx(ph, sid, 56)
        temp, r2, _ = pk.read1ByteTxRx(ph, sid, 63)
        mode, r3, _ = pk.read1ByteTxRx(ph, sid, 33)
        torque, r4, _ = pk.read1ByteTxRx(ph, sid, 40)
        load_raw, r5, _ = pk.read2ByteTxRx(ph, sid, 58)
        voltage, r6, _ = pk.read1ByteTxRx(ph, sid, 62)
        accel, r7, _ = pk.read1ByteTxRx(ph, sid, 37)
        p_coeff, r8, _ = pk.read1ByteTxRx(ph, sid, 21)
        d_coeff, r9, _ = pk.read1ByteTxRx(ph, sid, 22)

        read_ok = all(r == 0 for r in [r1, r2, r3, r4, r5, r6, r7, r8, r9])
        if not read_ok:
            print(f"{sid:<4} {JOINT_NAMES.get(sid, '?'):<16} {'—':>5} {'—':>5} {'—':>5} {'—':>5} {'—':>5}  COMM ERROR")
            all_ok = False
            continue

        load_pct = (load_raw & 0x3FF) * 100.0 / 1023.0

        issues: list[str] = []
        if temp >= TEMP_CRITICAL_C:
            issues.append(f"CRITICAL TEMP ({temp}C) - DISCONNECT IMMEDIATELY")
        elif temp >= TEMP_WARNING_C:
            issues.append(f"WARM ({temp}C)")

        if torque != 0:
            issues.append("TORQUE ON")
        if mode != 0:
            issues.append(f"MODE={mode}")
        if accel != EXPECTED_ACCELERATION:
            issues.append(f"ACCEL={accel} (want {EXPECTED_ACCELERATION})")
        if p_coeff != EXPECTED_P_COEFF:
            issues.append(f"P={p_coeff} (want {EXPECTED_P_COEFF})")
        if d_coeff != EXPECTED_D_COEFF:
            issues.append(f"D={d_coeff} (want {EXPECTED_D_COEFF})")

        status = "OK" if not issues else " | ".join(issues)
        if issues:
            all_ok = False

        print(f"{sid:<4} {JOINT_NAMES.get(sid, '?'):<16} {pos:>5} {temp:>4}C {mode:>5} {torque:>5} {load_pct:>4.0f}%  {status}")

        servo_data.append({"id": sid, "temp": temp, "mode": mode, "torque": torque})
        time.sleep(0.02)

    print("-" * 72)

    if servo_data:
        temps = [s["temp"] for s in servo_data]
        print(f"Temperature range: {min(temps)}C - {max(temps)}C")

        torque_on = [s for s in servo_data if s["torque"] != 0]
        if torque_on:
            print(f"WARNING: {len(torque_on)} servo(s) have torque enabled!")

        wrong_mode = [s for s in servo_data if s["mode"] != 0]
        if wrong_mode:
            ids = [str(s["id"]) for s in wrong_mode]
            print(f"WARNING: Servo(s) {', '.join(ids)} not in position mode!")

    print()
    if all_ok:
        print("RESULT: All 6 servos healthy.")
    else:
        print("RESULT: Issues detected. See above.")

    ph.closePort()
    return all_ok


def main() -> None:
    parser = argparse.ArgumentParser(description="SO-101 arm health check")
    parser.add_argument("--port", type=str, default=None, help="Serial port (auto-detected if omitted)")
    args = parser.parse_args()

    port = args.port or find_port()
    if port is None:
        print("ERROR: No SO-101 device found. Is the USB cable connected?")
        sys.exit(1)

    ok = run_health_check(port)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
