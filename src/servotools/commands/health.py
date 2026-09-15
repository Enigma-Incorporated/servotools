"""servo health -- check every joint's comms, temperature, mode and PID."""

from __future__ import annotations

import time

from .. import registers as reg
from ..bus import Bus
from ..cli import add_port_option, open_bus
from ..config import (
    ARM_IDS,
    EXPECTED_ACCELERATION,
    EXPECTED_D_COEFF,
    EXPECTED_P_COEFF,
    JOINT_NAMES,
    TEMP_CRITICAL_C,
    TEMP_WARNING_C,
)

HEADER = (f"{'ID':<4} {'Joint':<16} {'Pos':>5} {'Temp':>5} {'Mode':>5} "
          f"{'Torq':>5} {'Load':>5}  Status")
BLANK = f"{'—':>5} {'—':>5} {'—':>5} {'—':>5} {'—':>5}"


def add_parser(sub) -> None:
    p = sub.add_parser("health", help="check all six servos")
    add_port_option(p)
    p.set_defaults(run=run)


def run(args) -> int:
    bus, port = open_bus(args)
    try:
        return 0 if report(bus, port) else 1
    finally:
        bus.close()


def report(bus: Bus, port: str) -> bool:
    print(f"Port: {port}")
    print()
    print("=" * 72)
    print(HEADER)
    print("-" * 72)

    all_ok = True
    servo_data: list[dict] = []

    for sid in ARM_IDS:
        joint = JOINT_NAMES.get(sid, "?")
        if not bus.ping(sid):
            print(f"{sid:<4} {joint:<16} {BLANK}  MISSING")
            all_ok = False
            continue

        pos = bus.read(sid, reg.PRESENT_POSITION, 2)
        temp = bus.read(sid, reg.TEMPERATURE, 1)
        mode = bus.read(sid, reg.MODE, 1)
        torque = bus.read(sid, reg.TORQUE_ENABLE, 1)
        load_raw = bus.read(sid, reg.PRESENT_SPEED, 2)
        voltage = bus.read(sid, reg.VOLTAGE, 1)
        accel = bus.read(sid, reg.ACCELERATION, 1)
        p_coeff = bus.read(sid, reg.P_COEFF, 1)
        d_coeff = bus.read(sid, reg.D_COEFF, 1)

        values = [pos, temp, mode, torque, load_raw, voltage, accel, p_coeff, d_coeff]
        if any(v is None for v in values):
            print(f"{sid:<4} {joint:<16} {BLANK}  COMM ERROR")
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

        print(f"{sid:<4} {joint:<16} {pos:>5} {temp:>4}C {mode:>5} "
              f"{torque:>5} {load_pct:>4.0f}%  {status}")

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
    print("RESULT: All 6 servos healthy." if all_ok else "RESULT: Issues detected. See above.")
    return all_ok
