"""servo set-id -- assign an ID to a single servo and apply project defaults."""

from __future__ import annotations

import time

from .. import registers as reg
from ..bus import Bus
from ..cli import add_port_option, resolve_port
from ..config import (
    BAUD,
    EXPECTED_ACCELERATION,
    EXPECTED_D_COEFF,
    EXPECTED_MODE,
    EXPECTED_P_COEFF,
    JOINT_NAMES,
    STANDARD_BAUDS,
)

MODE_NAMES = {0: "position", 1: "velocity"}


def add_parser(sub) -> None:
    p = sub.add_parser("set-id", help="assign an ID to a fresh servo, with defaults")
    p.add_argument("new_id", type=int, help="new servo ID (1-253)")
    add_port_option(p)
    p.add_argument("--baud", type=int, default=None, help="baud rate (auto-detected if omitted)")
    p.add_argument("--current-id", type=int, default=None,
                   help="the servo's present ID; required with --baud")
    p.set_defaults(run=run)


def find_servo(bus: Bus, bauds=STANDARD_BAUDS) -> tuple[int, int] | None:
    """Sweep the given baud rates and every ID for one responding servo."""
    for baud in bauds:
        bus.baudrate = baud
        time.sleep(0.1)
        for sid in range(0, 254):
            if bus.read(sid, reg.PRESENT_POSITION, 2) is not None:
                return baud, sid
            time.sleep(0.005)
    return None


def show(bus: Bus, sid: int, baud: int) -> None:
    pos = bus.read(sid, reg.PRESENT_POSITION, 2)
    temp = bus.read(sid, reg.TEMPERATURE, 1)
    mode = bus.read(sid, reg.MODE, 1)
    torque = bus.read(sid, reg.TORQUE_ENABLE, 1)
    accel = bus.read(sid, reg.ACCELERATION, 1)
    p_coeff = bus.read(sid, reg.P_COEFF, 1)
    d_coeff = bus.read(sid, reg.D_COEFF, 1)

    print(f"  Current ID:     {sid} ({JOINT_NAMES.get(sid, 'unknown')})")
    print(f"  Baud rate:      {baud}")
    print(f"  Position:       {pos}")
    print(f"  Temperature:    {temp}C")
    print(f"  Mode:           {mode} ({MODE_NAMES.get(mode, f'unknown({mode})')})")
    print(f"  Torque:         {'ON' if torque else 'OFF'}")
    print(f"  Acceleration:   {accel}")
    print(f"  PID P/D:        {p_coeff}/{d_coeff}")


def write_and_verify(bus: Bus, sid: int, addr: int, val: int, name: str) -> bool:
    bus.write_byte(sid, addr, val, settle=0.1)
    readback = bus.read(sid, addr, 1)
    if readback != val:
        print(f"  FAILED: {name} (addr {addr}) = {readback}, expected {val}")
        return False
    return True


def run(args) -> int:
    if not 1 <= args.new_id <= 253:
        print("ERROR: ID must be 1-253")
        return 1

    port = resolve_port(args)
    bus = Bus.open(port, args.baud or BAUD)
    print(f"Port: {port}")

    if args.baud and args.current_id is not None:
        bus.baudrate = args.baud
        time.sleep(0.1)
        found = bus.read(args.current_id, reg.PRESENT_POSITION, 2) is not None
        result = (args.baud, args.current_id) if found else None
    elif args.baud:
        print(f"Scanning for servo at {args.baud} baud...")
        result = find_servo(bus, [args.baud])
    else:
        print("Scanning for servo...")
        result = find_servo(bus)

    if result is None:
        print("ERROR: No servo found at any baud rate / ID.")
        print("Check: cable connected? 12V power on? servo plugged into bus port?")
        bus.close()
        return 1

    baud, current_id = result
    bus.baudrate = baud

    print()
    print("Found servo:")
    show(bus, current_id, baud)

    new_id = args.new_id
    joint_name = JOINT_NAMES.get(new_id, "unknown")
    print()
    print(f"Will set ID to {new_id} ({joint_name}) with project defaults.")

    bus.write_byte(current_id, reg.TORQUE_ENABLE, 0, settle=0.1)
    bus.write_byte(current_id, reg.LOCK, 0, settle=0.1)
    bus.write_byte(current_id, reg.ID, new_id, settle=0.1)

    ok = True
    ok &= write_and_verify(bus, new_id, reg.ACCELERATION, EXPECTED_ACCELERATION, "Acceleration")
    ok &= write_and_verify(bus, new_id, reg.P_COEFF, EXPECTED_P_COEFF, "P_Coefficient")
    ok &= write_and_verify(bus, new_id, reg.D_COEFF, EXPECTED_D_COEFF, "D_Coefficient")
    ok &= write_and_verify(bus, new_id, reg.MODE, EXPECTED_MODE, "Mode")

    bus.write_byte(new_id, reg.LOCK, 1, settle=0.1)  # commits the new ID to EEPROM

    if bus.read(new_id, reg.PRESENT_POSITION, 2) is None:
        print(f"\nERROR: Cannot read servo at new ID {new_id}")
        bus.close()
        return 1

    print()
    print(f"Servo configured as ID {new_id} ({joint_name}):")
    show(bus, new_id, baud)
    print()
    print("All writes verified OK." if ok else "WARNING: Some writes failed. Check values above.")

    bus.close()
    return 0 if ok else 1
