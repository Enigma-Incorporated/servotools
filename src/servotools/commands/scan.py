"""servo scan -- find servos at any baud rate and dump their registers."""

from __future__ import annotations

import time

import serial as pyserial

from .. import registers as reg
from ..bus import Bus
from ..cli import add_port_option, resolve_port
from ..config import BAUD, JOINT_NAMES, STANDARD_BAUDS

# Kept as-is from scan_bus.py, including the duplicated address 56. See registers.DUMP.
LEGACY_DUMP = [
    (56, 2, "Present_Position"), (63, 1, "Temperature"), (33, 1, "Mode"),
    (40, 1, "Torque_Enable"), (37, 1, "Acceleration"), (21, 1, "P_Coefficient"),
    (22, 1, "D_Coefficient"), (23, 1, "I_Coefficient"), (62, 1, "Voltage"),
    (58, 2, "Present_Load"), (56, 2, "Present_Speed"), (16, 2, "Max_Torque"),
    (48, 2, "Torque_Limit"), (5, 1, "ID"), (4, 1, "Baud_Rate_Reg"),
    (8, 1, "Response_Level"),
]


def add_parser(sub) -> None:
    p = sub.add_parser("scan", help="find servos at any baud rate, detect bus poison")
    add_port_option(p)
    p.add_argument("--baud", type=int, default=None, help="scan only this baud rate")
    p.add_argument("--quick", action="store_true", help="only IDs 0-10 at 1 Mbps")
    p.set_defaults(run=run)


def listen_for_activity(port: str, duration_s: float = 1.0) -> bytes:
    """Unsolicited traffic means a servo is talking over everyone else."""
    ser = pyserial.Serial(port, BAUD, timeout=duration_s)
    ser.reset_input_buffer()
    time.sleep(0.01)
    data = ser.read(500)
    ser.close()
    return data


def dump_servo(bus: Bus, sid: int) -> None:
    for addr, size, name in LEGACY_DUMP:
        val = bus.read(sid, addr, size)
        if val is not None:
            print(f"      {name:<20} (addr {addr:>2}): {val}")
        else:
            print(f"      {name:<20} (addr {addr:>2}): READ ERROR")
        time.sleep(0.01)


def run(args) -> int:
    port = resolve_port(args)

    print(f"Port: {port}")
    print()
    print("Listening for unsolicited bus activity (1s)...")
    data = listen_for_activity(port)
    if data:
        print(f"  WARNING: Received {len(data)} unsolicited bytes!")
        print(f"  First 30: {[hex(b) for b in data[:30]]}")
        print("  A servo may be flooding the bus (bus-poisoning).")
        print("  Disconnect servos one at a time to identify the culprit.")
    else:
        print("  Bus is quiet.")

    bauds = [args.baud] if args.baud else ([BAUD] if args.quick else STANDARD_BAUDS)
    max_id = 11 if args.quick else 254

    bus = Bus.open(port, bauds[0])
    found: list[tuple[int, int]] = []

    for baud in bauds:
        bus.baudrate = baud
        time.sleep(0.1)
        print(f"\nScanning baud={baud}, IDs 0-{max_id - 1}...")

        for sid in range(0, max_id):
            val = bus.read(sid, reg.PRESENT_POSITION, 2)
            if val is not None:
                joint = JOINT_NAMES.get(sid, "")
                label = f" ({joint})" if joint else ""
                print(f"  ID {sid}{label}: pos={val} FOUND")
                found.append((baud, sid))
            time.sleep(0.005)

    if found:
        print(f"\n{'=' * 50}")
        print(f"Found {len(found)} servo(s). Detailed info:")
        print(f"{'=' * 50}")
        for baud, sid in found:
            bus.baudrate = baud
            time.sleep(0.05)
            joint = JOINT_NAMES.get(sid, "unknown")
            print(f"\n  Servo ID {sid} ({joint}) @ baud {baud}:")
            dump_servo(bus, sid)
    else:
        print("\nNo servos found.")
        print("Check: 12V power on? Cable connected? Jumper in USB position?")

    bus.close()
    return 0 if found else 1
