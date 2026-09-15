"""servo defaults -- write the project-standard parameters to every servo."""

from __future__ import annotations

from .. import registers as reg
from ..cli import add_port_option, open_bus
from ..config import (
    ARM_IDS,
    EXPECTED_ACCELERATION,
    EXPECTED_D_COEFF,
    EXPECTED_MODE,
    EXPECTED_P_COEFF,
    JOINT_NAMES,
)

VALUES = [
    (reg.ACCELERATION, EXPECTED_ACCELERATION, "Acceleration"),
    (reg.P_COEFF, EXPECTED_P_COEFF, "P_Coefficient"),
    (reg.D_COEFF, EXPECTED_D_COEFF, "D_Coefficient"),
    (reg.MODE, EXPECTED_MODE, "Mode"),
]


def add_parser(sub) -> None:
    p = sub.add_parser("defaults", help="write project-standard params to all servos")
    add_port_option(p)
    p.set_defaults(run=run)


def run(args) -> int:
    bus, port = open_bus(args)

    print(f"Port: {port}")
    print(f"Defaults: Accel={EXPECTED_ACCELERATION}, P={EXPECTED_P_COEFF}, "
          f"D={EXPECTED_D_COEFF}, Mode={EXPECTED_MODE}")
    print()

    all_ok = True
    for sid in ARM_IDS:
        joint = JOINT_NAMES.get(sid, "?")
        if not bus.ping(sid):
            print(f"  Servo {sid} ({joint}): not found, skipping")
            continue

        bus.write_byte(sid, reg.TORQUE_ENABLE, 0, settle=0.1)  # EEPROM writes need torque off
        bus.write_byte(sid, reg.LOCK, 0, settle=0.1)

        servo_ok = True
        for addr, val, name in VALUES:
            bus.write_byte(sid, addr, val, settle=0.1)
            readback = bus.read(sid, addr, 1)
            if readback != val:
                print(f"  Servo {sid} ({joint}): FAILED {name} (wrote {val}, got {readback})")
                servo_ok = False

        bus.write_byte(sid, reg.LOCK, 1, settle=0.05)

        if servo_ok:
            print(f"  Servo {sid} ({joint}): OK")
        else:
            all_ok = False

    bus.close()

    print()
    print("All reachable servos configured." if all_ok
          else "WARNING: Some writes failed. See above.")
    return 0 if all_ok else 1
