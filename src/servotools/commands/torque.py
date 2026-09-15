"""servo torque -- emergency broadcast torque off (or on)."""

from __future__ import annotations

import time

from .. import registers as reg
from ..bus import Bus
from ..cli import add_port_option, resolve_port
from ..config import ARM_IDS, BAUD, BROADCAST_ID, PORT_TIMEOUT_FIRE_AND_FORGET


def add_parser(sub) -> None:
    p = sub.add_parser("torque", help="disable (or enable) torque on every servo")
    p.add_argument("state", nargs="?", choices=["off", "on"], default="off",
                   help="default: off")
    add_port_option(p)
    p.set_defaults(run=run)


def run(args) -> int:
    port = resolve_port(args)
    val = 1 if args.state == "on" else 0
    action = "ON" if val else "OFF"

    # Fire-and-forget: no reply is awaited, so this still works on a corrupted bus.
    bus = Bus.open(port, BAUD, PORT_TIMEOUT_FIRE_AND_FORGET)
    time.sleep(0.05)
    for sid in (BROADCAST_ID, *ARM_IDS):
        bus.write_byte(sid, reg.TORQUE_ENABLE, val, settle=0.01)
    bus.close()

    print(f"Torque {action} sent to broadcast + IDs 1-6 on {port}")
    return 0
