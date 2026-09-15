"""servo wiggle -- move one servo, or each in turn, so you can see which is which."""

from __future__ import annotations

import time

from .. import registers as reg
from ..bus import Bus
from ..cli import add_port_option, open_bus

AMP = 500  # ~44 degrees each way
REPS = 4
DWELL = 0.5


def add_parser(sub) -> None:
    p = sub.add_parser("wiggle", help="wiggle a servo so you can spot it physically")
    p.add_argument("ids", nargs="*", type=int, help="servo IDs; with several, one at a time")
    add_port_option(p)
    p.set_defaults(run=run)


def wiggle_one(bus: Bus, sid: int) -> bool:
    pos = bus.read(sid, reg.PRESENT_POSITION, 2)
    if pos is None:
        print(f"ID {sid} not responding.")
        return False

    base = min(3300, max(800, pos))  # keep both swings inside the travel limits
    print(f"Wiggling ID {sid} (start {pos}) — watch which servo moves.")
    bus.write_byte(sid, reg.TORQUE_ENABLE, 1, settle=0.05)
    for _ in range(REPS):
        bus.write_word(sid, reg.GOAL_POSITION, base + AMP)
        time.sleep(DWELL)
        bus.write_word(sid, reg.GOAL_POSITION, base - AMP)
        time.sleep(DWELL)
    bus.write_word(sid, reg.GOAL_POSITION, pos)
    time.sleep(0.4)
    bus.write_byte(sid, reg.TORQUE_ENABLE, 0, settle=0.0)
    print(f"Done ID {sid}.")
    return True


def run(args) -> int:
    if not args.ids:
        print("usage: servo wiggle <ID> [ID ...]")
        return 1

    bus, _ = open_bus(args)
    ok = True
    try:
        for sid in args.ids:
            ok &= wiggle_one(bus, sid)
            if len(args.ids) > 1:
                time.sleep(0.7)
    finally:
        bus.close()
    return 0 if ok else 1
