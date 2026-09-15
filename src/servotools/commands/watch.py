"""servo watch -- log connectivity changes while the arm is posed by hand."""

from __future__ import annotations

import time
from datetime import datetime

from .. import registers as reg
from ..bus import Bus
from ..cli import add_port_option, open_bus
from ..config import JOINT_NAMES


def add_parser(sub) -> None:
    p = sub.add_parser("watch", help="log servo dropouts while you move the arm")
    add_port_option(p)
    p.add_argument("--joints", type=int, choices=range(1, 7), default=6,
                   help="number of installed joints, starting at ID 1 (default: 6)")
    p.add_argument("--interval", type=float, default=0.05,
                   help="seconds between full-chain polls (default: 0.05)")
    p.set_defaults(run=run)


def poll_chain(bus: Bus, ids: tuple[int, ...], positions: dict[int, int]) -> dict[int, bool]:
    """Poll every servo once, keeping the last known position when a read fails."""
    states = {}
    for sid in ids:
        position = bus.read(sid, reg.PRESENT_POSITION, 2)
        states[sid] = position is not None
        if states[sid]:
            positions[sid] = position
    return states


def chain_status(states: dict[int, bool], positions: dict[int, int]) -> str:
    if all(states.values()):
        return "ALL SERVOS CONNECTED"
    if not any(states.values()):
        return "ALL SERVOS DISCONNECTED"

    # Report the start of the missing downstream suffix, not a one-poll miss on a lower ID.
    ids = tuple(states)
    first_missing = None
    for sid in reversed(ids):
        if states[sid]:
            break
        first_missing = sid
    if first_missing is None:
        first_missing = next(sid for sid in ids if not states[sid])
    return (f"FIRST MISSING: ID{first_missing} {JOINT_NAMES[first_missing]} "
            f"at {positions.get(first_missing, '?')}")


def run(args) -> int:
    if args.interval <= 0:
        print("ERROR: --interval must be greater than zero")
        return 1

    bus, port = open_bus(args)
    positions: dict[int, int] = {}
    previous: dict[int, bool] | None = None
    ids = tuple(range(1, args.joints + 1))

    try:
        print(f"Port: {port}  Watching IDs 1-{args.joints}")
        print("Ensure torque is off, then move the arm slowly by hand.")
        print("Press Ctrl-C to stop.\n")

        while True:
            states = poll_chain(bus, ids, positions)
            if previous is None:
                print(f"Initial: {chain_status(states, positions)}", flush=True)
            elif states != previous:
                stamp = datetime.now().astimezone().isoformat(timespec="seconds")
                print(f"{stamp}  {chain_status(states, positions)}", flush=True)
            previous = states
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        bus.close()
    return 0
