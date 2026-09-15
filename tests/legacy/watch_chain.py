#!/usr/bin/env python3
"""Watch an SO-101 servo chain for pose-dependent communication failures."""

from __future__ import annotations

import argparse
from datetime import datetime
import sys
import time

import scservo_sdk as scs

from _common import JOINT_NAMES, find_port

IDS = tuple(range(1, 7))
POSITION_ADDR = 56
BAUD = 1_000_000


def poll_chain(pk, ph, ids: tuple[int, ...], positions: dict[int, int]) -> dict[int, bool]:
    """Poll every servo once, preserving the last position when a read fails."""
    states = {}
    for sid in ids:
        position, comm_result, _ = pk.read2ByteTxRx(ph, sid, POSITION_ADDR)
        states[sid] = comm_result == 0
        if states[sid]:
            positions[sid] = position
    return states


def chain_status(states: dict[int, bool], positions: dict[int, int]) -> str:
    if all(states.values()):
        return "ALL SERVOS CONNECTED"
    if not any(states.values()):
        return "ALL SERVOS DISCONNECTED"

    # Report the start of the missing downstream suffix, not an unrelated
    # one-poll miss on a lower ID.
    ids = tuple(states)
    first_missing = None
    for sid in reversed(ids):
        if states[sid]:
            break
        first_missing = sid
    if first_missing is None:
        first_missing = next(sid for sid in ids if not states[sid])
    return (
        f"FIRST MISSING: ID{first_missing} {JOINT_NAMES[first_missing]} "
        f"at {positions.get(first_missing, '?')}"
    )


def run_watch(port: str, interval: float, joint_count: int) -> bool:
    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    if not ph.openPort():
        print(f"ERROR: Cannot open port {port}")
        return False
    if not ph.setBaudRate(BAUD):
        print(f"ERROR: Cannot set {port} to {BAUD} baud")
        ph.closePort()
        return False

    positions: dict[int, int] = {}
    previous: dict[int, bool] | None = None
    ids = tuple(range(1, joint_count + 1))

    try:
        print(f"Port: {port}  Watching IDs 1-{joint_count}")
        print("Ensure torque is off, then move the arm slowly by hand.")
        print("Press Ctrl-C to stop.\n")

        while True:
            states = poll_chain(pk, ph, ids, positions)
            if previous is None:
                print(f"Initial: {chain_status(states, positions)}", flush=True)
            elif states != previous:
                print(
                    f"{datetime.now().astimezone().isoformat(timespec='seconds')}  "
                    f"{chain_status(states, positions)}",
                    flush=True,
                )

            previous = states
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ph.closePort()

    return True


def self_test() -> None:
    class FakePacket:
        def read2ByteTxRx(self, _ph, sid, _addr):
            return {1: (101, 0, 0), 2: (0, -1, 0)}.get(sid, (100 + sid, 0, 0))

    positions = {2: 202}
    states = poll_chain(FakePacket(), None, IDS, positions)
    assert states == {1: True, 2: False, 3: True, 4: True, 5: True, 6: True}
    assert positions[1] == 101 and positions[2] == 202
    assert "ID2 shoulder_lift" in chain_status(states, positions)
    assert "ID4 wrist_flex" in chain_status(
        {1: False, 2: True, 3: True, 4: False, 5: False, 6: False},
        positions,
    )
    print("self-test passed")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Log servo-chain connectivity changes while moving the arm by hand"
    )
    parser.add_argument("--port", default=None, help="Serial port (auto-detected if omitted)")
    parser.add_argument("--joints", type=int, choices=range(1, 7), default=6,
                        help="number of installed joints, starting at ID 1 (default: 6)")
    parser.add_argument("--interval", type=float, default=0.05,
                        help="seconds between full-chain polls (default: 0.05)")
    parser.add_argument("--self-test", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return
    if args.interval <= 0:
        parser.error("--interval must be greater than zero")

    port = args.port or find_port()
    if port is None:
        print("ERROR: No SO-101 device found. Is the USB cable connected?")
        sys.exit(1)
    sys.exit(0 if run_watch(port, args.interval, args.joints) else 1)


if __name__ == "__main__":
    main()
