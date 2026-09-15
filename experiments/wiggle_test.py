#!/usr/bin/env python3
"""Wiggle each servo in turn so you can SEE which physical joint is which ID.

The reboot-race assigns IDs by boot speed, not chain position -- so this is
how you map ID -> physical servo. Each servo nudges ~22 degrees and returns,
one at a time, with a banner. Watch which one moves.

Usage:
    python wiggle_test.py            # wiggles IDs 1 2 3
    python wiggle_test.py 1 2        # wiggles only the given IDs
"""

from __future__ import annotations

import sys
import time

import scservo_sdk as scs

from _common import JOINT_NAMES, find_port

ADDR_TORQUE = 40
ADDR_GOAL = 42
ADDR_POS = 56
ADDR_VOLT = 62

DELTA = 250  # ~22 degrees (0.088 deg/step)


def main() -> None:
    ids = [int(x) for x in sys.argv[1:]] or [1, 2, 3]
    port = find_port()
    if port is None:
        print("ERROR: no adapter")
        sys.exit(1)
    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    if not ph.openPort():
        print("ERROR: cannot open port")
        sys.exit(1)
    ph.setBaudRate(1_000_000)

    print(f"Port: {port}")
    # Voltage gradient hint (servo nearest power reads highest).
    print("Voltage per ID (higher ~ closer to power injection):")
    for sid in ids:
        v, res, _ = pk.read1ByteTxRx(ph, sid, ADDR_VOLT)
        print(f"  ID {sid}: {v/10:.1f}V" if res == 0 else f"  ID {sid}: no read")
    print("\nNow wiggling each servo ~22deg, one at a time. Watch which moves.\n")

    for rnd in range(2):
        print(f"=== round {rnd + 1} ===")
        for sid in ids:
            pos, res, _ = pk.read2ByteTxRx(ph, sid, ADDR_POS)
            if res != 0:
                print(f"  ID {sid}: no clean read, skipping")
                continue
            print(f">>> MOVING ID {sid} ({JOINT_NAMES.get(sid, '?')}) — start pos {pos}",
                  flush=True)
            pk.write1ByteTxRx(ph, sid, ADDR_TORQUE, 1)
            time.sleep(0.05)
            for target in (pos + DELTA, pos - DELTA, pos):
                t = max(50, min(4045, target))
                pk.write2ByteTxRx(ph, sid, ADDR_GOAL, t)
                time.sleep(0.45)
            pk.write1ByteTxRx(ph, sid, ADDR_TORQUE, 0)
            time.sleep(0.9)
        print()

    ph.closePort()
    print("Done. Tell me the physical order you saw "
          "(e.g. 'nearest the adapter -> ID 2, then ID 1, then ID 3').")


if __name__ == "__main__":
    main()
