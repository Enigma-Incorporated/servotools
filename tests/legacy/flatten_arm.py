#!/usr/bin/env python3
"""Force EVERY connected servo onto one ID (default 1) -- a test fixture for
./autonumber.

This is the deliberate "make a mess" step: it collapses all the arm's servos
onto a single ID so you can then run ./autonumber and watch it split the
collision apart and renumber the joints by physical position.

It works by BROADCASTING the ID write to 0xFE (every servo on the bus answers
to the broadcast ID, whatever its real ID), so it is agnostic to:
  - how many servos there are (4/5/6 -- gripper and/or wrist-roll may be absent)
  - whatever IDs they currently hold
  - servos that already collide on a shared ID
Setting them one-by-one would break the instant two of them share ID 1 and can
no longer be addressed apart; the broadcast moves them all at once.

NOTHING MOVES. We only write the ID register (EEPROM): torque off -> unlock ->
write ID -> re-lock. Broadcast writes get no status reply, so the proof it took
is the post-scan: >=2 servos now collide at the target ID (a DOUBLE).

Usage:
    python auto_ids/flatten_arm.py              # set every servo -> ID 1
    python auto_ids/flatten_arm.py --id 1       # explicit
    python auto_ids/flatten_arm.py --scan-only  # read-only: report the bus
    python auto_ids/flatten_arm.py --yes        # skip the confirmation prompt
"""

from __future__ import annotations

import argparse
import sys
import time

import serial as pyserial

from _common import STANDARD_BAUDS, find_port
from chain_autonumber import (
    ADDR_ID,
    ADDR_LOCK,
    ADDR_TORQUE,
    BAUD,
    probe_id,
    write_reg,
)

BROADCAST_ID = 0xFE


def scan(ser, ids) -> dict[int, str]:
    """Read-only: classify each ID as SINGLE / DOUBLE, dropping EMPTY."""
    return {sid: st for sid in ids if (st := probe_id(ser, sid)) != "EMPTY"}


def describe(state: dict[int, str]) -> str:
    if not state:
        return "(nothing)"
    return ", ".join(f"ID{s}:{state[s]}" for s in sorted(state))


def flatten(ser, target: int) -> None:
    """Broadcast (ID 0xFE) every servo onto `target`. Motion-free: torque off,
    unlock EEPROM, write the ID, re-lock. The broadcast reaches every servo
    regardless of its current (possibly colliding) ID."""
    write_reg(ser, BROADCAST_ID, ADDR_TORQUE, bytes([0]), settle=0.05)
    write_reg(ser, BROADCAST_ID, ADDR_LOCK, bytes([0]), settle=0.05)
    write_reg(ser, BROADCAST_ID, ADDR_ID, bytes([target]), settle=0.20)
    write_reg(ser, BROADCAST_ID, ADDR_LOCK, bytes([1]), settle=0.05)


def find_servos_any_baud(ser, ids):
    """Diagnostic for the 'nothing here' path: report which baud (if any) has
    servos, so a baud mismatch is distinguishable from a dead/unpowered bus."""
    for b in STANDARD_BAUDS:
        ser.baudrate = b
        time.sleep(0.05)
        if st := scan(ser, ids):
            return b, st
    ser.baudrate = BAUD
    return None, {}


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Force all connected servos onto one ID (test fixture for ./autonumber)")
    ap.add_argument("--port", default=None)
    ap.add_argument("--id", type=int, default=1, dest="target",
                    help="ID to give every servo (default 1)")
    ap.add_argument("--baud", type=int, default=BAUD,
                    help=f"bus baud (default {BAUD}; ./autonumber needs 1 Mbps)")
    ap.add_argument("--max-id", type=int, default=20,
                    help="highest ID to scan when reporting the bus (the broadcast "
                         "flatten covers every ID regardless)")
    ap.add_argument("--scan-only", action="store_true",
                    help="read-only: report the bus, change nothing")
    ap.add_argument("--yes", action="store_true",
                    help="skip the confirmation prompt (for monitored runs)")
    args = ap.parse_args()

    if not 1 <= args.target <= 253:
        print("ERROR: --id must be 1-253")
        sys.exit(1)

    port = args.port or find_port()
    if port is None:
        print("ERROR: no Waveshare adapter found. Is the USB cable connected?")
        sys.exit(1)

    ser = pyserial.Serial(port, args.baud, timeout=0.006)
    time.sleep(0.05)
    print(f"Port: {port}  baud {args.baud}\n")

    scan_ids = range(1, args.max_id + 1)

    print("Scanning the bus...")
    before = scan(ser, scan_ids)
    print("  found: " + describe(before))

    if not before:
        # Nothing at this baud. Tell apart a baud mismatch from a dead bus.
        alt_baud, alt_state = find_servos_any_baud(ser, scan_ids)
        if alt_state:
            print(f"\n  servos ARE present at baud {alt_baud}: {describe(alt_state)}")
            print(f"  ./autonumber needs 1 Mbps -- re-run with --baud {alt_baud} to flatten")
            print("  there, or re-flash the servos to 1 Mbps first.")
        else:
            print("\nERROR: no servos on any standard baud. Check 12V power + bus cable.")
        ser.close()
        sys.exit(0 if args.scan_only else 1)

    if args.scan_only:
        doubles = [s for s, st in before.items() if st == "DOUBLE"]
        singles = [s for s, st in before.items() if st == "SINGLE"]
        print(f"\n  {len(singles)} distinct + {len(doubles)} collided ID(s). (read-only)")
        ser.close()
        return

    print(f"\nWill BROADCAST every servo on the bus -> ID {args.target}.")
    print("Nothing moves (ID register only). With >=2 servos this creates a")
    print(f"deliberate ID-{args.target} collision -- the mess ./autonumber resolves.")
    if not args.yes and input("Press Enter to flatten (or 'n' to cancel): ").strip().lower() == "n":
        print("cancelled; no IDs changed.")
        ser.close()
        return

    flatten(ser, args.target)

    print("\nVerifying...")
    after = scan(ser, scan_ids)
    target_state = after.get(args.target, "EMPTY")
    strays = {s: st for s, st in after.items() if s != args.target}
    print(f"  ID {args.target}: {target_state}")
    if strays:
        print(f"  leftover IDs (did NOT flatten): {describe(strays)}")
    ser.close()

    print()
    if not strays and target_state == "DOUBLE":
        print(f"DONE -- all servos now collide at ID {args.target}. Next:  ./autonumber")
    elif not strays and target_state == "SINGLE":
        print(f"DONE -- one servo at ID {args.target} (single-servo bus). Next:  ./autonumber")
    else:
        print("FINISHED WITH WARNINGS:")
        if target_state == "EMPTY":
            print(f"  - nothing answers at ID {args.target}; the broadcast may not have committed. Re-run.")
        if strays:
            print(f"  - some servos kept their old IDs: {describe(strays)}. Re-run.")
        sys.exit(1)


if __name__ == "__main__":
    main()
