"""servo flatten -- force every servo onto one ID, a test fixture for `servo autonumber`."""

from __future__ import annotations

import time

from .. import registers as reg
from ..bus import DOUBLE, EMPTY, SINGLE, Bus
from ..cli import add_port_option, confirm, resolve_port
from ..config import BAUD, BROADCAST_ID, STANDARD_BAUDS


def add_parser(sub) -> None:
    p = sub.add_parser("flatten", help="collide every servo onto one ID (test fixture)")
    add_port_option(p)
    p.add_argument("--id", type=int, default=1, dest="target",
                   help="ID to give every servo (default 1)")
    p.add_argument("--baud", type=int, default=BAUD,
                   help=f"bus baud (default {BAUD}; autonumber needs 1 Mbps)")
    p.add_argument("--max-id", type=int, default=20,
                   help="highest ID to scan when reporting the bus (the broadcast "
                        "flatten covers every ID regardless)")
    p.add_argument("--scan-only", action="store_true",
                   help="read-only: report the bus, change nothing")
    p.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    p.set_defaults(run=run)


def describe(state: dict[int, str]) -> str:
    if not state:
        return "(nothing)"
    return ", ".join(f"ID{s}:{state[s]}" for s in sorted(state))


def flatten(bus: Bus, target: int) -> None:
    """Broadcast every servo onto `target`. Motion-free; only the ID register changes."""
    bus.write_byte(BROADCAST_ID, reg.TORQUE_ENABLE, 0, settle=0.05)
    bus.write_byte(BROADCAST_ID, reg.LOCK, 0, settle=0.05)
    bus.write_byte(BROADCAST_ID, reg.ID, target, settle=0.20)
    bus.write_byte(BROADCAST_ID, reg.LOCK, 1, settle=0.05)


def find_servos_any_baud(bus: Bus, ids):
    """Tell a baud mismatch apart from a dead bus."""
    for b in STANDARD_BAUDS:
        bus.baudrate = b
        time.sleep(0.05)
        if st := bus.scan(ids):
            return b, st
    bus.baudrate = BAUD
    return None, {}


def run(args) -> int:
    if not 1 <= args.target <= 253:
        print("ERROR: --id must be 1-253")
        return 1

    port = resolve_port(args)
    bus = Bus.open(port, args.baud)
    print(f"Port: {port}  baud {args.baud}\n")

    scan_ids = range(1, args.max_id + 1)

    print("Scanning the bus...")
    before = bus.scan(scan_ids)
    print("  found: " + describe(before))

    if not before:
        alt_baud, alt_state = find_servos_any_baud(bus, scan_ids)
        if alt_state:
            print(f"\n  servos ARE present at baud {alt_baud}: {describe(alt_state)}")
            print(f"  autonumber needs 1 Mbps -- re-run with --baud {alt_baud} to flatten")
            print("  there, or re-flash the servos to 1 Mbps first.")
        else:
            print("\nERROR: no servos on any standard baud. Check 12V power + bus cable.")
        bus.close()
        return 0 if args.scan_only else 1

    if args.scan_only:
        doubles = [s for s, st in before.items() if st == DOUBLE]
        singles = [s for s, st in before.items() if st == SINGLE]
        print(f"\n  {len(singles)} distinct + {len(doubles)} collided ID(s). (read-only)")
        bus.close()
        return 0

    print(f"\nWill BROADCAST every servo on the bus -> ID {args.target}.")
    print("Nothing moves (ID register only). With >=2 servos this creates a")
    print(f"deliberate ID-{args.target} collision -- the mess autonumber resolves.")
    if not confirm(args.yes, "Press Enter to flatten (or 'n' to cancel): "):
        print("cancelled; no IDs changed.")
        bus.close()
        return 0

    flatten(bus, args.target)

    print("\nVerifying...")
    after = bus.scan(scan_ids)
    target_state = after.get(args.target, EMPTY)
    strays = {s: st for s, st in after.items() if s != args.target}
    print(f"  ID {args.target}: {target_state}")
    if strays:
        print(f"  leftover IDs (did NOT flatten): {describe(strays)}")
    bus.close()

    print()
    if not strays and target_state == DOUBLE:
        print(f"DONE -- all servos now collide at ID {args.target}. Next:  servo autonumber")
        return 0
    if not strays and target_state == SINGLE:
        print(f"DONE -- one servo at ID {args.target} (single-servo bus). Next:  servo autonumber")
        return 0

    print("FINISHED WITH WARNINGS:")
    if target_state == EMPTY:
        print(f"  - nothing answers at ID {args.target}; "
              "the broadcast may not have committed. Re-run.")
    if strays:
        print(f"  - some servos kept their old IDs: {describe(strays)}. Re-run.")
    return 1
