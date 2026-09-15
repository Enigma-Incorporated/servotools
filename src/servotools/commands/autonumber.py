"""servo autonumber -- number the joints by physical position. See docs/autonumber.md."""

from __future__ import annotations

import statistics
import time

from .. import registers as reg
from ..bus import DOUBLE, SINGLE, Bus
from ..cli import add_port_option, confirm, resolve_port
from ..config import BAUD, POSITION_MAX
from ..renumber import renumber
from ..split import split_all
from ..timers import enable_high_res_timers

MOVE_THRESHOLD = 80  # counts, ~7 degrees; a hand-wiggle is far more, noise is a few
DOMINANCE = 1.8  # the moved joint must beat the runner-up by this factor
CONFIRMATIONS = 4  # re-reads used to confirm a candidate
CONFIRMATIONS_NEEDED = 3
HINT_INTERVAL_S = 3.0


def add_parser(sub) -> None:
    p = sub.add_parser("autonumber", help="number the joints by physical position")
    add_port_option(p)
    p.add_argument("--base", type=int, default=1, help="ID of the first wiggled joint")
    p.add_argument("--max-id", type=int, default=90,
                   help="highest initial ID to scan for (an interrupted sweep can park "
                        "servos anywhere up to 89)")
    p.add_argument("--scan-only", action="store_true",
                   help="read-only: report servos, collisions and torque; no motion")
    p.add_argument("--split-only", action="store_true",
                   help="resolve duplicate IDs and stop; skip the wiggle ordering")
    p.add_argument("--detect-only", action="store_true",
                   help="wiggle-detect the order and print it; do NOT change IDs")
    p.add_argument("--apply", default=None,
                   help="renumber a known order without wiggling, e.g. --apply 3,1,2,4,6,5")
    p.add_argument("--yes", action="store_true", help="skip interactive confirmations")
    p.set_defaults(run=run)


def circ_delta(a, b) -> int:
    d = abs(int(a) - int(b)) % POSITION_MAX
    return min(d, POSITION_MAX - d)


def read_pos(bus: Bus, sid: int):
    return bus.read(sid, reg.PRESENT_POSITION, 2)


def stable_pos(bus: Bus, sid: int, n: int = 5):
    vs = [v for _ in range(n) if (v := read_pos(bus, sid)) is not None]
    return statistics.median(vs) if vs else None


def disable_torque(bus: Bus, ids) -> None:
    for s in ids:
        bus.write_byte(s, reg.TORQUE_ENABLE, 0, settle=0.02)


def wait_for_wiggle(bus: Bus, remaining, assigned, baseline):
    """Block until one unassigned servo shows a sustained, dominant position change."""
    last_hint = 0.0
    while True:
        deltas = {}
        for s in remaining:
            p = read_pos(bus, s)
            if p is not None:
                deltas[s] = circ_delta(p, baseline[s])
        if deltas:
            top = max(deltas, key=deltas.get)
            if deltas[top] > MOVE_THRESHOLD:
                others = sorted((d for k, d in deltas.items() if k != top), reverse=True)
                dominant = (not others) or others[0] < MOVE_THRESHOLD \
                    or deltas[top] > DOMINANCE * others[0]
                ok = sum(1 for _ in range(CONFIRMATIONS)
                         if (p := read_pos(bus, top)) is not None
                         and circ_delta(p, baseline[top]) > MOVE_THRESHOLD)
                if dominant and ok >= CONFIRMATIONS_NEEDED:
                    return top, deltas[top]
        now = time.time()
        if now - last_hint > HINT_INTERVAL_S:
            for s in assigned:
                p = read_pos(bus, s)
                if p is not None and circ_delta(p, baseline.get(s, p)) > MOVE_THRESHOLD:
                    print(f"        (ID {s} is already assigned — wiggle a NEW joint)")
                    break
            last_hint = now
        time.sleep(0.01)


def ensure_split(bus: Bus, ids, yes: bool = False):
    """Return the sorted distinct IDs, resolving collisions first if there are any."""
    state = bus.scan(ids)
    print("  found: " + (", ".join(f"ID{s}:{state[s]}" for s in sorted(state)) or "(nothing)"))
    if not state:
        print("ERROR: no servos found. Check power/cable/baud.")
        return None
    doubles = [s for s, st in state.items() if st == DOUBLE]
    if doubles:
        print(f"  duplicate-ID collisions at {doubles}; resolving with a reboot + ID sweep.")
        print("  NOTE: this reboots servos -> the arm goes briefly limp. SUPPORT IT.")
        confirm(yes, "  Press Enter to resolve collisions...")
        found, leftover = split_all(bus, ids)
        if leftover:
            print(f"ERROR: could not fully split (still colliding at {leftover}). Rerun.")
            return None
        return found
    return sorted(state)


def wiggle_order(bus: Bus, ids, yes: bool = False):
    print("\nSTEP 2/3 — set the order by wiggling")
    print("  The arm is about to go LIMP so you can move it by hand — HOLD IT.")
    confirm(yes, "  Press Enter when you're ready...")
    disable_torque(bus, ids)

    baseline = {s: stable_pos(bus, s) for s in ids}
    if any(v is None for v in baseline.values()):
        print("ERROR: could not read all servo positions. Rerun.")
        return None
    print("\n  Now WIGGLE EACH JOINT BY HAND, ONE AT A TIME, in the order you want them\n"
          "  numbered (normally base first, gripper last). Give each a clear\n"
          "  back-and-forth (~20 degrees), then stop and do the next one.\n")

    order = []
    remaining = set(ids)
    for k in range(len(ids)):
        print(f"  Joint {k + 1} of {len(ids)} — wiggle it now...", flush=True)
        sid, _ = wait_for_wiggle(bus, remaining, set(order), baseline)
        order.append(sid)
        remaining.discard(sid)
        print("     got it.")
        time.sleep(0.5)
        for r in remaining:  # re-baseline: neighbouring joints may have shifted
            baseline[r] = stable_pos(bus, r) or baseline[r]
        baseline[sid] = stable_pos(bus, sid) or baseline[sid]
    return order


def do_renumber(bus: Bus, order, base: int, yes: bool = False) -> bool:
    mapping = {order[i]: base + i for i in range(len(order))}
    print("\nSTEP 3/3 — saving IDs to match the order you wiggled")
    for i, o in enumerate(order):
        was = "" if o == base + i else f"   (was ID {o})"
        print(f"  joint {base + i}  ->  ID {base + i}{was}")
    if not confirm(yes, "  Press Enter to save (or 'n' to cancel): "):
        print("  cancelled; no IDs changed.")
        return False
    ok = renumber(bus, mapping)
    print("\nVerification:")
    good = ok
    for fid in range(base, base + len(order)):
        st = bus.probe(fid, n=8)
        print(f"  ID {fid}: {st}")
        if st != SINGLE:
            good = False
    return good


def scan_only(bus: Bus, ids) -> None:
    print("SCAN-ONLY (read-only, no motion):")
    state = bus.scan(ids)
    if not state:
        print("  no servos found.")
    for sid in sorted(state):
        if state[sid] == DOUBLE:
            print(f"  ID {sid}: DOUBLE (>=2 servos collide here)")
        else:
            pos = bus.read(sid, reg.PRESENT_POSITION, 2)
            trq = bus.read(sid, reg.TORQUE_ENABLE, 1)
            volt = bus.read(sid, reg.VOLTAGE, 1)
            tq = {0: "off", 1: "ON"}.get(trq, trq)
            v = f"{volt / 10:.1f}V" if volt is not None else "?"
            print(f"  ID {sid}: SINGLE  pos={pos}  torque={tq}  {v}")
    doubles = [s for s, st in state.items() if st == DOUBLE]
    singles = [s for s, st in state.items() if st == SINGLE]
    print(f"\n  {len(singles)} distinct + {len(doubles)} collided ID(s).")
    if any(bus.read(s, reg.TORQUE_ENABLE, 1) for s in singles):
        print("  NOTE: some servos have torque ON (arm is actively held).")


def run(args) -> int:
    enable_high_res_timers()  # Windows: the sweep needs sub-ms sleeps
    ids = list(range(1, args.max_id + 1))
    port = resolve_port(args)
    bus = Bus.open(port, BAUD)
    print(f"Port: {port}  baud {BAUD}\n")

    if args.scan_only:
        scan_only(bus, ids)
        bus.close()
        return 0

    print("=== SO-arm servo auto-numbering ===")
    print("Numbers the servos by physical position. Nothing moves on its own —")
    print("you'll wiggle each joint by hand when asked.\n")
    print("STEP 1/3 — finding servos...")
    found = ensure_split(bus, ids, yes=args.yes)
    if not found:
        bus.close()
        return 1
    print(f"  -> {len(found)} servo(s) on distinct IDs: {found}")

    if args.split_only:
        bus.close()
        print("\nDONE — collisions resolved. IDs are arbitrary; to number them, run:")
        print("    servo autonumber                       (wiggle each joint in order)")
        print(f"    servo autonumber --apply {','.join(map(str, found))} --yes   (keep this order)")
        return 0

    if args.apply:
        order = [int(x) for x in args.apply.split(",") if x.strip()]
        if sorted(order) != sorted(found):
            print(f"ERROR: --apply order {order} doesn't match found IDs {found}.")
            bus.close()
            return 1
        good = do_renumber(bus, order, args.base, yes=args.yes)
        bus.close()
        print("\n" + ("DONE." if good else "WARNINGS — re-scan."))
        return 0 if good else 1

    if len(found) == 1:
        renumber(bus, {found[0]: args.base})
        print(f"Single servo -> ID {args.base}. Done.")
        bus.close()
        return 0

    order = wiggle_order(bus, found, yes=args.yes)
    if order is None:
        bus.close()
        return 1

    if args.detect_only:
        print(f"\nDETECTED ORDER (in the sequence you wiggled): {order}")
        print("  No IDs changed. To commit this mapping, run:")
        print(f"    servo autonumber --apply {','.join(map(str, order))} --base {args.base}")
        bus.close()
        return 0

    good = do_renumber(bus, order, args.base, yes=args.yes)
    bus.close()
    print("\n" + ("DONE — IDs now match your wiggle order." if good
                  else "FINISHED WITH WARNINGS — re-scan."))
    return 0 if good else 1
