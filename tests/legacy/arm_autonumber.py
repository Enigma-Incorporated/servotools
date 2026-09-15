#!/usr/bin/env python3
"""Auto-number the servos of an ALREADY-ASSEMBLED arm by physical order.

The software NEVER commands motion (safe on a built arm). Instead:

  1. SCAN + SPLIT  -- detect every servo and any duplicate-ID collisions,
     agnostic to initial IDs. Collisions are resolved with a reboot (instr
     0x08) followed by a hardware-paced sweep of set-ID bursts across the
     wake window, which commands no motion and needs no precise host timing
     (works from a Raspberry Pi). (A reboot does make a servo briefly limp
     ~0.8s, and resolving collisions needs reboots -- so if there ARE
     collisions, support the arm.)

  2. WIGGLE-ORDER  -- torque is disabled so you can back-drive the joints by
     hand. You wiggle each joint ONCE, in the order you want them numbered
     (e.g. base -> tip). The script watches every servo's encoder and detects
     which one you moved (a hand-wiggle = hundreds of position counts; the
     encoder reports angle even with torque off). That is the order.

  3. RENUMBER  -- assign IDs base..base+N-1 in wiggle order, via temp IDs
     (collision-free, motion-free). Verify.

Usage:
    python auto_ids/arm_autonumber.py                 # IDs 1..N in wiggle order
    python auto_ids/arm_autonumber.py --base 1 --max-id 20
    python auto_ids/arm_autonumber.py --self-test     # BENCH ONLY: self-nudges
                                                       # to validate detection
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time

import serial as pyserial

from _common import enable_high_res_timers, find_port
import chain_autonumber as ca
from chain_autonumber import (
    ADDR_GOAL,
    ADDR_POS,
    ADDR_TORQUE,
    ADDR_VOLT,
    BAUD,
    probe_id,
    read_reg,
    renumber,
    split_all,
    write_reg,
    write_word,
)

POS_MAX = 4096
MOVE_THRESHOLD = 80      # counts (~7 deg). hand-wiggle is far more; noise ~few.


def circ_delta(a, b) -> int:
    d = abs(int(a) - int(b)) % POS_MAX
    return min(d, POS_MAX - d)


def read_pos(ser, sid):
    return read_reg(ser, sid, ADDR_POS, 2)


def stable_pos(ser, sid, n=5):
    vs = [v for _ in range(n) if (v := read_pos(ser, sid)) is not None]
    return statistics.median(vs) if vs else None


def scan_arm(ser) -> dict[int, str]:
    """Detect all servos and collisions across the candidate ID range."""
    return {sid: st for sid in ca.CANDIDATE_IDS
            if (st := probe_id(ser, sid)) != "EMPTY"}


def disable_torque(ser, ids):
    for s in ids:
        write_reg(ser, s, ADDR_TORQUE, bytes([0]), settle=0.02)


def _pause(yes, msg):
    if yes:
        print(msg + "  [auto]")
    else:
        input(msg)


def wait_for_wiggle(ser, remaining, assigned, baseline):
    """Block until ONE unassigned servo shows a sustained, dominant position
    change. Returns (id, delta). Hints if an already-assigned servo is moved."""
    last_hint = 0.0
    while True:
        deltas = {}
        for s in remaining:
            p = read_pos(ser, s)
            if p is not None:
                deltas[s] = circ_delta(p, baseline[s])
        if deltas:
            top = max(deltas, key=deltas.get)
            if deltas[top] > MOVE_THRESHOLD:
                others = sorted((d for k, d in deltas.items() if k != top), reverse=True)
                dominant = (not others) or others[0] < MOVE_THRESHOLD \
                    or deltas[top] > 1.8 * others[0]
                ok = sum(1 for _ in range(4)
                         if (p := read_pos(ser, top)) is not None
                         and circ_delta(p, baseline[top]) > MOVE_THRESHOLD)
                if dominant and ok >= 3:
                    return top, deltas[top]
        now = time.time()
        if now - last_hint > 3.0:
            for s in assigned:
                p = read_pos(ser, s)
                if p is not None and circ_delta(p, baseline.get(s, p)) > MOVE_THRESHOLD:
                    print(f"        (ID {s} is already assigned — wiggle a NEW joint)")
                    break
            last_hint = now
        time.sleep(0.01)


def ensure_split(ser, yes=False):
    """Return sorted distinct IDs, splitting collisions if present."""
    state = scan_arm(ser)
    print("  found: " + (", ".join(f"ID{s}:{state[s]}" for s in sorted(state)) or "(nothing)"))
    if not state:
        print("ERROR: no servos found. Check power/cable/baud.")
        return None
    doubles = [s for s, st in state.items() if st == "DOUBLE"]
    if doubles:
        print(f"  duplicate-ID collisions at {doubles}; resolving with a reboot + ID sweep.")
        print("  NOTE: this reboots servos -> the arm goes briefly limp. SUPPORT IT.")
        _pause(yes, "  Press Enter to resolve collisions...")
        ids, leftover = split_all(ser)
        if leftover:
            print(f"ERROR: could not fully split (still colliding at {leftover}). Rerun.")
            return None
        return ids
    return sorted(state)


def wiggle_order(ser, ids, base, yes=False):
    print("\nSTEP 2/3 — set the order by wiggling")
    print("  The arm is about to go LIMP so you can move it by hand — HOLD IT.")
    _pause(yes, "  Press Enter when you're ready...")
    disable_torque(ser, ids)

    baseline = {s: stable_pos(ser, s) for s in ids}
    if any(v is None for v in baseline.values()):
        print("ERROR: could not read all servo positions. Rerun.")
        return None
    print(f"\n  Now WIGGLE EACH JOINT BY HAND, ONE AT A TIME, in the order you want them\n"
          f"  numbered (normally base first, gripper last). Give each a clear\n"
          f"  back-and-forth (~20 degrees), then stop and do the next one.\n")

    order = []
    remaining = set(ids)
    for k in range(len(ids)):
        print(f"  Joint {k + 1} of {len(ids)} — wiggle it now...", flush=True)
        sid, d = wait_for_wiggle(ser, remaining, set(order), baseline)
        order.append(sid)
        remaining.discard(sid)
        print("     got it.")
        time.sleep(0.5)
        for r in remaining:               # re-baseline (joints may have shifted)
            baseline[r] = stable_pos(ser, r) or baseline[r]
        baseline[sid] = stable_pos(ser, sid) or baseline[sid]
    return order


def do_renumber(ser, order, base, yes=False):
    mapping = {order[i]: base + i for i in range(len(order))}
    print("\nSTEP 3/3 — saving IDs to match the order you wiggled")
    for i, o in enumerate(order):
        was = "" if o == base + i else f"   (was ID {o})"
        print(f"  joint {base + i}  ->  ID {base + i}{was}")
    if not yes and input("  Press Enter to save (or 'n' to cancel): ").strip().lower() == "n":
        print("  cancelled; no IDs changed.")
        return False
    ok = renumber(ser, mapping)
    print("\nVerification:")
    good = ok
    for fid in range(base, base + len(order)):
        st = probe_id(ser, fid, n=8)
        print(f"  ID {fid}: {st}")
        if st != "SINGLE":
            good = False
    return good


def self_test(ser, base):
    """BENCH ONLY: nudge each servo (reversed ID order) to simulate a human
    wiggle and confirm detection recovers the order. Uses motion -- never run
    on an assembled arm you don't want moved."""
    ids = ensure_split(ser)
    if not ids:
        return
    print(f"\nSELF-TEST (bench): will nudge {len(ids)} servos and check detection.")
    disable_torque(ser, ids)
    test_order = list(reversed(ids))      # non-trivial order to recover
    baseline = {s: stable_pos(ser, s) for s in ids}
    remaining = set(ids)
    detected = []
    for X in test_order:
        p0 = read_pos(ser, X)
        delta = -350 if p0 > 2048 else 350
        tgt = max(60, min(4035, p0 + delta))
        write_reg(ser, X, ADDR_TORQUE, bytes([1]), settle=0.02)
        write_word(ser, X, ADDR_GOAL, tgt)
        sid, d = wait_for_wiggle(ser, remaining, set(detected), baseline)
        detected.append(sid)
        remaining.discard(sid)
        mark = "OK" if sid == X else f"MISMATCH (expected {X})"
        print(f"  nudged ID{X} -> detected ID{sid}  [{mark}]")
        write_word(ser, X, ADDR_GOAL, p0)        # restore
        time.sleep(0.4)
        write_reg(ser, X, ADDR_TORQUE, bytes([0]), settle=0.02)
        for r in remaining:
            baseline[r] = stable_pos(ser, r) or baseline[r]
    correct = sum(1 for a, b in zip(detected, test_order) if a == b)
    print(f"\nself-test: detection recovered {correct}/{len(ids)} correctly "
          f"({'PASS' if correct == len(ids) else 'FAIL'})")
    print(f"  nudge order : {test_order}")
    print(f"  detected    : {detected}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Auto-number an assembled arm by hand-wiggle order")
    ap.add_argument("--port", default=None)
    ap.add_argument("--base", type=int, default=1, help="ID of the first wiggled joint")
    ap.add_argument("--max-id", type=int, default=90,
                    help="highest initial ID to scan for (the sweep split can park "
                         "servos anywhere up to 89, e.g. after an interrupted run)")
    ap.add_argument("--self-test", action="store_true", help="BENCH ONLY: self-nudge to validate detection")
    ap.add_argument("--scan-only", action="store_true", help="read-only: report servos/collisions/torque, no motion")
    ap.add_argument("--detect-only", action="store_true", help="wiggle-detect the order and print it; do NOT change IDs")
    ap.add_argument("--apply", default=None, help="renumber a known order without wiggling, e.g. --apply 3,1,2,4,6,5")
    ap.add_argument("--yes", action="store_true", help="skip interactive confirmations (for background/monitored runs)")
    args = ap.parse_args()

    enable_high_res_timers()   # Windows: sub-ms sleeps for the reboot-race (no-op on POSIX)

    ca.CANDIDATE_IDS = list(range(1, args.max_id + 1))   # widen scan for arbitrary IDs

    port = args.port or find_port()
    if port is None:
        print("ERROR: no adapter found.")
        sys.exit(1)
    ser = pyserial.Serial(port, BAUD, timeout=0.006)
    time.sleep(0.05)
    print(f"Port: {port}  baud {BAUD}\n")

    if args.scan_only:
        print("SCAN-ONLY (read-only, no motion):")
        state = scan_arm(ser)
        if not state:
            print("  no servos found.")
        for sid in sorted(state):
            if state[sid] == "DOUBLE":
                print(f"  ID {sid}: DOUBLE (>=2 servos collide here)")
            else:
                pos = read_reg(ser, sid, ADDR_POS, 2)
                trq = read_reg(ser, sid, ADDR_TORQUE, 1)
                volt = read_reg(ser, sid, ADDR_VOLT, 1)
                tq = {0: "off", 1: "ON"}.get(trq, trq)
                v = f"{volt / 10:.1f}V" if volt is not None else "?"
                print(f"  ID {sid}: SINGLE  pos={pos}  torque={tq}  {v}")
        doubles = [s for s, st in state.items() if st == "DOUBLE"]
        singles = [s for s, st in state.items() if st == "SINGLE"]
        print(f"\n  {len(singles)} distinct + {len(doubles)} collided ID(s).")
        if any(read_reg(ser, s, ADDR_TORQUE, 1) for s in singles):
            print("  NOTE: some servos have torque ON (arm is actively held).")
        ser.close()
        return

    if args.self_test:
        self_test(ser, args.base)
        ser.close()
        return

    print("=== SO-arm servo auto-numbering ===")
    print("Numbers the servos by physical position. Nothing moves on its own —")
    print("you'll wiggle each joint by hand when asked.\n")
    print("STEP 1/3 — finding servos...")
    ids = ensure_split(ser, yes=args.yes)
    if not ids:
        ser.close()
        sys.exit(1)
    print(f"  -> {len(ids)} servo(s) on distinct IDs: {ids}")

    # --apply: renumber a known order (no wiggling)
    if args.apply:
        order = [int(x) for x in args.apply.split(",") if x.strip()]
        if sorted(order) != sorted(ids):
            print(f"ERROR: --apply order {order} doesn't match found IDs {ids}.")
            ser.close()
            sys.exit(1)
        good = do_renumber(ser, order, args.base, yes=args.yes)
        ser.close()
        print("\n" + ("DONE." if good else "WARNINGS — re-scan."))
        sys.exit(0 if good else 1)

    if len(ids) == 1:
        renumber(ser, {ids[0]: args.base})
        print(f"Single servo -> ID {args.base}. Done.")
        ser.close()
        return

    order = wiggle_order(ser, ids, args.base, yes=args.yes)
    if order is None:
        ser.close()
        sys.exit(1)

    if args.detect_only:
        print(f"\nDETECTED ORDER (in the sequence you wiggled): {order}")
        print(f"  No IDs changed. To commit this mapping, run:")
        print(f"    python auto_ids/arm_autonumber.py --apply {','.join(map(str, order))} --base {args.base}")
        ser.close()
        return

    good = do_renumber(ser, order, args.base, yes=args.yes)
    ser.close()
    print("\n" + ("DONE — IDs now match your wiggle order." if good
                  else "FINISHED WITH WARNINGS — re-scan."))
    sys.exit(0 if good else 1)


if __name__ == "__main__":
    main()
